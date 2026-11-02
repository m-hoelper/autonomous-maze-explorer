#!/usr/bin/env python3
import os
from searchforce.functions.paths import CACHE_DIR
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
from ros_robot_controller_msgs.msg import SetPWMServoState,PWMServoState
from std_msgs.msg import String
import logging
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # registers PointStamped for tf_buffer.transform; needed although unreferenced
from numba import njit
from nav_msgs.msg import OccupancyGrid
from collections import Counter

@njit
def _kadane2d(weights):
    rows, cols = weights.shape
    max_score = -1e308
    bt=bl=bb=br=0
    for left in range(cols):
        tmp = np.zeros(rows, dtype=np.float64)
        for right in range(left, cols):
            for r in range(rows):
                tmp[r] += weights[r, right]
            current = 0.0
            start = 0
            for r in range(rows):
                if current <= 0.0:
                    current = tmp[r]
                    start = r
                else:
                    current += tmp[r]
                if current > max_score:
                    max_score = current
                    bt, bl, bb, br = start, left, r, right
    return bt, bl, bb, br, max_score

class AirTagListener(Node):
    def __init__(self):
        super().__init__('airtag_listener_with_pose_and_color')
        self.latest_image = None     # Store latest image from camera
        self.bridge = CvBridge()
        self.found_timer = None
        self.dynamic_white_balcane=True
        self.scales=[1.0659836010963957, 1.106072530785276, 1.044802187740238]
        self.focal_length_px=None
        self.real_width=0.016
        self.image_shape=[480,640]
        self.object_positions = []
        self.object_colours = []

        self.WHITE_PENALTY = 2.0
        self.ANGLE_STEP = 1
        
        self.camera_set = self.create_publisher(SetPWMServoState,'/ros_robot_controller/pwm_servo/set_state',10)
        self.status_publisher = self.create_publisher(String,"searchforce_status",10)
        self.searchforce_status = 'searching'
        self.searching = True
        self.saved = False
        self._status_sub = self.create_subscription(String, '/searchforce_status', self.status_callback, 10)
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        msg = SetPWMServoState()

        servo_state = PWMServoState()
        servo_state.id = [1]  

        servo_state.position = [1500] #1505
        servo_state.offset = [] 

        msg.state = [servo_state] 
        msg.duration = 0.01

        self.camera_set.publish(msg)
        self.get_logger().info('Published servo state')

        msg = SetPWMServoState()

        servo_state = PWMServoState()
        servo_state.id = [2]  

        servo_state.position = [1465] #1465
        servo_state.offset = [] 

        msg.state = [servo_state] 
        msg.duration = 0.01

        self.camera_set.publish(msg)
        self.get_logger().info('Published servo state')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to AprilTag detection topic
        self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10
        )

        # Subscribe to rectified image
        self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10
        )
        self.create_subscription(
        CameraInfo,
        '/ascamera/camera_publisher/rgb0/camera_info',
        self.camera_info_callback,
        10
        )
        
        log_path = os.path.expanduser(os.path.join(CACHE_DIR, 'detection_log.txt'))
        logging.basicConfig(
            filename=log_path,
            filemode='w',  # 追加模式
            format='%(asctime)s - %(levelname)s - %(message)s',
            level=logging.INFO
        )
        self.logger = logging.getLogger("detection_log")

    def map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def status_callback(self, msg: String):
        self.searchforce_status = msg.data
        if self.searchforce_status == 'finish_detection':
            self.searching = False
            if not self.saved:
                if getattr(self, '_map', None) is None or len(self.object_positions) == 0:
                    self.get_logger().error('finish_detection received without a map or without detections, nothing saved', once=True)
                    return
                occ_grid = np.array(self._map.data).reshape(
                    (self._map.info.height, self._map.info.width))
                corners, angle, score = self.find_max_rotated_rectangle(
                    occ_grid, white_penalty=self.WHITE_PENALTY, angle_step=self.ANGLE_STEP)
                self.object_positions = np.array(self.object_positions)

                colour = Counter(self.object_colours).most_common(1)[0][0] if self.object_colours else 'unknown'

                p_orig = np.mean(self.object_positions, axis=0)
                # 2. Rotationswinkel definieren (in Radiant)
                theta = np.deg2rad(180-angle)                # Beispiel: 30° in Radiant

                # 3. Rotationsmatrix um den Ursprung
                R = np.array([
                    [np.cos(theta), -np.sin(theta)],
                    [np.sin(theta),  np.cos(theta)]
                ])

                # 4. Punkt rotieren
                p_rot = R @ p_orig

                with open(os.path.join(CACHE_DIR, 'object.txt'),'w',encoding='utf-8') as datei:
                    datei.write(
                        f"colour: {colour}, p_orig: {p_orig}, p_rot: {p_rot}, angle: {angle}\n"
                    )
                self.saved = True
                self.get_logger().warn(f'shutting down')
                self.destroy_node()
                rclpy.shutdown()

    def find_max_rotated_rectangle(self, occ_grid, white_penalty=1.0, angle_step=5):
        # 1) Graustufenbild laden
        img = self.occ_to_pgm_data_internal(occ_grid)
        weights = np.where(img==0, 1.0, -white_penalty).astype(np.float32)

        best = {'score': -np.inf}
        angles = np.arange(170, 190, angle_step)
        for angle in angles:
            rot_weights, M = self.rotate_matrix(weights, angle)
            # HIER: matrix-Variante aufrufen
            t, l, b, r, score = self.find_max_weighted_rectangle_fast(rot_weights)
            if score > best['score']:
                best.update(score=score, angle=angle, rect=(t,l,b,r), M=M.copy())

        # 3) Eckpunkte zurücktransformieren (bleibt unverändert)
        t, l, b, r = best['rect']
        corners = np.array([[l,t],[r,t],[r,b],[l,b]], dtype=np.float32)
        M_inv = cv2.invertAffineTransform(best['M'])
        orig_corners = cv2.transform(np.array([corners]), M_inv)[0]

        return orig_corners, best['angle'], best['score']
    
    def occ_to_pgm_data_internal(self, occ_grid_input):

        pgm_free = 254    # weiß
        pgm_occupied = 0  # schwarz

        pgm_data = np.full(occ_grid_input.shape, pgm_free, dtype=np.uint8)
        pgm_data[occ_grid_input >= 0] = pgm_occupied
        return pgm_data

    def find_max_weighted_rectangle_fast(self, weights: np.ndarray):

        # Stelle sicher, dass es float64 ist (Numba erwartet das)
        w64 = weights.astype(np.float64)
        return _kadane2d(w64)

    def rotate_matrix(self, mat, angle_deg):
        h, w = mat.shape
        M = cv2.getRotationMatrix2D((w/2, h/2), -angle_deg, 1.0)
        # borders auffüllen mit Wert 0 (kein Gewicht außerhalb)
        return cv2.warpAffine(mat, M, (w, h), flags=cv2.INTER_NEAREST, borderValue=0), M

    def publish_found(self):
        msg = String()
        msg.data = "found"
        self.status_publisher.publish(msg)

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def detection_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id
            if 992 <= tag_id <= 997:
                self.get_logger().info(f'Detected tag: {tag_id}')
                bbox = detection.corners
                if len(bbox) != 4:
                    self.get_logger().warn(f'Tag {tag_id} corners missing, skipping.')
                    return

                pts = [(int(pt.x), int(pt.y)) for pt in bbox]
                x_min = max(min([pt[0] for pt in pts]), 0)
                y_min = max(min([pt[1] for pt in pts]), 0)
                x_max = min(max([pt[0] for pt in pts]), self.image_shape[1])
                y_max = min(max([pt[1] for pt in pts]), self.image_shape[0])

                object_cx = detection.centre.x

                # 计算像素宽度，仍用bbox左右边界差
                x_coords = [pt[0] for pt in pts]
                pixel_width = max(x_coords) - min(x_coords)

                distance, angle_deg, (rel_x, rel_z) = self.estimate_distance_angle_position(
                    self.real_width, pixel_width, object_cx, self.image_shape
                )

                # 打印或处理结果
                if distance is not None:
                    self.get_logger().info(
                        f"distance: {distance:.4f} m, angle_deg: {angle_deg:.2f}°, "
                        f"相对坐标: rel_x={rel_x:.4f} m, rel_z={rel_z:.4f} m"
                    )
                else:
                    self.get_logger().warn("无法估算距离和角度")
                    return
                
                obj_x = None
                obj_y = None

                obj_x, obj_y = self.transform_point(rel_z+0.11,-rel_x)

                if obj_x is not None:
                    self.get_logger().info(
                        f'\n🧿 Detected tag: {tag_id}\n'
                        f'🗺️ 物体 real word 坐标: x={obj_x:.4f} m, y={obj_y:.4f} m'
                    )

                #     p = self.current_pose.position  
                #     o = self.current_pose.orientation

                #     _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])

                if self.latest_image is None:
                    self.get_logger().warn('No image received yet, skipping color detection.')
                    if obj_x is not None:
                        s = f"{tag_id},{obj_x:.4f},{obj_y:.4f}"
                        
                        if self.found_timer is None:
                            self.found_timer = self.create_timer(1.0, self.publish_found)

                        if self.searching:
                            self.logger.info(s)
                            self.logger.info(msg)
                            self.object_positions.append((obj_x,obj_y))

                else:

                    # 复制一份图像用于绘制
                    vis_image = self.latest_image.copy()

                    self.compute_white_balance_scales(vis_image,enable=self.dynamic_white_balcane)

                    vis_image=self.white_balance(vis_image,self.scales)
                    # 加在开头：先压亮度
                    vis_image = self.adjust_gamma(vis_image, gamma=0.6)

                    vis_image = self.increase_saturation(vis_image, scale=2.0)

                    tag_region = vis_image[y_min:y_max, x_min:x_max]

                    if tag_region.size == 0:
                        self.get_logger().warn(f'Tag {tag_id} region invalid, skipping.')
                        return

                    avg_bgr,avg_hsv =self.image_filter(tag_region)
                    
                    color_name = self.get_color_name_lab(avg_bgr)

                    self.get_logger().info(f'HSV: {avg_hsv}, BGR: {avg_bgr}, Detected tag color: {color_name}')
                    if obj_x is not None:
                        s = f"{tag_id},{color_name},{obj_x:.4f},{obj_y:.4f}"
                        if self.found_timer is None:
                            self.found_timer = self.create_timer(1.0, self.publish_found)

                        if self.searching:
                            self.logger.info(s)
                            self.logger.info(msg)
                            self.object_positions.append((obj_x,obj_y))
                            self.object_colours.append(color_name)

                return  # 只处理一个标签

    def image_filter(self, region):
        # 1. 高斯模糊，降低噪声
        region = cv2.GaussianBlur(region, (5, 5), 0)

        # 2. 转换为 HSV
        hsv_img = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

        # 3. 对 V 通道应用 CLAHE 亮度增强
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        v_channel = hsv_img[:, :, 2]

        # 根据V通道的均值判断是否增强，避免过度处理
        mean_v = np.mean(v_channel)
        if mean_v < 100:  
            # 亮度较低，增强更明显
            v_enhanced = clahe.apply(v_channel)
        else:
            # 亮度正常，做较轻微增强或保持原样
            v_enhanced = cv2.addWeighted(v_channel, 0.7, clahe.apply(v_channel), 0.3, 0)

        hsv_img[:, :, 2] = v_enhanced
        region_after_clahe = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR)

        # 4. 白平衡，可选（如果你定义了）

        # 5. 提取非黑色像素，避免低亮度噪声影响
        bgr_pixels = region_after_clahe.reshape(-1, 3)
        bgr_sum = np.sum(bgr_pixels, axis=1)
        mask_non_black = bgr_sum > 10  # 阈值调高，过滤更暗的点
        non_black_pixels = bgr_pixels[mask_non_black]

        # 6. 计算平均颜色
        if len(non_black_pixels) == 0:
            avg_bgr = np.array([127, 127, 127], dtype=np.uint8)  # 默认灰色
        else:
            avg_bgr = np.mean(non_black_pixels, axis=0).astype(np.uint8)

        # 7. 转换为 HSV 返回
        avg_hsv = cv2.cvtColor(np.uint8([[avg_bgr]]), cv2.COLOR_BGR2HSV)[0][0]

        return avg_bgr, avg_hsv

    def adjust_gamma(self,image, gamma=0.8):
        inv_gamma = 1.0 / gamma
        table = np.array([
            ((i / 255.0) ** inv_gamma) * 255 for i in np.arange(256)
        ]).astype("uint8")
        return cv2.LUT(image, table)
    
    def increase_saturation(self,image, scale=1.5):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # 饱和度增强并限制在 0~255
        s = np.clip(s.astype(np.float32) * scale, 0, 255).astype(np.uint8)

        hsv_enhanced = cv2.merge([h, s, v])
        enhanced_image = cv2.cvtColor(hsv_enhanced, cv2.COLOR_HSV2BGR)
        return enhanced_image

    def get_color_name_lab(self, bgr):
        import numpy as np
        import cv2

        STANDARD_COLORS_LAB = {
            "Red":    np.array([53, 80, 67]),
            "Orange": np.array([74, 23, 78]),
            "Yellow": np.array([97, -21, 94]),
            "Green":  np.array([87, -86, 83]),
            "Blue":   np.array([32, 79, -108]),
            "Pink":   np.array([60, 98, -60]),
            "Lila":   np.array([50, 60, -50]),   # Approx LAB for purple
            "Braun":  np.array([37, 20, 30])     # Approx LAB for brown
        }

        def lab_distance(c1, c2):
            return np.linalg.norm(c1 - c2)

        # 转HSV
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv

        # HSV初判
        if h < 10 or h >= 170:
            color_guess = "Red"
        elif 10 <= h < 20:
            color_guess = "Orange"
        elif 20 <= h < 35:
            color_guess = "Yellow"
        elif 35 <= h < 85:
            color_guess = "Green"
        elif 85 <= h < 130:
            color_guess = "Blue"
        elif 130 <= h < 150:
            color_guess = "Lila"
        elif 150 <= h < 170:
            color_guess = "Pink"
        else:
            color_guess = "Braun"

        # 转LAB
        lab = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2LAB)[0][0]
        L = lab[0] * 100 / 255.0
        a = lab[1] - 128
        b = lab[2] - 128
        lab_point = np.array([L, a, b])

        # 找最接近的LAB颜色
        min_dist = float('inf')
        lab_closest_color = None
        for name, std_lab in STANDARD_COLORS_LAB.items():
            dist = lab_distance(lab_point, std_lab)
            if dist < min_dist:
                min_dist = dist
                lab_closest_color = name

        # 用LAB修正HSV判断
        if lab_closest_color != color_guess and min_dist < 25:
            return lab_closest_color

        return color_guess

    def white_balance(self, image, scales):
        image = image.astype(np.float32)
        image[:, :, 0] *= scales[0]
        image[:, :, 1] *= scales[1]
        image[:, :, 2] *= scales[2]
        return np.clip(image, 0, 255).astype(np.uint8)
    
    def camera_info_callback(self, msg):
        self.focal_length_px = msg.k[0]  # fx

    def estimate_distance_angle_position(self, real_width_m, pixel_width, object_cx, image_shape):
        
        if self.focal_length_px is None:
            self.get_logger().warn("⚠️ 焦距尚未获取，无法估算距离")
            return None, None, (None,None)

        if pixel_width <= 0:
            self.get_logger().warn("⚠️ 像素宽度无效")
            return None, None, (None,None)

        # 估算距离
        distance = (real_width_m * self.focal_length_px) / pixel_width

        # 图像中心 x
        image_cx = image_shape[1] / 2
        dx = object_cx - image_cx

        # 角度（弧度和角度）
        angle_rad = np.arctan(dx / self.focal_length_px)
        angle_deg = np.degrees(angle_rad)

        # 相对坐标
        rel_x = distance * np.sin(angle_rad)
        rel_z = distance * np.cos(angle_rad)

        return distance, angle_deg, (rel_x, rel_z)

    def compute_white_balance_scales(self, image, enable=False,percent=5, target_white=(255, 255, 255), alpha=0.1):
        if enable :
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            flat_gray = gray.flatten()
            threshold = np.percentile(flat_gray, 100 - percent)

            mask = gray >= threshold
            bright_pixels = image[mask]

            if len(bright_pixels) == 0:
                self.get_logger().warn("⚠️ 未找到足够亮的区域，跳过白平衡计算")
                return

            avg_bgr = np.mean(bright_pixels, axis=0)
            scales = np.array(target_white) / avg_bgr

            if not hasattr(self, 'scales'):
                self.scales = scales.tolist()
            else:
                # 指数滑动平均
                self.scales = (1 - alpha) * np.array(self.scales) + alpha * scales
                self.scales = self.scales.tolist()
            return

    def transform_point(self, x, y):
        try:
            # Punkt relativ zu base_footprint
            point_in_base = PointStamped()
            point_in_base.header.stamp = rclpy.time.Time()
            point_in_base.header.frame_id = 'base_footprint'
            point_in_base.point.x = x
            point_in_base.point.y = y
            point_in_base.point.z = 0.0

            # Transformation in map-Frame
            transformed_point = self.tf_buffer.transform(point_in_base, 'map', timeout=rclpy.duration.Duration(seconds=10.0))

            self.get_logger().info(f"Transformed point in map frame: {transformed_point.point}")
            return transformed_point.point.x, transformed_point.point.y

        except Exception as e:
            self.get_logger().warn(f"Could not transform point: {e}")
            return None, None

def main(args=None):
    rclpy.init(args=args)
    node = AirTagListener()
    node.get_logger().info(f"node start")
    rclpy.spin(node)
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()