import os
from searchforce.functions.paths import CACHE_DIR
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import networkx as nx

# Helper functions
def occupancy_grid_to_pgm(occupancy_grid):
    height, width = occupancy_grid.shape
    pgm_image = np.zeros((height, width), dtype=np.uint8)
    pgm_image[occupancy_grid == -1] = 205  # ROS-Standard für unbekannt (grau)
    pgm_image[occupancy_grid == 0] = 254    # Frei (fast weiß)
    pgm_image[occupancy_grid > 0] = 0       # Belegt (schwarz)
    return pgm_image

def is_mostly_black_region(image, region, threshold=0.99, black_thresh=80):
    x, y, w, h = region
    roi = image[y:y+h, x:x+w]
    if roi.size == 0:
        return False
    if len(roi.shape) == 3:
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        roi_gray = roi
    black_pixels = roi_gray > black_thresh
    black_ratio = np.sum(black_pixels) / (w * h)
    return black_ratio <= threshold

def zeichnen(breite, hoehe, img, offset,big2):
    offset = offset - 26
    # Canvas size and cell pixel size are tuned to the maze used in the competition
    circles_canvas = np.full(((25 * (2 * hoehe + 1)) + 2 * offset, (25 * (2 * breite + 1)) + 2 * offset, 3), 255, dtype=np.uint8)
    wand = np.zeros((hoehe+1, breite))
    wand2 = np.zeros((hoehe, breite+1))
    G_grid = nx.Graph()
    for r in range(hoehe):
        for c in range(breite):
            G_grid.add_node((r, c))

    cell_width = 26.0  # float for precise calculation
    cell_height = 26.0
    scale_factor = 2
    # height_canvas_grid = circles_canvas.shape[0] - offset * 2
    # width_canvas_grid = circles_canvas.shape[1] - offset * 2

    for i in range(1, breite):
        x_grid = offset + int(i * 2 * cell_width)
        cv2.line(circles_canvas, (x_grid, offset), (x_grid, int(hoehe * scale_factor * cell_height) + offset), (120, 157, 0), 1)

    for j in range(1, hoehe):
        y_grid = offset + int(j * 2 * cell_height)
        cv2.line(circles_canvas, (offset, y_grid), (int(breite * scale_factor * cell_width) + offset, y_grid), (120, 157, 0), 1)
    
    for i in range(breite):
        for j in range(hoehe + 1):
            if(j==0):
                    region = (2 * 26 * i + 32, 2 * 25 * j, 5, 30)
            if(j>0):
                    region = (2 * 26 * i + 32, 2 * 25 * j-3, 5, 30)
            if (hoehe == 6):
                if (j > 0):
                    region = (2 * 26 * i + 25, 2 * 25 * j - 10, 5, 30)
                    
            if(big2 and breite>7):
                
                region = (2 * 25 * i + 28 , 2 * 25 * j+2, 5, 27)
            
            draw_x = offset + 2 * 26 * i
            draw_y = offset + 2 * 26 * j

            if is_mostly_black_region(img, region):
                wand[j][i] = 1
                if j == 0:
                    cv2.rectangle(circles_canvas, (draw_x, draw_y), (draw_x + 52, draw_y + 6), (5, 0, 4), -1)
                    cv2.rectangle(img, (region[0], region[1]), (region[0] + region[2], region[1] + region[3]), (120, 157, 0),-1)
                else:
                    cv2.rectangle(circles_canvas, (draw_x, draw_y + 3), (draw_x + 52, draw_y - 3), (5, 0, 4), -1)
                    cv2.rectangle(img, (region[0], region[1]), (region[0] + region[2], region[1] + region[3]), (120, 157, 0),-1)
            else:
                cv2.rectangle(img, (region[0], region[1]), (region[0] + region[2], region[1] + region[3]), (0, 157, 0),-1)
    for j in range(hoehe):
        for i in range(breite + 1):
            region = (2 * 25 * i + 0, 2 * 26 * j + 25, 30, 5)
            
            if(j>7 and big2):
               
               region = (2 * 25 * i + 0, 2 * 25 * j + 25, 30, 5)
            if(big2 and breite>7):
                
                region = (2 * 24 * i, 2 * 25 * j + 35, 37, 5) 
            draw_x = offset + 2 * 26 * i
            draw_y = offset + 2 * 26 * j

            if is_mostly_black_region(img, region):
                wand2[j][i] = 1
                if i == 0:
                    cv2.rectangle(circles_canvas, (draw_x, draw_y), (draw_x + 6, draw_y + 52), (5, 0, 4), -1)
                    cv2.rectangle(img, (region[0], region[1]), (region[0] + region[2], region[1] + region[3]), (120, 157, 100),-1)
                  
                else:
                    cv2.rectangle(img, (region[0], region[1]), (region[0] + region[2], region[1] + region[3]), (120, 157, 0),-1)
                    cv2.rectangle(circles_canvas, (draw_x + 3, draw_y), (draw_x - 3, draw_y + 52), (5, 2, 4), -1)
                    
    for i in range(breite):  # Iterate through columns using 'breite' (i represents column index)
       # Iterate through rows, skipping first and last (indices 1 to hoehe-2 for an 8x8 array, so 1 to 6)
       for j in range(1, hoehe):
           current_value = wand[j, i]
   
           # Check the element above the current one
           # The condition j > 0 is always true for j in range(1, hoehe - 1)
           previous_value = wand[j - 1, i]
   
           # Rule 1: If after a 1 comes a 0, set an edge
           if previous_value == 1 and current_value == 0:
               G_grid.add_edge((j - 1, i), (j, i))
           
           # Rule 2: If after a 0 comes a 0, set an edge
           elif previous_value == 0 and current_value == 0:
               G_grid.add_edge((j - 1, i), (j, i))

    for j in range(hoehe):  # Iteriere durch alle Reihen (inklusive der ersten)
       # Iteriere durch Spalten von links nach rechts, von der ersten bis zur vorletzten Spalte.
       # Wir müssen bei 'breite - 1' aufhören, da wir 'i + 1' überprüfen.
       for i in range(breite - 1): # <--- Iteration von links nach rechts
           current_value = wand2[j, i + 1] # Das Element, das wir jetzt betrachten (rechts vom 'previous')
           previous_value_left = wand2[j, i]   # Das Element direkt links davon ('previous' in dieser Richtung)
   
           # Regel 1: Wenn nach einer 1 (links) eine 0 (rechts) kommt, setze eine Kante
           if previous_value_left == 1 and current_value == 0:
               G_grid.add_edge((j, i), (j, i + 1))
           
           # Regel 2: Wenn nach einer 0 (links) eine 0 (rechts) kommt, setze eine Kante
           elif previous_value_left == 0 and current_value == 0:
               G_grid.add_edge((j, i), (j, i + 1))

    connected_components = list(nx.connected_components(G_grid))
   
   # Handle the case where G_grid might be empty or only has one component
    if not connected_components:
       largest_component = set() # Empty set if no components
       removed_areas = [] # Initialize as empty list if no components
    else:
       largest_component = max(connected_components, key=len)

       # This will store each non-largest component as its own list
       removed_areas = []
       for component in connected_components:
           if component != largest_component:
               removed_areas.append(list(component)) # Append each component as a new list

    nodes_to_remove = [node for sublist in removed_areas for node in sublist] # Simplified

    G_grid.remove_nodes_from(nodes_to_remove)   
    circles_canvas_white=circles_canvas.copy()     
    if nodes_to_remove:

        for i in range(len(nodes_to_remove)):
           cv2.rectangle(circles_canvas,(nodes_to_remove[i][1]*52,nodes_to_remove[i][0]*52) ,(nodes_to_remove[i][1]*52+52,nodes_to_remove[i][0]*52+52),(0,0,0),-1)   
    return circles_canvas, wand,wand2,circles_canvas_white

class MapProcessorNode(Node):
    def __init__(self):
        super().__init__('map_processor_node')
        self._map = None

        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.create_subscription(
            String,
            'searchforce_status',
            self.status_callback,
            10
        )
        self._map_saved = False
        self._saving = False

        self.get_logger().info('Map Processor Node ist bereit.')

    def map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def status_callback(self, msg: String):
        if msg.data.strip().lower() == 'finished' and not self._map_saved and not self._saving:
            self._saving = True
            self.get_logger().info('„finished“ empfangen – speichere Karte…')
            # One‐Shot‐Timer
            def _retry(timer):
                timer.cancel()
                timer = None
                self.process_map_service_callback()
                self._map_saved = True
                self._saving = False

            timer = self.create_timer(5.0, lambda: _retry(timer))

    def process_map_service_callback(self):
        self.get_logger().info('Starte Kartenverarbeitung.')

        occ_grid = self._map

        if occ_grid is None:
            self.get_logger().error('Keine Karte empfangen. Abbruch.')
            return

        # Load robot positions from file
        robot_positions_file = os.path.join(CACHE_DIR, 'robot_positions.npy')
        if not os.path.exists(robot_positions_file):
            self.get_logger().error(f'Roboterpositionsdatei nicht gefunden: "{robot_positions_file}". Abbruch.')
            return 

        try:
            position = np.load(robot_positions_file)
            if position.ndim != 2 or position.shape[1] != 2:
                self.get_logger().error(f'Ungültiges Format in "{robot_positions_file}". Erwarte Nx2-Array.')
                return 
            self.get_logger().info(f'Erfolgreich {position.shape[0]} Roboterpositionen aus "{robot_positions_file}" geladen.')
        except Exception as e:
            self.get_logger().error(f'Fehler beim Laden der Roboterpositionen: {e}')
            return

        # 1. Map-Daten vorbereiten
        map_info = occ_grid.info
        map_data = np.array(occ_grid.data, dtype=np.int8).reshape((map_info.height, map_info.width))
        map_data = np.flipud(map_data)  # Konvertierung von Map-Koordinaten zu Bild-Koordinaten
        img = occupancy_grid_to_pgm(map_data)
        img_big=img

        # 2. Image processing
        _, img_binarized = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        angle = 1
        center = (img.shape[1] // 2, img.shape[0] // 2)
        scale = 1.0
        M_rot = cv2.getRotationMatrix2D(center, angle, scale)  # Variable umbenannt, um Konflikt zu vermeiden
        img = cv2.warpAffine(img, M_rot, (img.shape[1], img.shape[0]))
        
        big2=False
    
        heightx, widthx = img.shape

        if(heightx>300 and widthx >210 and heightx/widthx>1.4):
            big2=True

        filtered = cv2.medianBlur(img, 3)
        filtered = cv2.GaussianBlur(filtered, (5, 5), 1)
        edges = cv2.Canny(filtered, 100, 500)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().error("Keine Kontur erkannt. Abbruch.")
            return

        main_contour = max(contours, key=lambda cnt: cv2.boundingRect(cnt)[2] * cv2.boundingRect(cnt)[3])
        rect = cv2.minAreaRect(main_contour)
        box_points = cv2.boxPoints(rect)
        box_points = np.int32(box_points)

        sorted_by_x = box_points[np.argsort(box_points[:, 0])]
        left_points = sorted_by_x[:2]; right_points = sorted_by_x[2:]
        top_left = left_points[np.argmin(left_points[:, 1])]; bottom_left = left_points[np.argmax(left_points[:, 1])]
        top_right = right_points[np.argmin(right_points[:, 1])]; bottom_right = right_points[np.argmax(right_points[:, 1])]

        src_pts = np.array([bottom_left, top_left, top_right, bottom_right], dtype="float32")

        dim1 = rect[1][0]
        dim2 = rect[1][1]
        angle = rect[2]

        if abs(angle) > 45:
            width_raw = dim2
            height_raw = dim1
        else:
            width_raw = dim1
            height_raw = dim2

        width = int(width_raw)
        height = int(height_raw)
        if width == 0 or height == 0:
            self.get_logger().error("Breite oder Höhe der Kontur ist null. Abbruch.")
            return 

        # Cell count from the warped maze size
        REAL_BLOCK_CM = 26
        # Note: depends on the perspective warp and can be inaccurate for large distortions.
        num_cols = round(width / REAL_BLOCK_CM); num_rows = round(height / REAL_BLOCK_CM)
        if num_cols == 0: num_cols = 1
        if num_rows == 0: num_rows = 1
        self.get_logger().info(f"Grid: {num_cols} x {num_rows} Zellen")

        dst_pts = np.array([[0, height - 1], [0, 0], [width - 1, 0], [width - 1, height - 1]], dtype="float32")
        M_persp = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped_original_img_binarized = cv2.warpPerspective(img_binarized, M_persp, (width, height))

        scale_factor = 2
        new_width_scaled = int(width * scale_factor); new_height_scaled = int(height * scale_factor)
        warped_original_scaled = cv2.resize(warped_original_img_binarized, (new_width_scaled, new_height_scaled), interpolation=cv2.INTER_LINEAR)
        
        height2, width2 = warped_original_scaled.shape
        if(big2==True and (heightx/widthx>height2/width2 or ((height2*width2)/(heightx*widthx))<1.4)):

                # Bildverarbeitung
                img_processed = img_big.copy() # Eine Kopie erstellen, um das Original nicht zu ändern
                img_processed[(img_processed >= 220) & (img_processed <= 255)] = 0
                
                # 2. Filterung und Binarisierung
                blurred_img = cv2.medianBlur(img_processed, 1)
                _, binary_img = cv2.threshold(blurred_img, 100, 255, cv2.THRESH_BINARY_INV) # Korrektur: THRESH_BINARY_INV
                
                # 3. Morphologische Operationen
                kernel = np.ones((3,3), np.uint8)
                dilated_img = cv2.dilate(binary_img, kernel, iterations=0)
                cleaned_img = cv2.erode(dilated_img, kernel, iterations=7)

                # --- Integration der Harris Corner Detection ---
                
                # Bild für Harris Corner Detection vorbereiten
                harris_input = np.float32(cleaned_img)
                
                # Harris Corner Detection anwenden
                dst = cv2.cornerHarris(harris_input, 2, 3, 0.04)
                
                # Die Ergebnisse mit Dilatation hervorheben
                dst = cv2.dilate(dst, None)

                # Finde die Koordinaten der erkannten Harris-Ecken
                # corner_coords enthält Paare von (y, x) Koordinaten
                corner_coords = np.argwhere(dst > 0.01 * dst.max())
                
                # --- Initialisiere das zugeschnittene Bild
                cropped_img_with_border = np.zeros((1,1), dtype=np.uint8) # Dummy-Initialisierung
                scaled_cropped_img = np.zeros((1,1), dtype=np.uint8) # Dummy-Initialisierung für skaliertes Bild
                
                # Überprüfe, ob überhaupt Harris-Ecken gefunden wurden
                if len(corner_coords) > 0:
                    # Finde die aufrechte Bounding Box um alle Harris-Ecken
                    # corner_coords ist (y, x), also müssen wir x_min, y_min, x_max, y_max bestimmen
                    x_coords = corner_coords[:, 1]
                    y_coords = corner_coords[:, 0]

                    x_min_raw = np.min(x_coords)
                    y_min_raw = np.min(y_coords)
                    x_max_raw = np.max(x_coords)
                    y_max_raw = np.max(y_coords)

                    # Berechne Breite und Höhe des aufrechten Rechtecks
                    # +1, da die Differenz zwischen max und min die Anzahl der Intervalle ist, nicht der Pixel
                    width_final = x_max_raw - x_min_raw + 1
                    height_final = y_max_raw - y_min_raw + 1

                    # Sicherstellen, dass die Dimensionen positiv sind und mindestens 1 Pixel groß
                    width_final = max(1, width_final)
                    height_final = max(1, height_final)

                    # --- Zuschneiden des Bildes ohne Rotation ---
                    # Sicherstellen, dass die Koordinaten innerhalb der Bildgrenzen liegen, bevor zugeschnitten wird
                    x1 = np.clip(x_min_raw, 0, img_big.shape[1] - 1)
                    y1 = np.clip(y_min_raw, 0, img_big.shape[0] - 1)
                    x2 = np.clip(x_max_raw, 0, img_big.shape[1] - 1)
                    y2 = np.clip(y_max_raw, 0, img_big.shape[0] - 1)

                    # Überprüfen, ob die Zuschnittsregion gültig ist (Breite und Höhe > 0)
                    if x2 < x1 or y2 < y1: # Wenn x_max <= x_min oder y_max <= y_min
                    
                        cropped_img = np.zeros((height_final, width_final), dtype=np.uint8) # Erstelle ein leeres Bild
                    else:
                        cropped_img = img_big[y1:y2+1, x1:x2+1] # +1 für den Endindex im Slicing

                    # Überprüfen und Anpassen der Größe, falls die Abmessungen leicht abweichen
                    if cropped_img.shape[0] > 0 and cropped_img.shape[1] > 0:
                        if cropped_img.shape[1] != width_final or cropped_img.shape[0] != height_final:
                            cropped_img = cv2.resize(cropped_img, (width_final, height_final), interpolation=cv2.INTER_LINEAR)
                    else:
                        cropped_img = np.zeros((height_final, width_final), dtype=np.uint8) # Fallback

                    # Schwarzen Rand hinzufügen
                    border_size = 5
                    if cropped_img.shape[0] > 0 and cropped_img.shape[1] > 0:
                        cropped_img_with_border = cv2.copyMakeBorder(cropped_img, border_size, border_size, border_size, border_size, cv2.BORDER_CONSTANT, value=0)
                    else:
                        cropped_img_with_border = np.zeros((height_final + 2*border_size, width_final + 2*border_size), dtype=np.uint8)

                    # --- Zugeschnittenes Bild mit Rand um Faktor 2 skalieren ---
                    scale_factor = 2
                    scaled_width = int(cropped_img_with_border.shape[1] * scale_factor)
                    scaled_height = int(cropped_img_with_border.shape[0] * scale_factor)
                    
                    if scaled_width > 0 and scaled_height > 0:
                        scaled_cropped_img = cv2.resize(cropped_img_with_border, (scaled_width, scaled_height), interpolation=cv2.INTER_LINEAR)
                    else:
                        scaled_cropped_img = np.zeros((1,1), dtype=np.uint8) # Fallback für leeres Bild

                    # Das finale skalierte Bild in Schwarz-Weiß umwandeln
                    final_scaled_img = scaled_cropped_img.copy()
                    final_scaled_img[final_scaled_img > 150] = 255 

                    offset = 26+0 # zweiter wert ist zeichen offset
                    REAL_BLOCK_CM = 25.5
                    
                    # Anpassung: num_cols und num_rows basieren jetzt direkt auf den finalen Abmessungen des zugeschnittenen Bildes
                    # und nicht auf der ursprünglichen 'crop_width'/'crop_height' aus der Rotationslogik.
                    # Es ist entscheidend, dass die `width_final` und `height_final` hier verwendet werden,
                    # da diese die tatsächlichen Abmessungen des zugeschnittenen Harris-Bereichs darstellen.
                    num_cols = round(width_final / REAL_BLOCK_CM)
                    num_rows = round(height_final / REAL_BLOCK_CM)
                    
                    map_circles, wand,wand2,circles_canvas_white=zeichnen(num_cols,num_rows,final_scaled_img, offset,big2) #wichtig
        else:

                    big2=False
                    offset = 26+0 # zweiter wert ist zeichen offset
                    map_circles, wand,wand2,circles_canvas_white=zeichnen(num_cols,num_rows,warped_original_scaled, offset,big2) #wichtig

        # 4. Draw the driven path
        if position.size > 0:
            map_circles_drawable_width = map_circles.shape[1] - 2 * offset
            map_circles_drawable_height = map_circles.shape[0] - 2 * offset
            previous_point_on_map = None

            for i in range(len(position)):
                current_x = position[i, 0]
                current_y = position[i, 1]
                # Scaling is tuned to the maze used in the competition
                start_y = -2 * min(position[:, 0]) / 0.254
                current_point_on_map_x = 200 * current_x + offset * (1 + start_y)
                start = 2 * max(position[:, 1]) / 0.254
                current_point_on_map_y = -200 * current_y + offset * (1 + start)

                # Startpunkt wird bei jeder Iteration neu gezeichnet (wie im Original)
                # Beachten Sie, dass der Startpunkt hier immer der erste Punkt im Array ist, was etwas redundant ist,
                # da er in der Schleife bei jedem Schritt neu gezeichnet wird.
                start_point_on_map = (int((position[0, 0] / (width * map_info.resolution)) * map_circles_drawable_width + offset * (1 + start_y)),
                                      int((position[0, 1] / (height * map_info.resolution)) * map_circles_drawable_height + offset * (1 + start)))
                cv2.circle(map_circles, start_point_on_map, 8, (0, 255, 0), -1) # Green circle for start
                cv2.circle(circles_canvas_white, start_point_on_map, 8, (0, 255, 0), -1) # Green circle for start

                current_point_on_map = (int(current_point_on_map_x), int(current_point_on_map_y))
                cv2.circle(map_circles, current_point_on_map, 3, (120, 157, 0), -1) # Blueish circle for current point
                cv2.circle(circles_canvas_white, current_point_on_map, 3, (120, 157, 0), -1) # Blueish circle for current point

                if previous_point_on_map is not None:
                    cv2.line(map_circles, previous_point_on_map, current_point_on_map, (120, 157, 0), 2) # Line connecting points
                    cv2.line(circles_canvas_white, previous_point_on_map, current_point_on_map, (120, 157, 0), 2) # Line connecting points
                previous_point_on_map = current_point_on_map

        if(position[:, 0][-1]>0 and position[:, 1][-1]>0):
            map_circles=cv2.rotate(map_circles, cv2.ROTATE_90_CLOCKWISE)
            map_circles=cv2.rotate(circles_canvas_white, cv2.ROTATE_90_CLOCKWISE)
        # Ergebnis speichern
        save_path = os.path.expanduser(os.path.join(CACHE_DIR, 'finished_map_original_logic.png'))
        save_path2 = os.path.expanduser(os.path.join(CACHE_DIR, 'finished_map_original_logic2.png'))
        cv2.imwrite(save_path, map_circles)
        cv2.imwrite(save_path2, circles_canvas_white)
        self.get_logger().info(f'Karte erfolgreich gespeichert: "{save_path}"')

def main(args=None):
    rclpy.init(args=args)
    node = MapProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MapProcessorNode wird heruntergefahren.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()