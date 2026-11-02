import os
from searchforce.functions.paths import CACHE_DIR
import rclpy
from rclpy.node import Node
# Importiere den Service aus dem maze_interface Paket
from maze_interface.srv import GetRosMaze

class MazeRequester(Node):

    def __init__(self):
        super().__init__('maze_requester')
        
        # 1. Erstelle einen Service-Client für den GetRosMaze Service
        # Der Service-Name wird hier explizit auf '/get_ros_maze' gesetzt.
        self.cli = self.create_client(GetRosMaze, '/get_ros_maze')
        
        # Warte, bis der Service verfügbar ist
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "/get_ros_maze" not available, waiting again...')
        
        self.req = GetRosMaze.Request()

    def send_request(self, maze_number, filename):
        self.req.maze_nr = maze_number
        self.get_logger().info(f'Requesting maze ID: {maze_number}')
        
        # Sende die Anfrage asynchron
        self.future = self.cli.call_async(self.req)
        
        # Warte, bis die Antwort empfangen wird
        rclpy.spin_until_future_complete(self, self.future)
        
        maze_response = self.future.result()

        if maze_response is not None:
            # Überprüfe, ob das Labyrinth ungültig ist (alle Werte sind 0)
            is_invalid = (maze_response.maze.n == 0 and
                          maze_response.maze.m == 0 and
                          maze_response.maze.start_idx == 0 and
                          maze_response.maze.end_idx == 0 and
                          maze_response.maze.start_orientation == 0 and
                          not maze_response.maze.l) # Prüft, ob die Liste 'l' leer ist

            if is_invalid:
                self.get_logger().warn(f'Received invalid maze for ID {maze_number} (all values are 0). Not publishing.')
            else:
                
                n = maze_response.maze.n
                m = maze_response.maze.m
                start_idx = maze_response.maze.start_idx
                end_idx = maze_response.maze.end_idx
                start_orientation = maze_response.maze.start_orientation
                flat_L = maze_response.maze.l

                with open(filename, 'w') as f:
                    f.write(f"{n}\n")
                    f.write(f"{m}\n")
                    f.write(f"{start_idx}\n")
                    f.write(f"{end_idx}\n")
                    f.write(f"{start_orientation}\n")
                    for value in flat_L:
                        f.write(f"{value}\n")
                # Veröffentliche die erhaltene Maze-Nachricht auf dem /maze Topic
                print(maze_response.maze.l)
                
        else:
            self.get_logger().error(f'Failed to receive maze for ID {maze_number}.')

def main(args=None):
    rclpy.init(args=args)

    maze_requester = MazeRequester()
    
    # Fordere Labyrinth ID X an (kann angepasst werden)
    maze_number = 29
    filename = os.path.join(CACHE_DIR, 'rosfile.maze')
    maze_requester.send_request(maze_number,filename)

    # The script exits after the request. To keep the node running for further requests,
    # use rclpy.spin(maze_requester) here.

if __name__ == '__main__':
    main()
