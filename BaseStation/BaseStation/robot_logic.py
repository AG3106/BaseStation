import json
from communication import WiFiHandler # Assuming communication.py is in the same directory or package

class Robot:
    def __init__(self, robot_id, name="Robot", color="blue", ip_address=None, send_to_port=None, base_station_listen_port=None, initial_pos=(0,0), initial_orient=0):
        self.robot_id = robot_id
        self.name = f"{name} {robot_id}"
        self.color = color
        
        # Data from the robot's sensors/localization (global coordinates)
        self.position = list(initial_pos)  # [x, y]
        self.orientation = initial_orient  # degrees
        self.local_ball_position = None  # [x, y] as seen by robot, in global frame
        self.local_obstacles = []        # List of [x, y] obstacles in global frame

        self.parameters = {
            "max_speed": 2.0, "rotation_speed": 1.0, "kick_power": 0.8,
            "acceleration": 1.5, "deceleration": 1.5, "battery_level": 100,
            "vision_range": 5.0, "ball_detection_threshold": 0.7,
            "obstacle_detection_threshold": 0.6, "communication_range": 20.0
        }
        self.connected = False
        self.status_label = None # For UI updates
        self.battery_label = None # For UI updates

        if ip_address and send_to_port and base_station_listen_port:
            self.wifi_handler = WiFiHandler(ip_address, send_to_port, base_station_listen_port, self.handle_received_data)
        else:
            self.wifi_handler = None
            print(f"WiFi handler not initialized for {self.name} due to missing IP/Port configuration.")

    def handle_received_data(self, data_str):
        """Callback to process received data from this robot."""
        print(f"Received data for {self.name}: {data_str}")
        try:
            data_dict = json.loads(data_str)
            
            # Update robot's own pose (position and orientation)
            if 'position' in data_dict and len(data_dict['position']) == 3:
                self.position = [data_dict['position'][0], data_dict['position'][1]]
                self.orientation = data_dict['position'][2] # theta
            
            # Update ball position as seen by this robot (assumed global)
            if 'ball_position' in data_dict and data_dict['ball_position'] is not None:
                self.local_ball_position = list(data_dict['ball_position'])
            else:
                self.local_ball_position = None # Ball not seen or not reported

            # Update obstacles as seen by this robot (assumed global)
            if 'obstacles' in data_dict:
                self.local_obstacles = [list(obs) for obs in data_dict['obstacles']] # Ensure it's a list of lists
            else:
                self.local_obstacles = []

            print(f"{self.name} updated: Pos={self.position}, Orient={self.orientation}, Ball={self.local_ball_position}, Obstacles={len(self.local_obstacles)}")

        except json.JSONDecodeError:
            print(f"Error decoding JSON from {self.name}: {data_str}")
        except Exception as e:
            print(f"Error processing data for {self.name}: {e}")


    def connect(self):
        """Connect to the robot using WiFiHandler."""
        if self.wifi_handler and not self.wifi_handler.connected: # Check wifi_handler's connected status
            if self.wifi_handler.connect(): # This now also starts listening
                self.connected = True # Robot considered connected if WiFi link is up
                return True
            else:
                self.connected = False
                return False
        elif self.wifi_handler and self.wifi_handler.connected:
            self.connected = True # Already connected
            return True
        return False

    def disconnect(self):
        """Disconnect from the robot."""
        if self.wifi_handler: # and self.connected: # self.connected might be true even if wifi_handler is None
            self.wifi_handler.disconnect()
        self.connected = False # Always set to false on disconnect intent

    def send_to_robot(self, msg):
        """Send a message to the robot."""
        if self.wifi_handler and self.connected:
            # print(f"Attempting to send to {self.name}: {msg}") # Debug
            self.wifi_handler.send(msg)
        else:
            print(f"Cannot send to {self.name}: Not connected or no WiFi handler.")

    def set_parameters(self, parameters):
        self.parameters.update(parameters)
        print(f"Updated parameters for {self.name}")

class GlobalWorldMap:
    def __init__(self, field_dims=(12,9)):
        self.field_dimensions = tuple(field_dims)
        self.ball_position = [self.field_dimensions[0] / 2, self.field_dimensions[1] / 2] # Default to center
        self.obstacles = [] # Global list of unique obstacles

    def update_from_robots(self, robots):
        # Aggregate ball position (e.g., average of robots that see it)
        # Aggregate obstacles (e.g., union of all seen obstacles)
        
        # Ball position aggregation
        visible_balls = []
        for robot in robots:
            if robot.connected and robot.local_ball_position: # Use local_ball_position
                visible_balls.append(robot.local_ball_position)
        
        if visible_balls:
            avg_ball_x = sum(pos[0] for pos in visible_balls) / len(visible_balls)
            avg_ball_y = sum(pos[1] for pos in visible_balls) / len(visible_balls)
            self.ball_position = [avg_ball_x, avg_ball_y]
        # else: keep last known or default if no robot sees the ball

        # Obstacle aggregation (simple union, could be improved with filtering/merging)
        all_obstacles = []
        for robot in robots:
            if robot.connected and robot.local_obstacles:
                all_obstacles.extend(robot.local_obstacles)
        
        # To avoid duplicates if obstacles are represented precisely
        # This is a simple way; more robust methods might be needed for real-world noise
        unique_obstacles_tuples = {tuple(obs) for obs in all_obstacles}
        self.obstacles = [list(obs) for obs in unique_obstacles_tuples]