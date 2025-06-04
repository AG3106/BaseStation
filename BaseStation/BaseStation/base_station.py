import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import tkinter as tk
from base_station_UI import BaseStationUI # UI is passed in
from communication import RefBoxHandler # WiFiHandler is managed by Robot class

# from base_station_UI import load_config # If logic needed config directly

class BaseStationLogic:
    def __init__(self, ui):
        self.ui = ui
        self.robots = ui.robots # Get robots from UI (already initialized with config)
        self.opponents = ui.opponents # Get opponents from UI
        self.global_world = ui.global_world # Get global_world from UI
        
        # Connection status for the group of robots, not individual.
        # Individual robot.connected tracks specific robot.
        self.overall_robot_connection_active = False 

        # RefBox connection using RefBoxHandler
        refbox_config = ui.config.get('refbox', {"ip": "127.0.0.1", "port": 28097})
        self.refbox_handler = RefBoxHandler(
            refbox_config["ip"],
            refbox_config["port"],
            self.handle_refbox_message,
            self.handle_refbox_disconnect
        )
        # self.refbox_messages = [] # store all messages from RefBox here (UI logs them)0

    def connect_to_robots(self):
        self.overall_robot_connection_active = True # Flag that we've attempted to connect
        connection_results = {}
        self.ui.log_message("Attempting to connect to robots...\n")
        for robot in self.robots:
            if robot.wifi_handler: # Ensure handler exists
                if robot.connect():
                    # UI update is now handled in the periodic update_robot_ui_elements
                    # and also via robot.status_label if set directly
                    self.ui.log_message(f"Successfully connected to {robot.name}.\n")
                    connection_results[robot.name] = "Connected"
                else:
                    self.ui.log_message(f"Failed to connect to {robot.name}.\n")
                    connection_results[robot.name] = "Failed"
            else:
                 self.ui.log_message(f"No WiFi handler for {robot.name}. Cannot connect.\n")
                 connection_results[robot.name] = "No Handler"
        
        # Update UI elements after attempting all connections
        self.ui.update_robot_ui_elements()
        return connection_results


    def disconnect_from_robots(self):
        self.overall_robot_connection_active = False
        self.ui.log_message("Disconnecting from all robots...\n")
        for robot in self.robots:
            robot.disconnect()
            # UI update handled by periodic refresh
        self.ui.update_robot_ui_elements() # Immediate UI feedback
        self.ui.log_message("Disconnected from robots.\n")


    def connect_to_refbox(self): # ip and port are now from config
        if not self.refbox_handler.connected:
            self.refbox_handler.connect()
            # UI update will be triggered by callbacks
            # self.ui.update_refbox_status(connected=True) # This is handled by callback now
        else:
            self.ui.log_message("RefBox already trying to connect or is connected.\n")


    def handle_refbox_message(self, message):
        # This is called when a message is received OR on connection status changes from RefBoxHandler
        if "Connection Established" in message or "Connected to RefBox" in message :
             self.ui.update_refbox_status(connected=True)
        elif "connection refused" in message or "connection error" in message:
             self.ui.update_refbox_status(connected=False)
        
        self.ui.log_refbox_message(message) # Log all messages

    def handle_refbox_disconnect(self):
        # This callback is when the connection loop in RefBoxHandler ends
        self.ui.update_refbox_status(connected=False)
        self.ui.log_message("RefBox connection terminated or lost.\n")


    def stop_refbox(self):
        self.refbox_handler.stop()
        # self.ui.update_refbox_status(connected=False) # Done by handle_refbox_disconnect

    def update_world_state_and_ui(self):
        # 1. Update global world map from robots' current states
        #    (Robot states are updated by their individual handle_received_data via WiFiHandler)
        self.global_world.update_from_robots(self.robots)
        
        # 2. Redraw main field display
        self.ui.redraw_field()

        # 3. Update individual robot UI elements (status, battery) in the grid
        self.ui.update_robot_ui_elements()
        
        # 4. If a robot detail window is open, refresh its local map and parameter display
        #    This is now also handled by update_robot_ui_elements which calls refresh_robot_detail_view

        # Keep scheduling next update
        self.ui.root.after(200, self.update_world_state_and_ui) # Update rate (e.g., 200ms for 5 FPS)

    # parse_message seems unused or was a placeholder
    # def parse_message(self, message):
    #     print(message)
class ROSBaseStation(Node):
    def __init__(self, robots, opponents, ball_pos):
        super().__init__('ROSBaseStation')
        self.robots = []
        self.opponents = []
        self.publ = {}
        self.tim = {}
        # Create 5 publishers
        for robot in robots:
            robot.position = [float(x) for x in robot.position]  # Convert to float
            self.robots.append(robot)
        for opponent in opponents:
            opponent.position = [float(x) for x in opponent.position]  # Convert to float
            self.robots.append(opponent)
        for i in range(0, 5):  
            self.publ[f'o{i+1}'] = self.create_publisher(
                Float32MultiArray,
                f'o{i+1}_data',
                10
            )
            self.tim[f'o{i+1}'] = self.create_timer(
                0.01,
                lambda robot_index=i-1, pub_name=f'o{i+1}': self.publish_robot(robot_index, pub_name)
            )
        for i in range(0, 5):  
            self.publ[f'e{i+1}'] = self.create_publisher(
                Float32MultiArray,
                f'e{i+1}_data',
                10
            )
            self.tim[f'e{i+1}'] = self.create_timer(
                0.01,
                lambda robot_index=i-1, pub_name=f'e{i+1}': self.publish_robot(robot_index, pub_name)
            )
    def publish_robot(self, robot_index, publisher_name):
        if robot_index < len(self.robots):
            robot_pos = self.robots[robot_index].position.copy()
            print(robot_pos)
            robot_pos.append(0.0)
            msg = Float32MultiArray()
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = f"{publisher_name}_data"
            msg.layout.dim[0].size = len(robot_pos)
            msg.layout.dim[0].stride = len(robot_pos)
            msg.layout.data_offset = 0
            msg.data = robot_pos
            self.publ[publisher_name].publish(msg)
            self.get_logger().info(f'Published {publisher_name}')

    def publish_opponent(self, opponent_index, publisher_name):
        if opponent_index< len(self.opponents):
            opponent_pos = self.opponents[opponent_index].position.copy()
            opponent_pos.append(0.0)
            msg = Float32MultiArray()
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = f"{publisher_name}_data"
            msg.layout.dim[0].size = len(opponent_pos)
            msg.layout.dim[0].stride = len(opponent_pos)
            msg.layout.data_offset = 0
            msg.data = opponent_pos
            self.publ[publisher_name].publish(msg)
            self.get_logger().info(f'Published {publisher_name}')
        


# Main execution part remains similar but ensure logic is passed to UI
def main():
    root = tk.Tk()
    app = BaseStationUI(root)
    if not app.config: # If config loading failed in UI, app might be destroyed.
        print("Exiting due to configuration error.")
        return

    logic = BaseStationLogic(app)
    app.logic = logic # Make logic accessible from UI (e.g., for button commands)

    # Initial connection attempts
    logic.connect_to_robots() 
    # logic.connect_to_refbox() # Optionally auto-connect to refbox on startup

    # Start the periodic update loop
    logic.update_world_state_and_ui()
    rclpy.init(args=None)
    ros_bs = ROSBaseStation(logic.robots, logic.opponents, logic.global_world.ball_position)
    try:
        rclpy.spin(ros_bs)
    except KeyboardInterrupt:
        pass
    finally:
        ros_bs.destroy_node()
        rclpy.shutdown()
    root.mainloop()

    # Cleanup on exit
    print("Closing application. Disconnecting services...")
    logic.disconnect_from_robots()
    logic.stop_refbox()
    print("Application closed.")


if __name__ == "__main__":
    main()
