import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os
import json
import math
try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("Pillow library not found. Images will not be loaded.")

from robot_logic import Robot, GlobalWorldMap 
CONFIG_FILE = "config.json"

# load_config function (assuming it's unchanged and working)
def load_config():
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        config.setdefault('refbox', {"ip": "127.0.0.1", "port": 28097})
        config.setdefault('robots', [])
        config.setdefault('opponents', [])
        config.setdefault('field_dimensions', [12, 9])
        config.setdefault('local_map_view_range_m', 6) 
        return config
    except FileNotFoundError:
        messagebox.showerror("Error", f"Configuration file '{CONFIG_FILE}' not found.")
        return None
    except json.JSONDecodeError:
        messagebox.showerror("Error", f"Error decoding JSON from '{CONFIG_FILE}'.")
        return None

class BaseStationUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Team Era Base Station")
        self.root.geometry("1200x800")

        self.config = load_config()
        if not self.config:
            self.root.destroy() 
            return

        self.global_world = GlobalWorldMap(field_dims=self.config['field_dimensions'])
        # local_map_view_range_m is no longer used for zoom, but keep if other uses exist
        self.local_map_view_range_m = self.config.get('local_map_view_range_m', 6) 
        self.current_detailed_robot = None
        self.logging_text = None
        self.robot_param_labels = {} # Initialize here

        # HOME ROBOTS
        self.robots = []
        for r_idx, r_conf in enumerate(self.config.get('robots', [])):
            self.robots.append(Robot(
                robot_id=r_conf['id'], name=r_conf.get('name', "Player"), color=r_conf.get('color', "blue"),
                ip_address=r_conf.get('ip'), send_to_port=r_conf.get('send_to_port'),
                base_station_listen_port=r_conf.get('base_listen_port'),
                initial_pos=r_conf.get('initial_pos', [1 + r_idx, 1]),
                initial_orient=r_conf.get('initial_orient', 0)
            ))
        if not self.robots:
             self.robots = [Robot(i + 1, "Player", "blue", initial_pos=(1+i,1), initial_orient=0) for i in range(5)]
        # OPPONENT ROBOTS
        self.opponents = []
        for idx, o_conf in enumerate(self.config.get('opponents', [])):
            opponent = Robot(robot_id=o_conf['id'], name=o_conf.get('name', "Opponent"), color=o_conf.get('color', "red"),
                             initial_pos=o_conf.get('initial_pos', [10 + idx, 1]),
                             initial_orient=o_conf.get('initial_orient', 180))
            opponent.wifi_handler = None 
            self.opponents.append(opponent)
        
        if not self.opponents: 
            self.opponents = [Robot(i + 1, "Opponent", color="red", initial_pos=(10+i,7), initial_orient=180) for i in range(5)]

        self.logic = None 
        self.is_playing = False
        self.robot_images = {} 

        self.setup_ui()

    def setup_ui(self):
        # ... (banner_frame and logo setup - assumed unchanged) ...
        banner_frame = tk.Frame(self.root, bg="#a8328d", height=80)
        banner_frame.pack(fill=tk.X)
        banner_frame.pack_propagate(0)

        def load_logo(path, size, parent, text_if_fail):
            if PIL_AVAILABLE:
                try:
                    img = ImageTk.PhotoImage(Image.open(path).resize(size))
                    label = tk.Label(parent, image=img, bg=parent.cget("bg"))
                    label.image = img 
                    return label
                except Exception as e:
                    print(f"Failed to load image {path}: {e}")
            return tk.Label(parent, text=text_if_fail, fg="white", bg=parent.cget("bg"), font=("Arial", 12))

        team_logo_label = load_logo("robocup_logo.png", (180, 60), banner_frame, "Team Logo")
        team_logo_label.pack(side=tk.LEFT, padx=10)
        
        center_logo_frame = tk.Frame(banner_frame, bg="#a8328d")
        center_logo_frame.pack(side=tk.LEFT, expand=True)
        msl_logo_label = load_logo("era_logo.png", (80, 80), center_logo_frame, "MSL")
        msl_logo_label.pack()
        
        right_frame = tk.Frame(banner_frame, bg="#a8328d")
        right_frame.pack(side=tk.RIGHT, padx=10)
        institute_logo_label = load_logo("iitk_logo.png", (60, 60), right_frame, "IITK")
        institute_logo_label.pack(side=tk.RIGHT, padx=5)

        self.refbox_status_label = tk.Label(right_frame, text="RefBox: Disconnected", fg="red", bg="#a8328d", font=("Arial", 10, "bold"))
        self.refbox_status_label.pack(side=tk.RIGHT, padx=10)
        self.refbox_connect_btn = tk.Button(right_frame, text="Connect RefBox", command=self.handle_refbox_connect, font=("Arial", 10))
        self.refbox_connect_btn.pack(side=tk.RIGHT, padx=5)


        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        content_frame = tk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_panel = tk.Frame(content_frame, width=400)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0,5))
        left_panel.pack_propagate(False)

        tk.Label(left_panel, text="Team Robots", font=("Arial", 12, "bold")).pack(pady=(0,5))
        robot_grid = tk.Frame(left_panel)
        robot_grid.pack(fill=tk.BOTH, expand=True)

        for i, robot in enumerate(self.robots):
            row = i // 2
            col = i % 2
            robot_frame = tk.Frame(robot_grid, width=180, height=180, bd=2, relief=tk.RAISED)
            robot_frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")
            robot_frame.grid_propagate(False) 

            robot_grid.grid_rowconfigure(row, weight=1)
            robot_grid.grid_columnconfigure(col, weight=1)

            tk.Label(robot_frame, text=f"{robot.name}", font=("Arial", 11, "bold")).pack(pady=(5,2))
            
            # Container for the robot image/text, with fixed size
            img_label_container = tk.Frame(robot_frame, width=150, height=100, bg="lightgrey") # Added bg for visibility
            img_label_container.pack(pady=5) # pady gives some spacing
            img_label_container.pack_propagate(False) # Prevent children from resizing this container

            robot_image_label = None 
            if PIL_AVAILABLE:
                try:
                    bot_img_path = "bot.png" 
                    # Ensure bot.png is in the correct path or provide an absolute path for testing
                    # print(f"DEBUG: Trying to load bot image from: {os.path.abspath(bot_img_path)}")
                    bot_img = Image.open(bot_img_path).resize((120, 90)) # Resized image
                    bot_photo = ImageTk.PhotoImage(bot_img)
                    robot_image_label = tk.Label(img_label_container, image=bot_photo, bg=img_label_container.cget("bg"))
                    robot_image_label.image = bot_photo # Keep a reference!
                except FileNotFoundError:
                    print(f"ERROR: bot.png not found at {os.path.abspath(bot_img_path)}")
                    robot_image_label = tk.Label(img_label_container, text="bot.png missing", fg="red", bg="white", width=18, height=4)
                except Exception as e:
                    print(f"Error loading bot.png: {e}")
                    # Fallback text label if image loading fails, give it explicit size
                    robot_image_label = tk.Label(img_label_container, text="No Image", fg="black", bg="white", width=18, height=4) # width/height in text units
            else:
                # Fallback text label if Pillow is not available
                robot_image_label = tk.Label(img_label_container, text="No Image (PIL)", fg="black", bg="white", width=18, height=4)
            
            if robot_image_label:
                # Place the label (image or text) in the center of its container
                robot_image_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
                robot_image_label.bind("<Button-1>", lambda event, r=robot: self.show_robot_detail(r))
            else:
                # This case should ideally not be reached with the logic above
                error_label = tk.Label(img_label_container, text="Display Error", bg="red", fg="white")
                error_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)


            status_text = "Connected" if robot.connected else "Disconnected"
            status_color = "green" if robot.connected else "red"
            status_label = tk.Label(robot_frame, text=status_text, fg=status_color, font=("Arial", 9, "bold"))
            status_label.pack(pady=(2,0))

            battery_str = f"Batt: {robot.parameters['battery_level']}%"
            battery_label = tk.Label(robot_frame, text=battery_str, fg="blue", font=("Arial", 9))
            battery_label.pack(pady=(0,5))

            robot.status_label = status_label 
            robot.battery_label = battery_label
        # ... (rest of setup_ui: middle_panel, logging_panel, bottom_panel - assumed unchanged) ...
        middle_panel = tk.Frame(content_frame)
        middle_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        tk.Label(middle_panel, text="Global Field View", font=("Arial", 12, "bold")).pack(pady=(0,5))
        self.field_canvas = tk.Canvas(middle_panel, bg="#3A5F0B", height=400) # Darker green
        self.field_canvas.pack(fill=tk.BOTH, expand=True, pady=5)
        self.draw_field()
        self.field_canvas.bind("<Configure>", lambda e: self.redraw_field())


        logging_panel = tk.Frame(content_frame, width=300, bd=1, relief=tk.SUNKEN)
        logging_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(5,0))
        logging_panel.pack_propagate(False)

        tk.Label(logging_panel, text="Event Logs", font=("Arial", 12, "bold")).pack(pady=5)
        self.logging_text = tk.Text(logging_panel, wrap=tk.WORD, font=("Arial", 9), height=10)
        self.logging_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=(0,5))

        log_buttons_frame = tk.Frame(logging_panel)
        log_buttons_frame.pack(fill=tk.X, pady=(0,5))
        tk.Button(log_buttons_frame, text="Save Log", command=self.save_log, font=("Arial", 9)).pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)

        bottom_panel = tk.Frame(main_frame, height=60, relief=tk.GROOVE, bd=1)
        bottom_panel.pack(fill=tk.X, pady=5, padx=10)
        additional_btn_frame = tk.Frame(bottom_panel)
        additional_btn_frame.pack(pady=5) 

        tk.Button(additional_btn_frame, text="Play/Pause", width=12, command=self.play_pause, font=("Arial", 10)).pack(side=tk.LEFT, padx=10)
        tk.Button(additional_btn_frame, text="Reset Positions", width=12, command=self.reset_position, font=("Arial", 10)).pack(side=tk.LEFT, padx=10)
        tk.Button(additional_btn_frame, text="Camera Check", width=12, command=self.camera_check, font=("Arial", 10)).pack(side=tk.LEFT, padx=10)

    # ... (handle_refbox_connect, update_refbox_status, log_refbox_message - assumed unchanged) ...
    def handle_refbox_connect(self):
        if self.logic:
            if not self.logic.refbox_handler.connected:
                self.log_message("Attempting to connect to RefBox...\n")
                self.logic.connect_to_refbox()
            else:
                self.log_message("RefBox is already connected.\n")
        else:
            self.log_message("Logic module not ready.\n")

    def update_refbox_status(self, connected):
        if connected:
            self.refbox_status_label.config(text="RefBox: Connected", fg="green")
            self.refbox_connect_btn.config(text="Disconnect RefBox") 
        else:
            self.refbox_status_label.config(text="RefBox: Disconnected", fg="red")
            self.refbox_connect_btn.config(text="Connect RefBox") 

    def log_refbox_message(self, message):
        self.log_message(f"RefBox: {message}\n")

    # Drawing functions: draw_field, redraw_field (modified highlight_robot_id in draw_robots_on_field)
    def draw_field(self):
        try:
            w = self.field_canvas.winfo_width()
            h = self.field_canvas.winfo_height()
        except tk.TclError: 
            return
        if w <=1 or h <=1: 
            return

        self.field_canvas.delete("all")
        self.draw_soccer_lines(self.field_canvas, w, h, self.global_world.field_dimensions)
        # For global field, no specific robot is highlighted by default unless we add such a feature
        self.draw_robots_on_field(self.field_canvas, self.robots, w, h, self.global_world.field_dimensions, highlight_robot_id=None)
        self.draw_robots_on_field(self.field_canvas, self.opponents, w, h, self.global_world.field_dimensions)
        self.draw_ball_on_field(self.field_canvas, self.global_world.ball_position, w, h, self.global_world.field_dimensions)

    def redraw_field(self):
        self.draw_field()

    def draw_soccer_lines(self, canvas, canvas_w, canvas_h, field_dims_m, view_center_m=None, view_range_m=None):
        # ... (This function seemed okay, ensure it correctly handles view_center_m=None for full field) ...
        field_w_m, field_h_m = field_dims_m
        
        if view_center_m and view_range_m: 
            scale_x = canvas_w / view_range_m
            scale_y = canvas_h / view_range_m
            origin_x_canvas, origin_y_canvas = 0, 0
            view_tl_m_x = view_center_m[0] - view_range_m / 2
            view_tl_m_y = view_center_m[1] - view_range_m / 2
        else: 
            margin = 10 
            drawable_w = canvas_w - 2 * margin
            drawable_h = canvas_h - 2 * margin
            scale_x = drawable_w / field_w_m
            scale_y = drawable_h / field_h_m
            origin_x_canvas, origin_y_canvas = margin, margin
            view_tl_m_x, view_tl_m_y = 0, 0

        def m_to_px(m_x, m_y):
            px_x = origin_x_canvas + (m_x - view_tl_m_x) * scale_x
            px_y = origin_y_canvas + (m_y - view_tl_m_y) * scale_y
            return px_x, px_y

        tl_px = m_to_px(0, 0)
        br_px = m_to_px(field_w_m, field_h_m)
        canvas.create_rectangle(tl_px[0], tl_px[1], br_px[0], br_px[1], outline="white", width=2)

        cl_start_px = m_to_px(field_w_m / 2, 0)
        cl_end_px = m_to_px(field_w_m / 2, field_h_m)
        canvas.create_line(cl_start_px[0], cl_start_px[1], cl_end_px[0], cl_end_px[1], fill="white", width=2)

        center_circle_radius_m = 0.75 
        cc_center_m_x, cc_center_m_y = field_w_m / 2, field_h_m / 2
        cc_tl_px = m_to_px(cc_center_m_x - center_circle_radius_m, cc_center_m_y - center_circle_radius_m)
        cc_br_px = m_to_px(cc_center_m_x + center_circle_radius_m, cc_center_m_y + center_circle_radius_m)
        canvas.create_oval(cc_tl_px[0], cc_tl_px[1], cc_br_px[0], cc_br_px[1], outline="white", width=2)
        
        goal_width_m = 0.6 
        goal_depth_px = 5 
        
        blue_goal_y1_m = field_h_m / 2 - goal_width_m / 2
        blue_goal_y2_m = field_h_m / 2 + goal_width_m / 2
        # Ensure scale_x is not zero before division
        blue_goal_depth_m = goal_depth_px / scale_x if scale_x != 0 else 0.1 
        bg1_px = m_to_px(0, blue_goal_y1_m)
        bg2_px = m_to_px(blue_goal_depth_m , blue_goal_y2_m) 
        canvas.create_rectangle(bg1_px[0]-goal_depth_px, bg1_px[1], bg2_px[0], bg2_px[1], fill="#4169E1", outline="#4169E1")

        yellow_goal_y1_m = field_h_m / 2 - goal_width_m / 2
        yellow_goal_y2_m = field_h_m / 2 + goal_width_m / 2
        yellow_goal_depth_m = goal_depth_px / scale_x if scale_x != 0 else 0.1
        yg1_px = m_to_px(field_w_m - yellow_goal_depth_m, yellow_goal_y1_m)
        yg2_px = m_to_px(field_w_m, yellow_goal_y2_m)
        canvas.create_rectangle(yg1_px[0], yg1_px[1], yg2_px[0]+goal_depth_px, yg2_px[1], fill="#FFD700", outline="#FFD700")


    def draw_robots_on_field(self, canvas, robots_to_draw, canvas_w, canvas_h, field_dims_m, 
                             view_center_m=None, view_range_m=None, highlight_robot_id=None):
        field_w_m, field_h_m = field_dims_m
        robot_radius_px = 8 

        if view_center_m and view_range_m: 
            scale_x = canvas_w / view_range_m
            scale_y = canvas_h / view_range_m
            origin_x_canvas, origin_y_canvas = 0,0
            view_tl_m_x = view_center_m[0] - view_range_m / 2
            view_tl_m_y = view_center_m[1] - view_range_m / 2
        else: 
            margin = 10
            drawable_w = canvas_w - 2 * margin
            drawable_h = canvas_h - 2 * margin
            scale_x = drawable_w / field_w_m if field_w_m > 0 else 1
            scale_y = drawable_h / field_h_m if field_h_m > 0 else 1
            origin_x_canvas, origin_y_canvas = margin, margin
            view_tl_m_x, view_tl_m_y = field_w_m/2,field_h_m/2

        for robot_obj in robots_to_draw: 
            if not hasattr(robot_obj, 'position') or not robot_obj.position or len(robot_obj.position) < 2:
                # print(f"Skipping robot {robot_obj.robot_id if hasattr(robot_obj, 'robot_id') else 'Unknown'} due to missing/invalid position.")
                continue
            rx_m, ry_m = robot_obj.position[0], robot_obj.position[1]

            cx_px = origin_x_canvas + (rx_m + view_tl_m_x) * scale_x
            cy_px = origin_y_canvas + (-ry_m + view_tl_m_y) * scale_y
            
            canvas.create_oval(cx_px - robot_radius_px, cy_px - robot_radius_px, 
                               cx_px + robot_radius_px, cy_px + robot_radius_px, 
                               fill=robot_obj.color, outline="white", width=1)
            
            angle_rad = robot_obj.orientation
            line_len_px = 15 
            x_end_px = cx_px + line_len_px * math.cos(angle_rad)
            y_end_px = cy_px - line_len_px * math.sin(angle_rad)
            canvas.create_line(cx_px, cy_px, x_end_px, y_end_px, fill="white", width=2)
            
            canvas.create_text(cx_px, cy_px, text=str(robot_obj.robot_id), fill="black", font=("Arial", 7, "bold"))

            if highlight_robot_id is not None and robot_obj.robot_id == highlight_robot_id:
                 # Check if the robot to be highlighted is one of our team's robots
                 is_our_team_detailed_robot = any(r.robot_id == highlight_robot_id and r.color != "red" for r in self.robots) # crude check
                 if is_our_team_detailed_robot:
                    canvas.create_oval(cx_px - robot_radius_px - 3, cy_px - robot_radius_px - 3,
                                   cx_px + robot_radius_px + 3, cy_px + robot_radius_px + 3,
                                   outline="yellow", width=2)


    def draw_ball_on_field(self, canvas, ball_pos_m, canvas_w, canvas_h, field_dims_m, view_center_m=None, view_range_m=None):
        if not ball_pos_m or len(ball_pos_m) < 2: return

        field_w_m, field_h_m = field_dims_m
        ball_radius_px = 5

        if view_center_m and view_range_m:
            scale_x = canvas_w / view_range_m
            scale_y = canvas_h / view_range_m
            origin_x_canvas, origin_y_canvas = 0,0
            view_tl_m_x = view_center_m[0] - view_range_m / 2
            view_tl_m_y = view_center_m[1] - view_range_m / 2
        else: 
            margin = 10
            drawable_w = canvas_w - 2 * margin
            drawable_h = canvas_h - 2 * margin
            scale_x = drawable_w / field_w_m if field_w_m > 0 else 1
            scale_y = drawable_h / field_h_m if field_h_m > 0 else 1
            origin_x_canvas, origin_y_canvas = margin, margin
            view_tl_m_x, view_tl_m_y = field_w_m/2,field_h_m/2

        bx_m, by_m = ball_pos_m[0], ball_pos_m[1]
        cx_px = origin_x_canvas + (bx_m + view_tl_m_x) * scale_x
        cy_px = origin_y_canvas + (-by_m + view_tl_m_y) * scale_y
        
        canvas.create_oval(cx_px - ball_radius_px, cy_px - ball_radius_px, 
                           cx_px + ball_radius_px, cy_px + ball_radius_px, 
                           fill="orange", outline="black", width=1)


    def draw_obstacles_on_field(self, canvas, obstacles_m, canvas_w, canvas_h, field_dims_m, view_center_m=None, view_range_m=None):
        # ... (This function seemed okay, ensure it correctly handles view_center_m=None for full field) ...
        if not obstacles_m: return

        field_w_m, field_h_m = field_dims_m
        obstacle_radius_px = 4

        if view_center_m and view_range_m:
            scale_x = canvas_w / view_range_m
            scale_y = canvas_h / view_range_m
            origin_x_canvas, origin_y_canvas = 0,0
            view_tl_m_x = view_center_m[0] - view_range_m / 2
            view_tl_m_y = view_center_m[1] - view_range_m / 2
        else: 
            margin = 10
            drawable_w = canvas_w - 2 * margin
            drawable_h = canvas_h - 2 * margin
            scale_x = drawable_w / field_w_m if field_w_m > 0 else 1
            scale_y = drawable_h / field_h_m if field_h_m > 0 else 1
            origin_x_canvas, origin_y_canvas = margin, margin
            view_tl_m_x, view_tl_m_y = field_w_m/2,field_h_m/2
        
        for obs_m in obstacles_m:
            if not obs_m or len(obs_m) < 2: continue
            ox_m, oy_m = obs_m[0], obs_m[1]
            cx_px = origin_x_canvas + (ox_m + view_tl_m_x) * scale_x
            cy_px = origin_y_canvas + (-oy_m + view_tl_m_y) * scale_y
            
            canvas.create_rectangle(cx_px - obstacle_radius_px, cy_px - obstacle_radius_px, 
                                   cx_px + obstacle_radius_px, cy_px + obstacle_radius_px, 
                                   fill="gray", outline="black")


    def show_robot_detail(self, robot):
        print(f"DEBUG: show_robot_detail called for {robot.name}, ID: {robot.robot_id}") # DEBUG LINE
        if not isinstance(robot, Robot):
            print(f"DEBUG: Invalid robot object passed to show_robot_detail: {robot}")
            return

        self.current_detailed_robot = robot
        detail_window = tk.Toplevel(self.root)
        detail_window.title(f"Detailed View - {robot.name}")
        detail_window.geometry("800x650")
        detail_window.transient(self.root) # Keep on top
        # detail_window.grab_set() # Make it modal (optional, can be annoying)


        info_frame = tk.Frame(detail_window, pady=5)
        info_frame.pack(fill=tk.X)
        tk.Label(info_frame, text=f"{robot.name}", font=("Arial", 16, "bold")).pack(side=tk.LEFT, padx=20)
        status_text = "Connected" if robot.connected else "Disconnected"
        status_color = "green" if robot.connected else "red"
        tk.Label(info_frame, text=f"Status: {status_text}", fg=status_color, font=("Arial", 12, "bold")).pack(side=tk.LEFT, padx=20)
        tk.Label(info_frame, text=f"Battery: {robot.parameters.get('battery_level', 'N/A')}%", font=("Arial", 12)).pack(side=tk.LEFT, padx=20)
        tk.Button(info_frame, text="Parameters...", command=self.open_parameters_window, font=("Arial", 10)).pack(side=tk.RIGHT, padx=20)

        content_frame = tk.Frame(detail_window)
        content_frame.pack(fill=tk.BOTH, expand=True, padx=10)

        param_display_frame = tk.LabelFrame(content_frame, text="Current Parameters", padx=10, pady=10)
        param_display_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0,10))
        
        self.robot_param_labels.clear() # Clear previous labels if any
        for i, (param, value) in enumerate(robot.parameters.items()):
            tk.Label(param_display_frame, text=param.replace('_', ' ').title() + ":", font=("Arial", 9)).grid(row=i, column=0, sticky="w", pady=2)
            val_label = tk.Label(param_display_frame, text=str(value), font=("Arial", 9, "bold"))
            val_label.grid(row=i, column=1, sticky="e", padx=5, pady=2)
            self.robot_param_labels[param] = val_label

        local_map_frame = tk.Frame(content_frame)
        local_map_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        tk.Label(local_map_frame, text="Local World Map (Robot's Perception on Full Field)", font=("Arial", 11, "bold")).pack(pady=(0,5))
        local_map_canvas = tk.Canvas(local_map_frame, bg="#556B2F", relief=tk.SUNKEN, bd=1)
        local_map_canvas.pack(fill=tk.BOTH, expand=True)

        def update_local_map_display(event=None):
            # print(f"DEBUG: update_local_map_display for {robot.name}") 
            if not robot: 
                return
            try:
                if not local_map_canvas.winfo_exists(): 
                    return
                w_local = local_map_canvas.winfo_width()
                h_local = local_map_canvas.winfo_height()
            except tk.TclError:
                return 
            if w_local <= 1 or h_local <= 1:
                return

            local_map_canvas.delete("all")
            
            # 1. Draw Soccer Lines (Full Field View)
            self.draw_soccer_lines(local_map_canvas, w_local, h_local, 
                                   self.global_world.field_dimensions, 
                                   view_center_m=None, view_range_m=None) # view_...=None for full field
            
            # 2. Draw ONLY the current detailed robot, highlighted
            #    Pass it as a list containing just this one robot.
            self.draw_robots_on_field(local_map_canvas, [robot], w_local, h_local, # MODIFIED: Only current robot
                                       self.global_world.field_dimensions,
                                       view_center_m=None, view_range_m=None, 
                                       highlight_robot_id=robot.robot_id) 
            
            # 3. Draw the ball AS PERCEIVED BY THIS ROBOT
            if robot.local_ball_position: # This is from the robot's own sensors
                 self.draw_ball_on_field(local_map_canvas, robot.local_ball_position, w_local, h_local,
                                        self.global_world.field_dimensions,
                                        view_center_m=None, view_range_m=None)

            # 4. Draw obstacles AS PERCEIVED BY THIS ROBOT
            if robot.local_obstacles: # These are from the robot's own sensors
                # Assuming robot.local_obstacles is a list of [x, y] coordinates
                # These will be drawn as generic obstacles.
                self.draw_obstacles_on_field(local_map_canvas, robot.local_obstacles, w_local, h_local,
                                             self.global_world.field_dimensions,
                                             view_center_m=None, view_range_m=None)
            # print(f"DEBUG: Local map for {robot.name} updated. Ball: {robot.local_ball_position}, Obstacles: {len(robot.local_obstacles)}")

        # Store references and schedule initial draw
        robot.local_map_canvas = local_map_canvas # Store on the robot object
        robot.update_local_map_display_func = update_local_map_display # Store on the robot object

        local_map_canvas.bind("<Configure>", update_local_map_display)
        # Use 'after' on the detail_window or root to schedule the first draw.
        # Ensure it's called after the window is likely visible and sized.
        detail_window.after(100, update_local_map_display) 

        control_frame = tk.Frame(detail_window, pady=10)
        control_frame.pack(fill=tk.X)
        tk.Button(control_frame, text="Test Kick Angle", command=lambda: self.test_robot("test_kick_angle"), font=("Arial", 9)).pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        tk.Button(control_frame, text="Charge", command=lambda: self.test_robot("charge"), font=("Arial", 9)).pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        tk.Button(control_frame, text="Kick", command=lambda: self.test_robot("kick"), font=("Arial", 9)).pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        tk.Button(control_frame, text="Close", command=detail_window.destroy, font=("Arial", 9, "bold")).pack(side=tk.RIGHT, padx=10)

        movement_controls_frame = tk.Frame(detail_window, pady=5)
        movement_controls_frame.pack()
        btn_font = ("Arial", 10)
        btn_width = 4
        tk.Button(movement_controls_frame, text="↑", width=btn_width, font=btn_font, command=lambda: self.move_robot("forward")).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(movement_controls_frame, text="←", width=btn_width, font=btn_font, command=lambda: self.move_robot("left")).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(movement_controls_frame, text="Stop", width=btn_width, font=btn_font, command=lambda: self.move_robot("stop")).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(movement_controls_frame, text="→", width=btn_width, font=btn_font, command=lambda: self.move_robot("right")).grid(row=1, column=2, padx=2, pady=2)
        tk.Button(movement_controls_frame, text="↓", width=btn_width, font=btn_font, command=lambda: self.move_robot("backward")).grid(row=2, column=1, padx=2, pady=2)
        tk.Button(movement_controls_frame, text="⟲", width=btn_width, font=btn_font, command=lambda: self.move_robot("rotate_left")).grid(row=1, column=3, padx=5, pady=2) 
        tk.Button(movement_controls_frame, text="⟳", width=btn_width, font=btn_font, command=lambda: self.move_robot("rotate_right")).grid(row=1, column=4, padx=5, pady=2)
        
        # print(f"DEBUG: show_robot_detail for {robot.name} window setup finished.") # DEBUG

    def refresh_robot_detail_view(self):
        if self.current_detailed_robot and \
           hasattr(self.current_detailed_robot, 'local_map_canvas') and \
           self.current_detailed_robot.local_map_canvas.winfo_exists() and \
           hasattr(self.current_detailed_robot, 'update_local_map_display_func'):
            
            # Update parameter display
            if self.robot_param_labels: # Check if the dict itself exists
                 for param, value in self.current_detailed_robot.parameters.items():
                     if param in self.robot_param_labels and self.robot_param_labels[param].winfo_exists():
                         self.robot_param_labels[param].config(text=str(value))
            # Update map
            self.current_detailed_robot.update_local_map_display_func()
        # else:
            # print("DEBUG: refresh_robot_detail_view: No current_detailed_robot or its canvas/func is missing/destroyed.")

    # ... (open_parameters_window and other methods - assumed mostly unchanged, check for parent=param_window in messageboxes) ...
    def open_parameters_window(self):
        if not self.current_detailed_robot:
            if not self.robots:
                messagebox.showerror("Error", "No robots available to configure.")
                return
            self.current_detailed_robot = self.robots[0] 

        param_window = tk.Toplevel(self.root)
        param_window.title(f"Parameters - {self.current_detailed_robot.name}")
        param_window.geometry("450x550") 
        param_window.transient(self.root) 
        param_window.grab_set() 

        main_param_frame = tk.Frame(param_window, padx=15, pady=15)
        main_param_frame.pack(fill=tk.BOTH, expand=True)
        
        tk.Label(main_param_frame, text=f"Configure Parameters for {self.current_detailed_robot.name}", font=("Arial", 13, "bold")).pack(pady=(0,15))

        entries_frame = tk.Frame(main_param_frame)
        entries_frame.pack(fill=tk.X)

        entries = {}
        for i, (param, value) in enumerate(self.current_detailed_robot.parameters.items()):
            row_frame = tk.Frame(entries_frame)
            row_frame.pack(fill=tk.X, pady=3)
            tk.Label(row_frame, text=param.replace('_', ' ').title() + ":", width=25, anchor="w", font=("Arial", 10)).pack(side=tk.LEFT)
            entry = tk.Entry(row_frame, width=15, font=("Arial", 10))
            entry.insert(0, str(value))
            entry.pack(side=tk.LEFT, padx=5)
            entries[param] = entry

        buttons_frame = tk.Frame(main_param_frame, pady=15)
        buttons_frame.pack(fill=tk.X)

        def save_current_parameters():
            try:
                updated_params = {}
                for param, entry_widget in entries.items(): # Renamed to entry_widget
                    value_str = entry_widget.get() # Use entry_widget
                    try: 
                        if '.' in value_str or 'e' in value_str.lower(): # Check for float characters
                            updated_params[param] = float(value_str)
                        else: 
                            updated_params[param] = int(value_str)
                    except ValueError:
                        updated_params[param] = value_str 
                
                self.current_detailed_robot.set_parameters(updated_params)
                self.refresh_robot_detail_view() 
                if 'battery_level' in updated_params and \
                   hasattr(self.current_detailed_robot, 'battery_label') and \
                   self.current_detailed_robot.battery_label.winfo_exists():
                    self.current_detailed_robot.battery_label.config(text=f"Batt: {updated_params['battery_level']}%")

                self.log_message(f"Parameters for {self.current_detailed_robot.name} updated locally.\n")
            except ValueError:
                messagebox.showerror("Error", "Invalid parameter value. Please enter appropriate values.", parent=param_window)

        def send_parameters_to_robot():
            save_current_parameters() 
            self.log_message(f"Sending parameters to {self.current_detailed_robot.name}:\n")
            param_data_to_send = {}
            for param, val in self.current_detailed_robot.parameters.items():
                self.log_message(f"  {param}: {val}\n")
                param_data_to_send[param] = val
            
            if param_data_to_send:
                 msg_to_send = json.dumps({"type": "set_parameters", "parameters": param_data_to_send})
                 self.current_detailed_robot.send_to_robot(msg_to_send)
            messagebox.showinfo("Sent", f"Parameters sent to {self.current_detailed_robot.name}.", parent=param_window)

        def send_to_all_robots():
            save_current_parameters() 
            current_params = self.current_detailed_robot.parameters.copy()
            
            num_sent = 0
            for rbt in self.robots: # Renamed to rbt to avoid conflict
                if rbt.connected:
                    rbt.set_parameters(current_params) 
                    param_data_to_send = current_params
                    msg_to_send = json.dumps({"type": "set_parameters", "parameters": param_data_to_send})
                    rbt.send_to_robot(msg_to_send)
                    self.log_message(f"Sent parameters to {rbt.name}\n")
                    num_sent +=1
            messagebox.showinfo("Sent to All", f"Parameters sent to {num_sent} connected robots.", parent=param_window)

        def save_params_to_file_action():
            save_current_parameters() 
            self.save_parameters_to_file(self.current_detailed_robot.parameters, parent_window=param_window)

        def load_params_from_file_action():
            loaded_params = self.load_parameters_from_file(parent_window=param_window)
            if loaded_params:
                for param, entry_widget in entries.items():
                    if param in loaded_params:
                        entry_widget.delete(0, tk.END)
                        entry_widget.insert(0, str(loaded_params[param]))
                save_current_parameters() 
                messagebox.showinfo("Success", "Parameters loaded into form.", parent=param_window)

        btn_font_small = ("Arial", 9)
        tk.Button(buttons_frame, text="Apply Locally", command=save_current_parameters, font=btn_font_small).pack(side=tk.LEFT, padx=3, expand=True,fill=tk.X)
        tk.Button(buttons_frame, text="Send to Robot", command=send_parameters_to_robot, font=btn_font_small).pack(side=tk.LEFT, padx=3, expand=True,fill=tk.X)
        tk.Button(buttons_frame, text="Send to All", command=send_to_all_robots, font=btn_font_small).pack(side=tk.LEFT, padx=3, expand=True,fill=tk.X)
        
        file_buttons_frame = tk.Frame(main_param_frame) 
        file_buttons_frame.pack(fill=tk.X, pady=5)
        tk.Button(file_buttons_frame, text="Load from File", command=load_params_from_file_action, font=btn_font_small).pack(side=tk.LEFT, padx=3, expand=True,fill=tk.X)
        tk.Button(file_buttons_frame, text="Save to File", command=save_params_to_file_action, font=btn_font_small).pack(side=tk.LEFT, padx=3, expand=True,fill=tk.X)

        tk.Button(main_param_frame, text="Close", command=param_window.destroy, font=("Arial", 10, "bold")).pack(pady=(10,0))


    def save_parameters_to_file(self, parameters_to_save, parent_window=None):
        # ... (ensure parent=parent_window for filedialog and messagebox) ...
        filename = filedialog.asksaveasfilename(title="Save Parameters", defaultextension=".json",
                                               filetypes=[("JSON files", "*.json"), ("Text files", "*.txt"), ("All files", "*.*")],
                                               parent=parent_window or self.root)
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(parameters_to_save, f, indent=4)
                self.log_message(f"Parameters saved to {filename}\n")
                messagebox.showinfo("Success", f"Parameters saved to {filename}", parent=parent_window or self.root)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save parameters: {e}", parent=parent_window or self.root)


    def load_parameters_from_file(self, parent_window=None):
        # ... (ensure parent=parent_window for filedialog and messagebox) ...
        filename = filedialog.askopenfilename(title="Load Parameters",
                                             filetypes=[("JSON files", "*.json"), ("Text files", "*.txt"), ("All files", "*.*")],
                                             parent=parent_window or self.root)
        if filename:
            try:
                with open(filename, 'r') as f:
                    loaded_params = json.load(f)
                self.log_message(f"Parameters loaded from {filename}\n")
                return loaded_params
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load parameters: {e}", parent=parent_window or self.root)
                return None
        return None

    def move_robot(self, direction):
        # ... (No changes, but ensure self.current_detailed_robot is valid) ...
        if self.current_detailed_robot and self.current_detailed_robot.connected:
            msg = {"type": "move", "direction": direction}
            self.current_detailed_robot.send_to_robot(json.dumps(msg))
            self.log_message(f"Move command '{direction}' sent to {self.current_detailed_robot.name}\n")
        elif not self.current_detailed_robot :
             self.log_message("No robot selected for movement.\n")
        else:
            self.log_message(f"Cannot move {self.current_detailed_robot.name}: Not connected.\n")


    def test_robot(self, test_action):
        # ... (No changes, but ensure self.current_detailed_robot is valid) ...
        if self.current_detailed_robot and self.current_detailed_robot.connected:
            msg = {"type": "test", "action": test_action}
            self.current_detailed_robot.send_to_robot(json.dumps(msg))
            self.log_message(f"Test command '{test_action}' sent to {self.current_detailed_robot.name}\n")
        elif not self.current_detailed_robot :
             self.log_message("No robot selected for test command.\n")
        else:
            self.log_message(f"Cannot test {self.current_detailed_robot.name}: Not connected.\n")

    def save_log(self):
        # ... (No changes) ...
        if not self.logging_text: return
        filename = filedialog.asksaveasfilename(title="Save Log", defaultextension=".log",
                                               filetypes=[("Log files", "*.log"), ("Text files", "*.txt"), ("All files", "*.*")])
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write(self.logging_text.get("1.0", tk.END))
                messagebox.showinfo("Log Saved", f"Log saved to {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save log: {e}")

    def log_message(self, msg):
        # ... (No changes) ...
        if self.logging_text and self.logging_text.winfo_exists():
            self.logging_text.insert(tk.END, msg)
            self.logging_text.see(tk.END) 

    def play_pause(self):
        # ... (No changes) ...
        self.is_playing = not self.is_playing
        command_type = "PLAY" if self.is_playing else "PAUSE"
        log_msg = "Resuming operation..." if self.is_playing else "Pausing operation..."
        self.log_message(log_msg + "\n")
        
        for robot_obj in self.robots: # Renamed to avoid conflict
            if robot_obj.connected:
                robot_obj.send_to_robot(json.dumps({"type": "command", "command": command_type}))
        if not any(r.connected for r in self.robots):
            self.log_message("No robots connected to send Play/Pause command.\n")

    def reset_position(self):
        # ... (No changes) ...
        self.log_message("Sending RESET POSITION command to all connected robots...\n")
        for robot_obj in self.robots: # Renamed to avoid conflict
            if robot_obj.connected:
                robot_obj.send_to_robot(json.dumps({"type": "command", "command": "RESET_POSITION"}))
        if not any(r.connected for r in self.robots):
            self.log_message("No robots connected to send Reset Position command.\n")

    def camera_check(self):
        # ... (No changes) ...
        self.log_message("Sending CHECK CAMERA command to all connected robots...\n")
        for robot_obj in self.robots: # Renamed to avoid conflict
            if robot_obj.connected:
                robot_obj.send_to_robot(json.dumps({"type": "command", "command": "CHECK_CAMERA"}))
        if not any(r.connected for r in self.robots):
            self.log_message("No robots connected to send Camera Check command.\n")
            
    def update_robot_ui_elements(self):
        for robot_obj in self.robots: # Renamed variable
            if hasattr(robot_obj, 'status_label') and robot_obj.status_label.winfo_exists():
                status_text = "Connected" if robot_obj.connected else "Disconnected"
                status_color = "green" if robot_obj.connected else "red"
                robot_obj.status_label.config(text=status_text, fg=status_color)
            if hasattr(robot_obj, 'battery_label') and robot_obj.battery_label.winfo_exists():
                robot_obj.battery_label.config(text=f"Batt: {robot_obj.parameters.get('battery_level', 'N/A')}%")
        self.refresh_robot_detail_view()