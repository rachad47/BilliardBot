import cv2
import customtkinter as ctk
from PIL import Image, ImageTk
import numpy as np
from main import get_processed_frame
from tkdial import Meter, Dial, Jogwheel
import tkinter as tk
import time
import threading
from robot_control import send_command, calculate_rotation_steps, calculate_translation_steps, send_strike_command, getCartesianStepsAndSpeed, check_movement_complete
from constants import MOTOR_SPEED, LOWER_CENTER, UPPER_CENTER, LOWER_Y_AXIS, UPPER_Y_AXIS, LOWER_BALL, UPPER_BALL, LOWER_TABLE, UPPER_TABLE,LOWER_ROBOT,UPPER_ROBOT , POOL_BALL_DIAMETER

# Constants
UPDATE_DELAY_MS = 10

# Global variables
ball_measurements = None
cap = None
root = None
logo_photo = None
main_frame = None
robot_control_frame = None
video_frame = None
top_frame = None
buttons_frame = None
label = None
switch_var = None
hold_var = None
sensitivity_entry = None
charging_time_entry = None
ipaddress_entry = None
dial3 = None
dial4 = None
slider_distance = None
X_direction = None
Y_direction = None
ball_sliders_frame = None
y_axis_sliders_frame = None
center_sliders_frame = None
table_sliders_frame = None
robot_sliders_frame = None
thresholds = []
center_sliders = [] 
directory = "Software"



def initialize_gui():
    global cap, root, logo_photo, thresholds
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 850)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 700)

    root = ctk.CTk()
    root.title("Billiard Bot")

    logo_image = Image.open(f"{directory}/logo.jpg").resize((80, 80))
    logo_photo = ImageTk.PhotoImage(logo_image)

    # Load thresholds if file exists
    try:
        with open(f"{directory}/thresholds.txt", "r") as file:
            loaded_thresholds = [np.array(list(map(int, line.strip().split(",")))) for line in file]
            thresholds = loaded_thresholds
    except FileNotFoundError:
        print("Thresholds file not found")
        pass  
    


def create_frame(parent, side='top', fill='both', expand=True, height=None, fg_color=None):
    frame = ctk.CTkFrame(parent, corner_radius=10, fg_color=fg_color)
    if height:
        frame.configure(height=height)  
    frame.pack(side=side, fill=fill, expand=expand)
    frame.columnconfigure(0, weight=1)
    frame.rowconfigure(1, weight=1)
    return frame



def create_main_frames():
    global main_frame, robot_control_frame, video_frame, top_frame, buttons_frame

    main_frame = ctk.CTkFrame(root)
    main_frame.pack(side="top", fill="both", expand=True, padx=10, pady=10)

    robot_control_frame = create_frame(main_frame, side="left", fill="both")
    video_frame = create_frame(main_frame, side="left", fill="both")
    top_frame = create_frame(video_frame, side="top", fill="x", height=100, fg_color='transparent')
    buttons_frame = create_frame(main_frame, side="left", fill="y")

    # Add logo and title to top_frame
    logo_label = ctk.CTkLabel(top_frame, image=logo_photo, text="")
    logo_label.image = logo_photo
    logo_label.pack(side="left", padx=10)

    title_label = ctk.CTkLabel(top_frame, text="BilliardBot", font=("Arial", 36), fg_color=None)
    title_label.pack(side="left", padx=10, expand=True)


def update_camera_feed():
    global ball_measurements
    ret, frame = cap.read()
    if ret:
        frame, ball_measurements = get_processed_frame(cap, thresholds)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img)
        label.imgtk = imgtk
        label.configure(image=imgtk)

    label.after(UPDATE_DELAY_MS, update_camera_feed)

def update_threshold_values(slider_set, value_labels, indices):
    for i, slider in enumerate(slider_set):
        value = int(slider.get())
        value_labels[i].configure(text=str(value))
        if i % 3 == 0:
            set_index = indices[i // 3]
            thresholds[set_index] = np.array([
                value,
                int(slider_set[i + 1].get()),
                int(slider_set[i + 2].get())
            ])


def create_sliders(frame, labels, update_function, defaults, indices, title):
    # Create a label for the title
    title_label = ctk.CTkLabel(frame, text=title, font=("Arial", 20), fg_color=None)
    title_label.grid(row=0, columnspan=1, pady=20)  

    sliders = []
    value_labels = []
    starting_row = 1  # Starting on the second row, since the title is on the first row
    for i, label in enumerate(labels):
        row = starting_row + i // 3
        label_column = (i % 3) * 2
        slider_column = label_column + 1

        slider_label = ctk.CTkLabel(frame, text=label)
        slider_label.grid(row=row, column=label_column, sticky='e', padx=5, pady=5)

        # Pass 'i' as a default argument to lambda to avoid late binding issues
        slider = ctk.CTkSlider(frame, from_=0, to=179 if 'Hue' in label else 255, command=lambda x=None, i=i: update_function(sliders, value_labels, indices))
        slider.set(defaults[i])
        slider.grid(row=row, column=slider_column, padx=5, pady=5, sticky='ew')  # Use 'ew' to expand horizontally
        sliders.append(slider)

        frame.grid_columnconfigure(label_column, weight=3)  
        frame.grid_columnconfigure(slider_column, weight=1)  # Giving more weight to slider column

        # Create a label to display the value of the slider
        value_label = ctk.CTkLabel(frame, text=str(defaults[i]))
        value_label.grid(row=row, column=slider_column + 1, sticky='w', padx=5, pady=5)
        value_labels.append(value_label)

        # Add padding column for alignment if necessary
        frame.grid_columnconfigure(slider_column + 2, weight=1)

    return sliders, value_labels


def create_sliders_and_controls():
    global ball_sliders_frame, y_axis_sliders_frame, center_sliders_frame
    global table_sliders_frame, robot_sliders_frame
    global ball_sliders, ball_value_labels, y_axis_sliders, y_axis_value_labels
    global center_sliders, center_value_labels, table_sliders, table_value_labels
    global robot_sliders, robot_value_labels
    global label, buttons_frame, thresholds

    # Define labels and defaults for sliders
    ball_labels = ["Lower Hue", "Lower Saturation", "Lower Value", "Upper Hue", "Upper Saturation", "Upper Value"]
    y_axis_labels = ball_labels.copy()
    center_labels = ball_labels.copy()
    table_labels = ball_labels.copy()
    robot_labels = ball_labels.copy()

    ball_defaults = thresholds[0].tolist() + thresholds[1].tolist() if len(thresholds) > 1 else [20, 30, 50, 70, 180, 130]
    y_axis_defaults = thresholds[2].tolist() + thresholds[3].tolist() if len(thresholds) > 3 else [0, 100, 200, 12, 180, 255]
    center_defaults = thresholds[4].tolist() + thresholds[5].tolist() if len(thresholds) > 5 else [0, 0, 0, 179, 180, 150]
    table_defaults = thresholds[6].tolist() + thresholds[7].tolist() if len(thresholds) > 7 else [0, 0, 100, 179, 40, 255]
    robot_defaults = thresholds[8].tolist() + thresholds[9].tolist() if len(thresholds) > 9 else [0, 15, 168, 15, 70, 255]


    thresholds = [np.array(ball_defaults[:3]), np.array(ball_defaults[3:]), np.array(y_axis_defaults[:3]), np.array(y_axis_defaults[3:]), np.array(center_defaults[:3]), np.array(center_defaults[3:]), np.array(table_defaults[:3]), np.array(table_defaults[3:]), np.array(robot_defaults[:3]), np.array(robot_defaults[3:])]

    # Create and populate slider frames
    ball_sliders_frame = ctk.CTkFrame(root)
    y_axis_sliders_frame = ctk.CTkFrame(root)
    center_sliders_frame = ctk.CTkFrame(root)
    table_sliders_frame = ctk.CTkFrame(root)
    robot_sliders_frame = ctk.CTkFrame(root)

    ball_sliders, ball_value_labels = create_sliders(ball_sliders_frame, ball_labels, update_threshold_values, ball_defaults, [0, 1], "Ball Thresholds")
    y_axis_sliders, y_axis_value_labels = create_sliders(y_axis_sliders_frame, y_axis_labels, update_threshold_values, y_axis_defaults, [2, 3], "Y-Axis Thresholds")
    center_sliders, center_value_labels = create_sliders(center_sliders_frame, center_labels, update_threshold_values, center_defaults, [4, 5], "Center Thresholds")
    table_sliders, table_value_labels = create_sliders(table_sliders_frame, table_labels, update_threshold_values, table_defaults, [6, 7], "Table Thresholds")
    robot_sliders, robot_value_labels = create_sliders(robot_sliders_frame, robot_labels, update_threshold_values, robot_defaults, [8, 9], "Robot Thresholds")

    # Video label
    label = ctk.CTkLabel(video_frame, text="")
    label.pack(fill="both", expand=True, padx=10, pady=10)

    # Buttons frame and buttons
    button_label = ctk.CTkLabel(buttons_frame, text="Thresholds:", font=("Arial", 24), fg_color=None)
    button_label.pack(side="top", padx=50, pady=20)

    button1 = ctk.CTkButton(buttons_frame, text="Ball thresholds", command=lambda: toggle_sliders(ball_sliders_frame))
    button2 = ctk.CTkButton(buttons_frame, text="Y_axis thresholds", command=lambda: toggle_sliders(y_axis_sliders_frame))
    button3 = ctk.CTkButton(buttons_frame, text="Center thresholds", command=lambda: toggle_sliders(center_sliders_frame))
    button4 = ctk.CTkButton(buttons_frame, text="Table thresholds", command=lambda: toggle_sliders(table_sliders_frame))
    button5 = ctk.CTkButton(buttons_frame, text="Robot thresholds", command=lambda: toggle_sliders(robot_sliders_frame))

    for btn in [button1, button2, button3, button4, button5]:
        btn.pack(padx=20, pady=10, fill='x')

def toggle_sliders(frame_to_show):
    for frame in [ball_sliders_frame, y_axis_sliders_frame, center_sliders_frame, table_sliders_frame, robot_sliders_frame]:
        frame.pack_forget()
    frame_to_show.pack(side="top", fill="x", expand=True, padx=10, pady=10)


def setup_robot_control():
    global switch_var, hold_var, sensitivity_entry, charging_time_entry, ipaddress_entry
    global dial3, dial4, slider_distance, X_direction, Y_direction, text2, text4, text5, center_sliders

    # Create a label for the robot control
    robot_label = ctk.CTkLabel(robot_control_frame, text="Robot Control:", font=("Arial", 24), fg_color=None)
    robot_label.grid(row=0, columnspan=2, padx=10, pady=20)

    # Number input container
    Num_input_container = ctk.CTkFrame(robot_control_frame, fg_color='transparent')
    Num_input_container.grid(row=1, column=1, padx=10, pady=20)

    sensitivity_entry = ctk.CTkEntry(Num_input_container, placeholder_text="sensitivity", width=70)
    sensitivity_entry.pack(side="left", fill="both", expand=True, padx=10)

    charging_time_entry = ctk.CTkEntry(Num_input_container, placeholder_text="duration", width=70)
    charging_time_entry.pack(side="left", fill="both", expand=True, padx=10)

    ipaddress_entry = ctk.CTkEntry(Num_input_container, placeholder_text="ip address", width=120)
    ipaddress_entry.pack(side="left", fill="both", expand=True, padx=10)

    # Controller switch container
    controller_switch_container = ctk.CTkFrame(robot_control_frame, fg_color='transparent')
    controller_switch_container.grid(row=1, column=0, padx=10, pady=20)

    switch_var = ctk.StringVar(value="off")
    switch = ctk.CTkSwitch(controller_switch_container, text="Custom values", variable=switch_var, onvalue="on", offvalue="off")
    switch.pack(side="left", fill="both", expand=True, padx=10)

    hold_var = ctk.StringVar(value="off")
    hold = ctk.CTkSwitch(controller_switch_container, text="Hold", variable=hold_var, onvalue="on", offvalue="off")
    hold.pack(side="left", fill="both", expand=True, padx=10)

    # Polar coordinates section
    text1 = ctk.CTkLabel(robot_control_frame, text="Polar coordinates", font=("Arial", 20), fg_color=None)
    text1.grid(row=2, column=0, padx=10, pady=5)

    

    Polar_button = ctk.CTkButton(robot_control_frame, text="Send Polar Command", fg_color="#b165ff", width=280, command=lambda: send_Polar_command())
    Polar_button.grid(row=2, column=1, padx=10, pady=20)


    def send_Polar_command():
        global dial3, dial4, slider_distance, X_direction, Y_direction, text2, text4, text5, center_sliders
        if ball_measurements is not None:
            for center, radius, distance, angle, X_coordinate, Y_coordinate in ball_measurements:
                print(f"Distance = {distance:.1f} cm, Angle = {angle:.1f} degrees")

            rotation_steps = -calculate_rotation_steps(angle)
            translation_steps = calculate_translation_steps(distance/100)-100
            print(f"Rotation Steps: {rotation_steps}, Translation Steps: {translation_steps}")
            send_command(rotation_steps, MOTOR_SPEED, rotation_steps, MOTOR_SPEED, rotation_steps, MOTOR_SPEED)
            threading.Thread(target=lambda: execute_follow_up_command(translation_steps, MOTOR_SPEED)).start()

            # send_command(translation_steps, MOTOR_SPEED, 0, MOTOR_SPEED, -translation_steps, MOTOR_SPEED)
            # sthreading.Thread(target=lambda: (time.sleep(wait_time(angle)), send_command(translation_steps, MOTOR_SPEED, 0, MOTOR_SPEED, -translation_steps, MOTOR_SPEED))).start()


    dial3 = Dial(master=robot_control_frame, color_gradient=("cyan", "pink"), text_color="white", text="Angle: ", unit_length=10, radius=60, start=-180, end=180)
    dial3.set(center_sliders[4].get())  # Assuming center_sliders[4] exists
    dial3.grid(row=3, column=0, padx=10, pady=20)

    dial4 = Dial(master=robot_control_frame, color_gradient=("cyan", "pink"), text_color="white", text="Speed: ", unit_length=10, radius=60, start=400, end=3000)
    dial4.set(1000)
    dial4.grid(row=3, column=1, padx=10, pady=20)

    Polar_button_container = ctk.CTkFrame(robot_control_frame, fg_color='transparent')
    Polar_button_container.grid(row=4, column=0, padx=10, pady=20)

    CW_button = ctk.CTkButton(Polar_button_container, text="CW", width=60,command=lambda: CW())
    CW_button.pack(side="left", fill="both", expand=True, padx=10)

    CCW_button = ctk.CTkButton(Polar_button_container, text="CCW", width=60,command=lambda: CCW())
    CCW_button.pack(side="left", fill="both", expand=True, padx=10)

    Polar_slider_container = ctk.CTkFrame(robot_control_frame, fg_color='transparent')
    Polar_slider_container.grid(row=4, column=1, padx=10, pady=20)

    text2 = ctk.CTkLabel(Polar_slider_container, text="Distance:", fg_color=None)
    text2.pack(side="left", fill="both", expand=True, padx=10)

    slider_distance = ctk.CTkSlider(Polar_slider_container, from_=0, to=50, height=20)
    slider_distance.pack(side="left", fill="x", expand=True, padx=10)



    def CW():
        value=sensitivity_entry.get()
        value=-int(value)
        MOTOR_SPEED=dial4.get()
        send_command(value,MOTOR_SPEED,value,MOTOR_SPEED,value,MOTOR_SPEED)

    def CCW():
        value=int(sensitivity_entry.get())
        MOTOR_SPEED=dial4.get()
        send_command(value,MOTOR_SPEED,value,MOTOR_SPEED,value,MOTOR_SPEED)
    
    def Right():
        value=int(sensitivity_entry.get())
        MOTOR_SPEED=dial4.get()/2
        send_command(value,MOTOR_SPEED,-value*2,MOTOR_SPEED*2,value,MOTOR_SPEED)

    def Left():
        value=int(sensitivity_entry.get())
        MOTOR_SPEED=dial4.get()/2
        send_command(-value,MOTOR_SPEED,value*2,MOTOR_SPEED*2,-value,MOTOR_SPEED)

    def Up():
        value=int(sensitivity_entry.get())
        MOTOR_SPEED=dial4.get()
        send_command(value,MOTOR_SPEED,0,MOTOR_SPEED,-value,MOTOR_SPEED)

    def Down():
        value=int(sensitivity_entry.get())
        MOTOR_SPEED=dial4.get()
        send_command(-value,MOTOR_SPEED,0,MOTOR_SPEED,value,MOTOR_SPEED)
    

    # Cartesian coordinates section
    text3 = ctk.CTkLabel(robot_control_frame, text="Cartesian coordinates", font=("Arial", 20), fg_color=None)
    text3.grid(row=5, column=0, padx=10, pady=20)

    Cartesian_button = ctk.CTkButton(robot_control_frame, text="Send Cartesian Command", fg_color="#b165ff", width=280, command=lambda: send_Cartesian_command())
    Cartesian_button.grid(row=5, column=1, padx=10, pady=20)

    def send_Cartesian_command():
        global dial3, dial4, slider_distance, X_direction, Y_direction, text2, text4, text5, center_sliders
        if ball_measurements is not None:
            for center, radius, distance, angle, X_coordinate, Y_coordinate in ball_measurements:
                print(f"X_coordinate = {X_coordinate:.1f} cm, Y_coordinate = {Y_coordinate:.1f} cm")
                # send_strike_command(1500)
                steps , speeds = getCartesianStepsAndSpeed(X_coordinate/100,Y_coordinate/100)
                # send_command(steps[0], speeds[0], steps[2], speeds[2], steps[1], speeds[1])


    X_button_container = ctk.CTkFrame(robot_control_frame, fg_color='transparent')
    X_button_container.grid(row=6, column=0, padx=10, pady=20)

    Left_button = ctk.CTkButton(X_button_container, text="Left", width=60,command=lambda: Left())
    Left_button.pack(side="left", fill="both", expand=True, padx=10)

    Right_button = ctk.CTkButton(X_button_container, text="Right", width=60,command=lambda: Right())
    Right_button.pack(side="left", fill="both", expand=True, padx=10)

    X_slider_container = ctk.CTkFrame(robot_control_frame, fg_color='transparent')
    X_slider_container.grid(row=6, column=1, padx=10, pady=20)

    text4 = ctk.CTkLabel(X_slider_container, text="e", fg_color=None)
    text4.pack(side="left", fill="both", expand=True, padx=10)

    X_direction = ctk.CTkSlider(X_slider_container, from_=-50, to=50, height=20)
    X_direction.pack(side="left", fill="x", expand=True, padx=10)

    Y_button_container = ctk.CTkFrame(robot_control_frame, fg_color='transparent')
    Y_button_container.grid(row=7, column=0, padx=10, pady=20)

    Up_button = ctk.CTkButton(Y_button_container, text="Up", width=60,command=lambda: Up())
    Up_button.pack(side="left", fill="both", expand=True, padx=10)

    Down_button = ctk.CTkButton(Y_button_container, text="Down", width=60,command=lambda: Down())
    Down_button.pack(side="left", fill="both", expand=True, padx=10)

    Y_slider_container = ctk.CTkFrame(robot_control_frame, fg_color='transparent')
    Y_slider_container.grid(row=7, column=1, padx=10, pady=20)

    text5 = ctk.CTkLabel(Y_slider_container, text="e", fg_color=None)
    text5.pack(side="left", fill="both", expand=True, padx=10)

    Y_direction = ctk.CTkSlider(Y_slider_container, from_=-50, to=50, height=20)
    Y_direction.pack(side="left", fill="x", expand=True, padx=10)

    # Final control - Charge and fire button
    fire_button = ctk.CTkButton(robot_control_frame, text="Charge and Fire", fg_color="#b165ff", width=350, command=lambda: Fire(duration=charging_time_entry.get()))
    fire_button.grid(row=8, column=0, columnspan=2, padx=40, pady=40)

    def Fire(duration):
        if duration:
            if int(duration)>1200:
                duration=1200
                send_strike_command(int(duration))
        else:
            send_strike_command(1000)


def update_dial():
    global ball_measurements, switch_var, dial3, slider_distance, text2, text4, text5, X_direction, Y_direction
    if switch_var.get() == "off":
        if ball_measurements is not None:
            for center, radius, distance, angle, X_coordinate, Y_coordinate in ball_measurements:
                dial3.set(angle)
                slider_distance.set(distance)
                text2.configure(text=f"Distance: {distance:.1f} cm")
                text4.configure(text=f"X-direction: {X_coordinate:.1f}")
                text5.configure(text=f"Y-direction: {Y_coordinate:.1f}")
                X_direction.set(X_coordinate)
                Y_direction.set(Y_coordinate)
    robot_control_frame.after(100, update_dial)


def save_thresholds():
    with open(f"{directory}/thresholds.txt", "w") as file:
        for threshold in thresholds:
            # Convert numpy arrays to lists for easier file writing
            threshold_list = threshold.tolist()
            file.write(",".join(map(str, threshold_list)) + "\n")


def on_closing():
    save_thresholds()
    root.destroy()

def wait_time(angle):
    if abs(angle) == 0:
        return 0
    if abs(angle)>0 and abs(angle)<=30:
        return 1
    if abs(angle)>30 and abs(angle)<=90:
        return 2
    if abs(angle)>90 and abs(angle)<=130:
        return 3
    if abs(angle)>130 and abs(angle)<=180:
        return 4

def execute_follow_up_command(translation_steps, motor_speed):
    # Wait for the first command to complete
    check_movement_complete()
    # Execute the second command
    send_command(translation_steps, motor_speed, 0, motor_speed, -translation_steps, motor_speed)

def main():
    initialize_gui()
    create_main_frames()
    create_sliders_and_controls()
    toggle_sliders(ball_sliders_frame)
    setup_robot_control()
    update_camera_feed()
    update_dial()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()