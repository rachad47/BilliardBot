import cv2
import numpy as np
import threading

# Import other necessary modules
from image_processing import detect_backgroud_boudary, detect_Robot, detect_colored_spots, detect_balls, detect_pockets
from utility_functions import create_click_event, detect_and_draw_Y_axis, calculate_center, calculate_ball_measurements, annotate_ball_measurements, draw_dotted_line,line_circle_intersection, bouncing
from robot_control import send_command, calculate_rotation_steps, calculate_translation_steps, send_strike_command, getCartesianStepsAndSpeed, check_movement_complete, forward_backward_movement, rotation_sequence, left_right_movement
from constants import MOTOR_SPEED, POOL_BALL_DIAMETER, RADIUS_ROBOT, PIXELS_BALL_RADIUS

# Initialize camera
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)  
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


# TODO: Self_adjusting_polar and execute_combined_command are still under testing & improvement process / should go to Robot_control.py
def Self_adjusting_polar(times):
    for i in range(times):
        threading.Thread(target=lambda: execute_combined_command(MOTOR_SPEED)).start()


def execute_combined_command(motor_speed):
    semaphore.acquire()

    Robot_Target = calculate_ball_measurements(frame, target_pt_data, origin, y_direction)


    for center, radius, distance, angle, X_coordinate, Y_coordinate in Robot_Target:
        print(f"Rotation Distance = {distance:.1f} cm, Angle = {angle:.1f} degrees")

    if angle > 90:
        angle = angle - 180
        distance = -distance
    
    if angle < -90:
        angle = angle + 180
        distance = -distance

    rotation_steps = -calculate_rotation_steps(angle)
    translation_steps = calculate_translation_steps(-distance/100)

    print(f"Rotation Steps: {rotation_steps}")
    if rotation_steps != 0:
        print(" ")

    # Execute rotation
        send_command(rotation_steps, motor_speed, rotation_steps, motor_speed, rotation_steps, motor_speed)
        check_movement_complete()  # Ensure rotation is complete before starting translation

    print(f"Translation Steps: {translation_steps}")
    if translation_steps != 0:

        print(" ")
    # Execute translation
        send_command(-translation_steps, motor_speed, 0, motor_speed, +translation_steps, motor_speed)
        check_movement_complete()
    
    semaphore.release()

semaphore = threading.Semaphore(1)


""" 
    Process the frame to detect the table boundaries and the robot

    parameters:
    cap: the camera object
    thresholds: the threshold values for the image processing functions

    return:
    call to the Robot_position_orientation function to detect the robot position and orientation
"""

def get_processed_frame(cap, thresholds):
    global table_contour, frame

    ret, frameOrigin = cap.read()
    if not ret:
        print("No frame received")
        return None, None

    frame = cv2.GaussianBlur(frameOrigin, (5, 5), 0)

    # Detect the table boundaries
    table_contour = detect_backgroud_boudary(frame, (thresholds[6], thresholds[7]))
    if table_contour is None:
        return frame, None

    table_mask = np.zeros_like(frame[:, :, 0])
    cv2.drawContours(table_mask, [table_contour], -1, 255, -1)

    # Detect the robot
    Robot = detect_Robot(frame, table_mask, (thresholds[8], thresholds[9]))
    if Robot is None:
        return frame, None

    return Robot_position_orientation(frame, Robot, thresholds)

"""
    Detect the robot position and orientation

    parameters:
    frame: the input frame
    Robot: the robot contour
    thresholds: the threshold values for the image processing functions

    return:
    call to the process_game_elements function to detect the cue, balls, pockets and calculate the target point
"""

def Robot_position_orientation(frame, Robot, thresholds):
    global origin, y_direction

    # Create a mask from the robot contour
    Robot_mask = np.zeros_like(frame[:, :, 0])
    cv2.drawContours(Robot_mask, [Robot], 0, 255, -1)

    # Detect the center spot of the robot
    center_spot = detect_colored_spots(frame, (thresholds[4], thresholds[5]), Robot_mask)
    if not center_spot:
        return frame, None

    origin = calculate_center(center_spot[0])
    cv2.circle(frame, origin, 5, (0, 0, 255), -1)

    # Detect the Y axis of the robot
    y_direction = detect_and_draw_Y_axis(frame, (thresholds[2], thresholds[3]), Robot_mask, origin)
    if y_direction is None:
        return frame, None
    
    return process_game_elements(frame, origin, y_direction, thresholds)


"""
    Process the game elements to detect the cue, balls, pockets and calculate the bouncing and target points
    parameters:
    frame: the input frame
    origin: the origin point of the robot
    y_direction: the y direction of the robot
    thresholds: the threshold values for the image processing functions
    
    return:
    the frame with the annotated target point
    Robot_Target: the target point data as defined in the calculate_ball_measurements() function in utility_functions.py
"""
def process_game_elements(frame, origin, y_direction, thresholds):
    global Robot_Target, target_pt_data, collision_pt,cue_center, bouncing_pt

    cue = detect_balls(frame,table_contour, (thresholds[10], thresholds[11]))
    ball = detect_balls(frame, table_contour, (thresholds[0], thresholds[1]))
    # if cue or ball is not detected, return the frame without any target point
    if not cue or not ball:
        return frame, None
    
    # get the fisrt detected cue and ball
    cue_center = cue[0][0]
    cue_radius = cue[0][1]

    ball_center = ball[0][0]
    ball_radius = ball[0][1]

    # the raduis of the robot in pixels relative to the cue ball
    cue_robot_radius = RADIUS_ROBOT * 100 * 2 / POOL_BALL_DIAMETER * cue_radius

    pockets = detect_pockets(frame, (thresholds[12], thresholds[13]))
    # if no pockets are detected, return the frame without any target point
    if not pockets:
        return frame, None
    
    # hard coded values for the boundary points of the table (for now)
    boundary_pt1 = (0, 66)
    boundary_pt2 = (1280, 66)

    Robot_Target = None
    # for each detected pocket, calculate the bouncing and target points
    for pocket in pockets:
        pocket_center = pocket[0]

        if pocket_center is not None:
            # collision_pt: the point where the cue ball will hit the target ball
            # bouncing_pt: the point where the cue ball will bounce after hitting the target ball
            # target_pt: the point where the robot should go to hit the target ball
            collision_pt= line_circle_intersection(frame, pocket_center, ball_center, ball_center, ball_radius*1.5) # *1.5 because we are looking for the center of the ball
            bouncing_pt = bouncing(cue_center, collision_pt[0], boundary_pt1, boundary_pt2)
            target_pt = line_circle_intersection(frame, bouncing_pt, cue_center, cue_center, cue_robot_radius+50) # +50 just for the simulation, it should be disabled in the real robot
            
            # added "PIXELS_BALL_RADIUS" to the target point data to represent the radius of the target ball in order scale the robot movement CM<->Pixels
            target_pt_data = [((target_pt[0][0], target_pt[0][1]), PIXELS_BALL_RADIUS)]

            # calulate the target point data which both the polar and cartesian data
            Robot_Target = calculate_ball_measurements(frame, target_pt_data, origin, y_direction)

            # annotations
            draw_dotted_line(frame, cue_center, bouncing_pt, (200, 30, 30), 2, 25)
            cv2.circle(frame, pocket_center, 5, (0, 0, 255), -1)
            cv2.circle(frame, bouncing_pt, 7, (100, 100, 200), -1)
            draw_dotted_line(frame, ball_center, pocket_center, (30, 200, 30), 2, 30)
            draw_dotted_line(frame, collision_pt[0], bouncing_pt, (30, 30, 200), 2, 30)
            annotate_ball_measurements(frame, Robot_Target, origin)
    
    return frame, Robot_Target
    


if __name__ == "__main__":
    # Load the threshold values from the thresholds.txt file
    try:
        with open("Software/thresholds.txt", "r") as file:
            loaded_thresholds = [np.array(list(map(int, line.strip().split(",")))) for line in file]
            thresholds = loaded_thresholds
    except FileNotFoundError:
        print("Thresholds file not found")
        pass  

    while True:
        # trigger the main program
        frame,_ = get_processed_frame(cap=cap, thresholds=thresholds)
        if frame is None:
            print("No frame received")
            break

        cv2.imshow('Frame', frame)

        # Create a mouse click event to get the coordinates and HSV values of clicked points
        cv2.setMouseCallback('Frame', create_click_event(frame))

        key = cv2.waitKey(1)

    # Quit the program
        if key & 0xFF == ord('q'):
            print("Quitting")
            break

    # send a self-adjusting polar command with a specified number of iterations
        elif key & 0xFF == ord('p'):
            Self_adjusting_polar(3)
                
    # send a CCW/CW rotation command with a specified angle to allign the robot with the bouncing point
        elif key & 0xFF == ord('r'):
            if 'collision_pt' in globals() and 'cue_center' in globals():
                # added "PIXELS_BALL_RADIUS" to the bouncing point data to represent the radius of the target ball in order to scale the robot movement CM<->Pixels
                bouncing_data=[((bouncing_pt[0], bouncing_pt[1]), PIXELS_BALL_RADIUS)] 
                data = calculate_ball_measurements(frame, bouncing_data, origin, y_direction)
                for center, radius, distance, angle, X_coordinate, Y_coordinate in data:
                    print(f"angle = {angle:.1f} degrees")
                rotation_sequence(angle) 
        
    # send a forward/backward movement command with a specified number of steps to reach the ball
        elif key & 0xFF == ord('f'):
            forward_backward_movement(900)

    # send a strike command to trigger the firing sequence of the selenoid with a specified charge duration in ms
        elif key & 0xFF == ord('s'):
            send_strike_command(900)

    # send a left/right movement command with a specified number of steps to reach the ball
        elif key & 0xFF == ord('l'):
            left_right_movement(-40)


    # Save the current measuraments to print later 
        elif key & 0xFF == ord('h'):
            cv2.imshow('hold', frame)
            hold_measurement = Robot_Target
            print("Measurements saved")

    #Cartesian coordinates
        elif key & 0xFF == ord('c'):
            if 'hold_measurement' in locals():
                # here Im using the hold_measurement to get the last measurements SAVED just to test my functions
                for center, radius, distance, angle, X_coordinate, Y_coordinate in hold_measurement:
                    print(f"X_coordinate = {X_coordinate:.1f} cm, Y_coordinate = {Y_coordinate:.1f} cm")                
                    steps, speeds = getCartesianStepsAndSpeed(X_coordinate, Y_coordinate)
                    send_command(steps[0], speeds[0], steps[2], speeds[2], steps[1], speeds[1])


cap.release()
cv2.destroyAllWindows()