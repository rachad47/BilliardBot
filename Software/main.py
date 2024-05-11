import cv2
import numpy as np
import threading
import time
# Import other necessary modules
from image_processing import detect_backgroud_boudary, detect_pink_paper, detect_colored_spots, detect_colored_spots2, detect_balls, detect_pockets
from utility_functions import create_click_event, detect_and_draw_Y_axis, calculate_center, calculate_ball_measurements, annotate_ball_measurements, draw_dotted_line,line_circle_intersection,calculate_perpendicular_distance
from robot_control import send_command, calculate_rotation_steps, calculate_translation_steps, send_strike_command, getCartesianStepsAndSpeed, check_movement_complete
from constants import MOTOR_SPEED, LOWER_CENTER, UPPER_CENTER, LOWER_Y_AXIS, UPPER_Y_AXIS, LOWER_BALL, UPPER_BALL, LOWER_TABLE, UPPER_TABLE,LOWER_ROBOT,UPPER_ROBOT , POOL_BALL_DIAMETER, RADIUS_ROBOT

# Initialize camera
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)  
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# frame = None

def polarr(times):
    for i in range(times):
        threading.Thread(target=lambda: execute_combined_command(MOTOR_SPEED)).start()
        

def execute_combined_command(motor_speed):
    semaphore.acquire()

    # target_pt_data = [((600, 298), 15)]
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


def get_processed_frame(cap, thresholds):
    global table_contour, frame
    ret, frameOrigin = cap.read()
    if not ret:
        print("No frame received")
        return None, None

    frame = cv2.GaussianBlur(frameOrigin, (5, 5), 0)
    table_contour = detect_backgroud_boudary(frame, (thresholds[6], thresholds[7]))
    if table_contour is None:
        return frame, None

    table_mask = np.zeros_like(frame[:, :, 0])
    cv2.drawContours(table_mask, [table_contour], -1, 255, -1)

    pink_paper_box = detect_pink_paper(frame, table_mask, (thresholds[8], thresholds[9]))
    if pink_paper_box is None:
        return frame, None

    return process_pink_paper_box(frame, pink_paper_box, table_mask, thresholds)

def process_pink_paper_box(frame, pink_paper_box, mask, thresholds):
    global origin, y_direction
    pink_paper_mask = np.zeros_like(frame[:, :, 0])
    cv2.drawContours(pink_paper_mask, [pink_paper_box], 0, 255, -1)

    center_spot = detect_colored_spots(frame, (thresholds[4], thresholds[5]), pink_paper_mask)
    if not center_spot:
        return frame, None

    origin = calculate_center(center_spot[0])
    cv2.circle(frame, origin, 5, (0, 0, 255), -1)
    y_direction = detect_and_draw_Y_axis(frame, (thresholds[2], thresholds[3]), pink_paper_mask, origin)
    if y_direction is None:
        return frame, None
    return process_game_elements(frame, origin, y_direction, thresholds)

def process_game_elements(frame, origin, y_direction, thresholds):
    global Robot_Target, target_pt_data, collision_pt,cue_center
    cue = detect_balls(frame,table_contour, (thresholds[10], thresholds[11]))
    ball = detect_balls(frame, table_contour, (thresholds[0], thresholds[1]))
    if not cue or not ball:
        return frame, None
    cue_center = cue[0][0]
    cue_radius = cue[0][1]

    ball_center = ball[0][0]
    ball_radius = ball[0][1]
    cue_robot_radius = RADIUS_ROBOT * 100 * 2 / POOL_BALL_DIAMETER * cue_radius

    # pockets = detect_pockets(frame, (thresholds[12], thresholds[13]))
    pockets=[]
    pockets.append(((87, 57),15))
    if not pockets:
        return frame, None

    Robot_Target = None
    for pocket in pockets:
        pocket_center = pocket[0]
        if pocket_center is not None:
            cv2.circle(frame, pocket_center, 5, (0, 0, 255), -1)
            collision_pt= line_circle_intersection(frame, pocket_center, ball_center, ball_center, ball_radius*2)
            draw_dotted_line(frame, ball_center, pocket_center, (155, 30, 100), 2, 30)

            target_pt = line_circle_intersection(frame, collision_pt[0], cue_center, cue_center, cue_robot_radius + 50)
            draw_dotted_line(frame, cue_center, collision_pt[0], (0, 30, 0), 1, 25)
            target_pt_data = [((target_pt[0][0], target_pt[0][1]), 15)]
            # target_pt_data = [((600, 298), 15)]

            Robot_Target = calculate_ball_measurements(frame, target_pt_data, origin, y_direction)
            annotate_ball_measurements(frame, Robot_Target, origin)

            # target_pt = line_circle_intersection(frame, pocket_center, cue_center, cue_center, cue_robot_radius + 50)
            # target_pt_data = [((target_pt[0][0], target_pt[0][1]), cue[0][1])]
            # Robot_Target = calculate_ball_measurements(frame, target_pt_data, origin, y_direction)
            # annotate_ball_measurements(frame, Robot_Target, origin)

    # target_pt_data = [((600, 298), 15)]

    # Robot_Target = calculate_ball_measurements(frame, target_pt_data, origin, y_direction)
    # annotate_ball_measurements(frame, Robot_Target, origin)
    
    return frame, Robot_Target
    


if __name__ == "__main__":

    try:
        with open("Software/thresholds.txt", "r") as file:
            loaded_thresholds = [np.array(list(map(int, line.strip().split(",")))) for line in file]
            thresholds = loaded_thresholds
    except FileNotFoundError:
        print("Thresholds file not found")
        pass  

    while True:

        frame,_ = get_processed_frame(cap=cap, thresholds=thresholds)
        if frame is None:
            print("No frame received")
            break

        cv2.imshow('Frame', frame)
        cv2.setMouseCallback('Frame', create_click_event(frame))

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            print("Quitting")
            break
        
        elif key & 0xFF == ord('h'):
            # Save the current measuraments to print later 
            cv2.imshow('hold', frame)
            hold_measurement = Robot_Target
            
        #Polar coordinates
        elif key & 0xFF == ord('p'):
            # if 'hold_measurement' in locals():
            polarr(3)
                
                
        # angle btwn y axis and line joining center of the cue ball and the collision_pt 
        elif key & 0xFF == ord('r'):
            # if 'hold_measurement' in locals():  
                # print(collision_pt[0], "    ", origin)
            collision_pt_data = [((collision_pt[0][0], collision_pt[0][1]), 15)]
            calc = calculate_ball_measurements(frame, collision_pt_data, origin, y_direction)
            for center, radius, distance, angle, X_coordinate, Y_coordinate in calc:
                print(f"angle = {angle:.1f} degrees")

            rotation_steps = -calculate_rotation_steps(angle)
            send_command(rotation_steps, MOTOR_SPEED, rotation_steps, MOTOR_SPEED, rotation_steps, MOTOR_SPEED)

        
        elif key & 0xFF == ord('l'):
            value=-40
            send_command(-value,MOTOR_SPEED,value*2,MOTOR_SPEED*2,-value,MOTOR_SPEED)

        

        elif key & 0xFF == ord('f'):
            translation_steps= 950
            send_command(-translation_steps, MOTOR_SPEED, 0, MOTOR_SPEED, +translation_steps, MOTOR_SPEED)

        elif key & 0xFF == ord('s'):
            send_strike_command(500)


        elif key & 0xFF == ord('t'):
            print("Y-dir ",y_direction)
            print("cue_center ",cue_center)
            print("coollision_pt ",collision_pt[0])
            a= calculate_perpendicular_distance(y_direction,cue_center,collision_pt[0],origin)
            print("Perpendicular distance ",a)

        #Cartesian coordinates
        elif key & 0xFF == ord('c'):
            if 'hold_measurement' in locals():
                for center, radius, distance, angle, X_coordinate, Y_coordinate in hold_measurement:
                    print(f"X_coordinate = {X_coordinate:.1f} cm, Y_coordinate = {Y_coordinate:.1f} cm")
                    # send_strike_command(1500) 
                
                    steps, speeds = getCartesianStepsAndSpeed(X_coordinate, Y_coordinate)
                    # send_command(steps[0], speeds[0], steps[2], speeds[2], steps[1], speeds[1])


cap.release()
cv2.destroyAllWindows()