import cv2
import numpy as np
import threading
import time
# Import other necessary modules
from image_processing import detect_backgroud_boudary, detect_pink_paper, detect_colored_spots, detect_colored_spots2, detect_balls
from utility_functions import create_click_event, detect_and_draw_Y_axis, calculate_center, calculate_ball_measurements, annotate_ball_measurements, annotate_ball_pair_line, annotate_ball_cue_pair_line
from robot_control import send_command, calculate_rotation_steps, calculate_translation_steps, send_strike_command, getCartesianStepsAndSpeed
from constants import MOTOR_SPEED, LOWER_CENTER, UPPER_CENTER, LOWER_Y_AXIS, UPPER_Y_AXIS, LOWER_BALL, UPPER_BALL, LOWER_TABLE, UPPER_TABLE,LOWER_ROBOT,UPPER_ROBOT , POOL_BALL_DIAMETER

# Initialize camera
# cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap = cv2.VideoCapture(4)  
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)


def get_processed_frame(cap, thresholds):
    # print(thresholds)
    global ball_measurements
    ret, frameOrigin = cap.read()
    if not ret:
        print("No frame received")
        return None
    
    frame = cv2.GaussianBlur(frameOrigin, (5, 5), 0)

    LOWER_TABLE=thresholds[6]
    UPPER_TABLE=thresholds[7]
    table_contour = detect_backgroud_boudary(frame, (LOWER_TABLE, UPPER_TABLE))
    if table_contour is not None:
        table_mask = np.zeros_like(frame[:, :, 0])
        cv2.drawContours(table_mask, [table_contour], -1, 255, -1)

        LOWER_ROBOT=thresholds[8]
        UPPER_ROBOT=thresholds[9]
        pink_paper_box = detect_pink_paper(frame, table_mask, (LOWER_ROBOT, UPPER_ROBOT))

        origin = None

        if pink_paper_box is not None:
            pink_paper_mask = np.zeros_like(frame[:, :, 0])
            cv2.drawContours(pink_paper_mask, [pink_paper_box], 0, 255, -1)

            LOWER_CENTER=thresholds[4]
            UPPER_CENTER=thresholds[5]
            center_spot = detect_colored_spots(frame, (LOWER_CENTER, UPPER_CENTER), pink_paper_mask)
            if center_spot:
                origin = calculate_center(center_spot[0])
                cv2.circle(frame, origin, 5, (0, 0, 255), -1)

            LOWER_Y_AXIS=thresholds[2]
            UPPER_Y_AXIS=thresholds[3]
            y_direction = detect_and_draw_Y_axis(frame, (LOWER_Y_AXIS, UPPER_Y_AXIS), pink_paper_mask, origin)

        if origin is not None and y_direction is not None:
           
            LOWER_BALL=thresholds[0]
            UPPER_BALL=thresholds[1]
            LOWER_CUE_BALL = thresholds[10]
            UPPER_CUE_BALL = thresholds[11]

            balls = detect_balls(frame, table_contour, (LOWER_BALL, UPPER_BALL))
            cue_balls = detect_balls(frame, table_contour, (LOWER_CUE_BALL, UPPER_CUE_BALL))

            ball_measurements = calculate_ball_measurements(frame, balls, origin, y_direction)
            cue_ball_measurements = calculate_ball_measurements(frame, cue_balls, origin, y_direction)

            print(f"balls: {balls} cue_balls: {cue_balls}")
            
            # annotate_ball_measurements(frame, ball_measurements, origin)
            annotate_ball_cue_pair_line(frame, ball_measurements, cue_ball_measurements)
            return frame,ball_measurements

    

    # cv2.imshow('hold', frame)

    return frame,None
    


if __name__ == "__main__":

    while True:

        frame,_ = get_processed_frame(cap=cap, thresholds=[LOWER_BALL, UPPER_BALL, LOWER_Y_AXIS, UPPER_Y_AXIS, LOWER_CENTER, UPPER_CENTER, LOWER_TABLE, UPPER_TABLE,LOWER_ROBOT,UPPER_ROBOT])
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
            hold_measurement = ball_measurements
            
        #Polar coordinates
        elif key & 0xFF == ord('p'):
            if 'hold_measurement' in locals():
                for center, radius, distance, angle, X_coordinate, Y_coordinate in hold_measurement:
                    print(f"Distance = {distance:.1f} cm, Angle = {angle:.1f} degrees")

                rotation_steps = -calculate_rotation_steps(angle)
                translation_steps = calculate_translation_steps(distance/100)-100

                print(f"Rotation Steps: {rotation_steps}, Translation Steps: {translation_steps}")

                send_command(rotation_steps, MOTOR_SPEED, rotation_steps, MOTOR_SPEED, rotation_steps, MOTOR_SPEED)
                threading.Thread(target=lambda: (time.sleep(2), send_command(translation_steps, MOTOR_SPEED, 0, MOTOR_SPEED, -translation_steps, MOTOR_SPEED))).start()

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