import cv2
import numpy as np
import threading
import time
# Import other necessary modules
from image_processing import detect_backgroud_boudary, detect_pink_paper, detect_colored_spots, detect_colored_spots2, detect_balls, detect_pockets
from utility_functions import create_click_event, detect_and_draw_Y_axis, calculate_center, calculate_ball_measurements, annotate_ball_measurements, draw_dotted_line,line_circle_intersection
from robot_control import send_command, calculate_rotation_steps, calculate_translation_steps, send_strike_command, getCartesianStepsAndSpeed
from constants import MOTOR_SPEED, LOWER_CENTER, UPPER_CENTER, LOWER_Y_AXIS, UPPER_Y_AXIS, LOWER_BALL, UPPER_BALL, LOWER_TABLE, UPPER_TABLE,LOWER_ROBOT,UPPER_ROBOT , POOL_BALL_DIAMETER, RADIUS_ROBOT

# Initialize camera
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)  
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)



def get_processed_frame(cap, thresholds):
    # print(thresholds)
    global ball_measurements,Robot_Target
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
            balls = detect_balls(frame, table_contour, (LOWER_BALL, UPPER_BALL))
            # ball_measurements = calculate_ball_measurements(frame, balls, origin, y_direction)
            # annotate_ball_measurements(frame, ball_measurements, origin)


            LOWER_CUE=thresholds[10]
            UPPER_CUE=thresholds[11]
            cue = detect_balls(frame, table_contour, (LOWER_CUE, UPPER_CUE))
            if cue:
                cue_center = cue[0][0]
                cue_radius = cue[0][1]
                cue_robot_raduis= RADIUS_ROBOT*100*2/(POOL_BALL_DIAMETER)*cue_radius
            

                LOWER_POCKET=thresholds[12]
                UPPER_POCKET=thresholds[13]

                pockets = detect_pockets(frame, (LOWER_POCKET, UPPER_POCKET))
                if pockets:
                    for pocket in pockets:
                        pocket_center = calculate_center(pocket)
                        cv2.circle(frame, pocket_center, 5, (0, 0, 255), -1)
                        draw_dotted_line(frame, cue_center, pocket_center,(155, 30, 100), 2, 30)

                        if pocket_center is not None:    
                            target_pt = line_circle_intersection(frame, pocket_center, cue_center, cue_center, cue_robot_raduis+50)  #the repeat is intentional to make the cue ball the center of the circle
                            target_pt_data=[]
                            target_pt_data.append( ( (target_pt[0][0],target_pt[0][1]), cue[0][1] ) )
                            Robot_Target = calculate_ball_measurements(frame, target_pt_data, origin, y_direction)
                            annotate_ball_measurements(frame, Robot_Target, origin)


            return frame, None  #remeber to change this to ball_measurements !!!!! or target measurements TODO  
    # cv2.imshow('hold', frame)

    return frame,None
    


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