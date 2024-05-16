import requests
import time
import numpy as np
import math
import threading
import numpy.linalg as la
from constants import esp32_ip, MOTOR_SPEED, RADIUS_ROBOT, WHEEL_RADIUS, STEPS_PER_ROTATION, DISTANCE_PER_STEP

"""
    Send a command to the ESP32 to control motor movements.
"""
def send_command(stepsX, speedX, stepsY, speedY, stepsZ, speedZ):
    
    url = f"http://{esp32_ip}/control"
    params = {
        'stepsX': stepsX,
        'speedX': speedX,
        'stepsY': stepsY,
        'speedY': speedY,
        'stepsZ': stepsZ,
        'speedZ': speedZ
    }
    try:
        response = requests.get(url, params=params)
        print(response.text)
    except requests.RequestException as e:
        print(f"Error sending request: {e}")


"""
 check_movement_complete() returns True if the ESP32 has finished moving the motors.
"""
def check_movement_complete():
    while True:
        response = requests.get(f"http://{esp32_ip}/status")
        if response.text == "Movement complete":
            break
        time.sleep(0.5)  # Poll every half second



"""
    Send a command to trigger the firing sequece of the selenoid.
"""

def send_strike_command(chargeDuration):
    """
    Starts a thread to send the strike command to avoid blocking the main UI.
    """
    thread = threading.Thread(target=strike_command_thread, args=(chargeDuration,))
    thread.start()

def strike_command_thread(chargeDuration):
    """
    Sends the actual strike command to the ESP32.
    """
    url = f"http://{esp32_ip}/strike"
    params = {'chargeDuration': chargeDuration}
    try:
        response = requests.get(url, params=params)
        print(response.text)
    except Exception as e:
        print(f"Error sending strike command: {e}")






# POLAR MOTION FUNCTIONS
    
"""
    Calculate the number of steps for rotation based on the given angle.
    parameter angle: The angle in degrees to rotate the robot.
    return: The number of steps to rotate the robot.
    """
def calculate_rotation_steps(angle):
    
    path_length = angle / 360 * 2 * math.pi * RADIUS_ROBOT
    rot_rot_num = path_length / (math.pi * 2 * WHEEL_RADIUS)
    return int(rot_rot_num * STEPS_PER_ROTATION)


"""
    Calculate the number of steps for translation based on the given distance.
    parameter distance: The distance in meters to translate the robot.
    return: The number of steps to translate the robot.
"""
def calculate_translation_steps(distance):
    return int(-distance / DISTANCE_PER_STEP * STEPS_PER_ROTATION)  


# CARTESIAN MOTION FUNCTIONS

'''
Name: convertToESPSpeeds

Description: given a list of steps for each motor, returns a list of speeds for the motors (to feed into accel.h)
@param absoluteValueSteps: the number of steps each motor is going to move (in the order of motorA, motorB, and motorC)
@param minimumSpeed: the minimum speed of the motors. Must be greater than 0. Default: 400. See accel.h documentation before changing
@return a list of motor speeds to go to accel.h. In the order of motorA, motorB, motorC

This code takes a list of steps and checks to see which wheel has to move the least amount of steps. It then assigns that wheel the designated minimum speed (by default it is 400). The speed
correlates to the speeds used by accel.h. It then assigns the rest of the wheels speeds scaling with the ratio of steps needed by that wheel vs smallest number of steps needed by any wheel. 
This should ensure that all wheels stop at the same speed. It then returns an array of speeds for the wheels (A, B, C).
'''

def convertToESPSpeeds(absoluteValueSteps, minimumSpeed = 400):
    speeds = []
    noZeros = [i for i in absoluteValueSteps if i != 0]
    smallestValue = min(noZeros) 
    #Find the smallest number of steps we need to take (not including 0's), will be absolute value

    if (smallestValue == absoluteValueSteps[0]):
        #print("lowest is X")
        speeds.append(minimumSpeed)
        speeds.append(minimumSpeed * absoluteValueSteps[1]/absoluteValueSteps[0])
        speeds.append(minimumSpeed * absoluteValueSteps [2]/absoluteValueSteps[0])
    elif (smallestValue == absoluteValueSteps[1]):
        #print("lowest is Y")
        speeds.append(minimumSpeed * absoluteValueSteps[0]/absoluteValueSteps[1])
        speeds.append(minimumSpeed)
        speeds.append(minimumSpeed * absoluteValueSteps [2]/absoluteValueSteps[1])
    elif (smallestValue == absoluteValueSteps[2]):
        #print("lowest is Z")
        speeds.append(minimumSpeed * absoluteValueSteps[0]/absoluteValueSteps[2])
        speeds.append(minimumSpeed * absoluteValueSteps[1]/absoluteValueSteps[2])
        speeds.append(minimumSpeed)
    return speeds

'''
Name: getStepsAndSpeed

Description: given an x and a y value, determines the steps and speed needed to move to new coordinate. x and y values are given in meters
@param desired_x is the x-coordinate you want to move in. Needs to be in meters
@param desired_x is the y-coordinate you want to move in. Needs to be in meters
@param angular_velocity WORK IN PROGRESS. Desired rotational speed. Default value of 0. Only for fun right now
@return two lists, the first containing steps and the second containing speeds. Outputs go to accel.h functions. Order is motorA, motorB, motorC

This code takes a desired x_coordinate and y_coordinate (which together form a 2D vector) and determines how many steps each wheel needs to move at and at what speeds 
each wheel needs to move at. The robot itself is at (0,0). It returns two arrays of length three containing speeds and steps for wheels (A, B, C).

'''

def getCartesianStepsAndSpeed(x_coordinate, y_coordinate, angular_velocity = 0):
    wheelRadius = 0.03 
    #given a desired x position, y position and angular velocity, solves for the speed of the wheels (in linear velocities, m/s)
    conversion_matrix = np.array([[0.5, 0.5, -1],
                                [math.sqrt(3)/2, -math.sqrt(3)/2, 0],
                                [1/wheelRadius, 1/wheelRadius, 1/wheelRadius]])
    b = np.array([x_coordinate, y_coordinate, angular_velocity])
    wheel_velocities = la.solve(conversion_matrix, b)

    #linear velocities converted to angular velocities (rad/s)
    angular_wheel_velocities = wheel_velocities/wheelRadius 
   
    #angular velocities converted to steps (distance). We are assuming 1/ms? 
    steps = np.round(angular_wheel_velocities * 1600 * 0.244).astype(int)

    slowest_speed = 400
    abssteps = abs(steps)
    speeds = convertToESPSpeeds(abssteps, slowest_speed)

    #If speeds are too high, we try to decrease all the speeds of the wheels so that our speeds are in some range btw 10 - 8000 but we want it as close to 400-8000 as possible
    while (max(speeds) > 2000 and slowest_speed > 20):
        slowest_speed -= 10
        speeds = convertToESPSpeeds(abssteps, slowest_speed)

    print("Steps: ", steps)
    print("Speeds: ", speeds)

    return(steps, speeds)


"""
    move the robot forward or backward by the given number of steps
"""
def forward_backward_movement(y):
    translation_steps = y  # Example value, adjust as needed
    send_command(-translation_steps, MOTOR_SPEED, 0, MOTOR_SPEED, +translation_steps, MOTOR_SPEED)


"move the robot left or right by the given number of steps"
def left_right_movement(x):
    value = x  # Example value, adjust as needed
    send_command(value,MOTOR_SPEED,-value*2,MOTOR_SPEED*2,value,MOTOR_SPEED)

"""
    Rotate the robot by the given angle in degrees.
    parameter angle: The angle in degrees to rotate the robot.
"""
def rotation_sequence(o):
    rotation_steps = -calculate_rotation_steps(o)
    send_command(rotation_steps, MOTOR_SPEED, rotation_steps, MOTOR_SPEED, rotation_steps, MOTOR_SPEED)

