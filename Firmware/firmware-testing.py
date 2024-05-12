

import requests
import time
import math


MOTOR_SPEED = 1500
RADIUS_ROBOT = 15.242 / 100
WHEEL_RADIUS = 0.03
STEPS_PER_ROTATION = 1600
DISTANCE_PER_STEP = 0.214
POOL_BALL_DIAMETER=5.7

# Replace with your ESP32 IP address
esp_ip = "192.168.137.82"

def send_command(stepsX, speedX, stepsY, speedY, stepsZ, speedZ):
    

    url = f"http://{esp_ip}/control"
    params = {
        'stepsX': stepsX,
        'speedX': speedX,
        'stepsY': stepsY,
        'speedY': speedY,
        'stepsZ': stepsZ,
        'speedZ': speedZ
    }
    print(url, params)
    response = requests.get(url, params=params)
    print(response.text)


    
def check_movement_complete():
    response = requests.get(f"http://{esp_ip}/status")
    print(response.text)
    return response.text == "Movement complete"   
# C
    
def stop():
    url = f"http://{esp_ip}/stop"
    params = {}
    response = requests.get(url, params=params)
    print(response.text)



def send_strike_command(chargeDuration):
    url = f"http://{esp_ip}/strike"
    params = {'chargeDuration': chargeDuration}
    response = requests.get(url, params=params)
    print(response.text)
# Example usage to toggle the LED


# send_strike_command(800)

# v=1
# stepsX = 1600  *3*0.2 
# speedX = 500   *2*v

# # B
# stepsY = -1600  *6*0.2
# speedY = 500   *4*v

# # A
# stepsZ = 1600  *3*0.2
# speedZ = 500   *2*v


v=1
stepsX = 100  *3*0.2 
speedX = 500   *2*v

# B
stepsY = -100  *6*0.2
speedY = 500   *4*v

# A
stepsZ = 100  *3*0.2
speedZ = 500   *2*v


# send_command(stepsX, speedX, stepsY, speedY, stepsZ, speedZ)
# time.sleep(3)
send_strike_command(300)



def calculate_rotation_steps(angle):
    
    path_length = angle / 360 * 2 * math.pi * RADIUS_ROBOT
    rot_rot_num = path_length / (math.pi * 2 * WHEEL_RADIUS)
    return int(rot_rot_num * 1600)





