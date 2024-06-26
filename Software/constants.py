import numpy as np

esp32_ip = "192.168.137.179"
MOTOR_SPEED = 2500
RADIUS_ROBOT = 15.242 / 100
WHEEL_RADIUS = 0.03
STEPS_PER_ROTATION = 1600
DISTANCE_PER_STEP = 0.214
POOL_BALL_DIAMETER=5.7
PIXELS_BALL_RADIUS=15


# default values for the color ranges if the Thresholds.txt file is not found
ball_defaults = [96, 25, 0, 149, 255, 95]
y_axis_defaults = [1, 150, 200, 12, 200, 255]
center_defaults = [0, 0, 20, 179, 110, 220]
table_defaults = [0, 0, 100, 179, 40, 255]
robot_defaults = [0, 15, 168, 15, 70, 255]
cue_defaults = [0, 0, 0, 179, 255, 255]
pocket_defaults = [1, 2, 3, 100, 44, 42]


LOWER_CENTER = np.array([center_defaults[0], center_defaults[1], center_defaults[2]])
UPPER_CENTER = np.array([center_defaults[3], center_defaults[4], center_defaults[5]])

LOWER_Y_AXIS = np.array([y_axis_defaults[0], y_axis_defaults[1], y_axis_defaults[2]])
UPPER_Y_AXIS = np.array([y_axis_defaults[3], y_axis_defaults[4], y_axis_defaults[5]])

LOWER_BALL = np.array([ball_defaults[0], ball_defaults[1], ball_defaults[2]])
UPPER_BALL = np.array([ball_defaults[3], ball_defaults[4], ball_defaults[5]])

LOWER_TABLE = np.array([table_defaults[0], table_defaults[1], table_defaults[2]])
UPPER_TABLE = np.array([table_defaults[3], table_defaults[4], table_defaults[5]])

LOWER_ROBOT = np.array([robot_defaults[0], robot_defaults[1], robot_defaults[2]])
UPPER_ROBOT = np.array([robot_defaults[3], robot_defaults[4], robot_defaults[5]])

LOWER_CUE = np.array([cue_defaults[0], cue_defaults[1], cue_defaults[2]])
UPPER_CUE = np.array([cue_defaults[3], cue_defaults[4], cue_defaults[5]])

LOWER_POCKET = np.array([pocket_defaults[0], pocket_defaults[1], pocket_defaults[2]])
UPPER_POCKET = np.array([pocket_defaults[3], pocket_defaults[4], pocket_defaults[5]])


