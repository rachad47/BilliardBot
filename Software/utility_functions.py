import cv2
import numpy as np
import math
from image_processing import detect_colored_spots2
from constants import POOL_BALL_DIAMETER


"""
    Detects the HSV values of the pixel at the position where the user clicks
    """
def create_click_event(frame):
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            pixel_value = hsv[y, x]
            print(f"Clicked at position: ({x}, {y}), Pixel value: {pixel_value}")
    return click_event


"""
    Calculates the center (centroid) of a given contour using image moments.

    Parameters:
    contour (np.array): The contour for which to find the center.

    Returns:
    tuple: The (x, y) coordinates of the center of the contour.
    """
def calculate_center(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
    return None


"""
    Detects the Y-axis spots in the frame within a specified color range and draws the axis.

    Parameters:
    frame (np.array): The input image frame.
    color_range (tuple): The lower and upper range for the Y-axis color.
    mask (np.array): The mask to constrain the detection area.
    origin (tuple): The (x, y) coordinates of the origin.

    Returns:
    tuple: The direction vector of the Y-axis, or None if not detected.
    """
def detect_and_draw_Y_axis(frame, color_range, mask, origin):
    Y_axis_spots = detect_colored_spots2(frame, color_range, mask)
    Y_axis_center = None
    y_direction = None

    for spot in Y_axis_spots:
        Y_axis_center = calculate_center(spot)
        if Y_axis_center and origin:
            y_direction = (Y_axis_center[0] - origin[0], Y_axis_center[1] - origin[1])
            break

    if origin is not None and Y_axis_center is not None:
        draw_axes(frame, origin, Y_axis_center)

    return y_direction


"""
    Draws the X and Y axes on the frame based on the specified origin and Y direction point.
    The X-axis is drawn perpendicular to the right of Y-axis.

    Parameters:
    frame (np.array): The image frame on which to draw the axes.
    origin (tuple): The (x, y) coordinates of the origin of the axes.
    y_point (tuple): The (x, y) coordinates of a point on the Y-axis.

    Returns:
    None
    """
def draw_axes(frame, origin, y_point):
    # Draw Y-axis as an arrow pointing to the yellow spot
    cv2.arrowedLine(frame, origin, y_point, (0, 255, 0), 2, tipLength=0.2)
    cv2.putText(frame, 'Y', (y_point[0] + 10, y_point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # Calculate the length of the Y-axis
    y_length = np.sqrt((y_point[0] - origin[0])**2 + (y_point[1] - origin[1])**2)

    # Calculate a perpendicular direction for the X-axis
    y_direction = (y_point[0] - origin[0], y_point[1] - origin[1])
    x_direction = (-y_direction[1], y_direction[0])

    # Normalize and scale the X direction to match the Y-axis length
    norm = np.sqrt(x_direction[0]**2 + x_direction[1]**2)
    x_direction = (int(x_direction[0]/norm * y_length), int(x_direction[1]/norm * y_length))

    # Define the end point for the X-axis
    x_point = (origin[0] + x_direction[0], origin[1] + x_direction[1])

    # Draw X-axis as an arrow of the same length as the Y-axis
    cv2.arrowedLine(frame, origin, x_point, (255, 0, 0), 2, tipLength=0.2)
    cv2.putText(frame, 'X', (x_point[0] - 10, x_point[1] +20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)



"""
    Calculates the measurements (polar and cartesian coordiantes) of detected balls from the origin.
    The origin and Y direction are used to establish a coordinate system for measurement.

    Parameters:
    frame (np.array): The image frame for reference.
    balls (list): A list of tuples containing the center and radius of each detected ball.
    origin (tuple): The (x, y) coordinates of the origin of the coordinate system.
    y_direction (tuple): The Y direction vector for establishing the coordinate system.
    ball_diameter_cm (float): The diameter of the balls in centimeters.

    Returns:
    list: A list of tuples containing calculated measurements for each ball (polar and cartesian coordiantes).
    """
def calculate_ball_measurements(frame, balls, origin, y_direction, ball_diameter_cm=POOL_BALL_DIAMETER):
    ball_data = []
    y_length = math.sqrt(y_direction[0]**2 + y_direction[1]**2)

    for center, radius in balls:
        # Draw circle around the ball
        cv2.circle(frame, center, radius, (255, 255, 0), 2)
        cv2.line(frame, origin, center, (255, 100, 255), 2)  # Line from origin to ball

        # Calculate distance
        ball_vector = (center[0] - origin[0], center[1] - origin[1])
        distance_pixels = math.sqrt(ball_vector[0]**2 + ball_vector[1]**2)
        pixel_to_cm_ratio = ball_diameter_cm / (2 * radius)
        distance_cm = distance_pixels * pixel_to_cm_ratio

        # Calculate angle
        dot_product = ball_vector[0]*y_direction[0] + ball_vector[1]*y_direction[1]
        if (distance_pixels * y_length) != 0:
            angle = math.acos(dot_product / (distance_pixels * y_length))
            angle_degrees = math.degrees(angle)

        # Determine the sign of the angle using the cross product
        cross_product_z = y_direction[0] * ball_vector[1] - y_direction[1] * ball_vector[0]
        angle_degrees = -angle_degrees if cross_product_z < 0 else angle_degrees

        #Cartesian coordinates
        dirX= distance_pixels * pixel_to_cm_ratio * math.sin(angle)
        dirY= distance_pixels * pixel_to_cm_ratio * math.cos(angle)
        dirX = -dirX if angle_degrees < 0 else dirX

        # Store the calculated data
        ball_data.append((center, radius, distance_cm, angle_degrees , dirX, dirY))

    return ball_data


""" Annotates the frame with measurements of detected balls.

    Parameters:
    frame (np.array): The image frame to draw annotations on.
    ball_measurements (list): A list of tuples containing ball measurements (center, radius, distance_cm, angle_degrees, X_coordinate, Y_coordinate).
    origin (tuple): The (x, y) coordinates of the origin point for measurements.

    Returns:
    None
    """
def annotate_ball_measurements(frame, ball_measurements, origin):
  
    for center, radius, distance_cm, angle_degrees, X_coordinate, Y_coordinate in ball_measurements:
        midpoint = ((origin[0] + center[0]) // 2, (origin[1] + center[1]) // 2)
        cv2.putText(frame, f"{distance_cm:.1f} cm", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 100), 2)
        cv2.putText(frame, f"{angle_degrees:.1f} degrees", (center[0] - 40, center[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 255), 2)
        
        # Cartesian coordinates annotation is static and only needs to be drawn once
        cv2.putText(frame, "Cartesian coordinates:", (50, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 100, 20), 2)
        cv2.putText(frame, f"   X: {X_coordinate:.1f} cm", (100, 500), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 255), 2)
        cv2.putText(frame, f"   Y: {Y_coordinate:.1f} cm", (100, 530), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 255), 2)


