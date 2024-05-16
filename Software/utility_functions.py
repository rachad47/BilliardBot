import cv2
import numpy as np
import math
from image_processing import detect_colored_spots
from constants import POOL_BALL_DIAMETER ,RADIUS_ROBOT


"""
    Detects the HSV values of the pixel at the position where the user clicks
    on the frame. The pixel value is printed to the console.

    parameters:
    frame (np.array): The input image frame.

    returns:
    None
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
    Y_axis_spots = detect_colored_spots(frame, color_range, mask)
    Y_axis_center = None
    y_direction = None

    for spot in Y_axis_spots:
        Y_axis_center = calculate_center(spot)
        if Y_axis_center and origin:
            y_direction = (-Y_axis_center[0] + origin[0], -Y_axis_center[1] + origin[1]) #i reverted it
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
        center= (int(center[0]), int(center[1]))
        # Draw circle around the ball
        cv2.line(frame, origin, center, (255, 100, 255), 2)  # Line from origin to ball

        # Calculate distance
        ball_vector = (center[0] - origin[0], center[1] - origin[1])
        distance_pixels = math.sqrt(ball_vector[0]**2 + ball_vector[1]**2)
        pixel_to_cm_ratio = ball_diameter_cm / (2 * radius)
        distance_cm = distance_pixels * pixel_to_cm_ratio

        # Calculate angle
        dot_product = ball_vector[0]*y_direction[0] + ball_vector[1]*y_direction[1]
        angle_degrees = 0
        dirX=0
        dirY=0
        if (distance_pixels * y_length) != 0:

            cosine_value = dot_product / (distance_pixels * y_length)
            clamped_cosine_value = max(-1, min(1, cosine_value))
            angle = math.acos(clamped_cosine_value)
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
        x=80 
        cv2.putText(frame, "Cartesian coordinates:", (50, 60+x), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 100, 20), 2)
        cv2.putText(frame, f"   X: {X_coordinate:.1f} cm", (100, 100+x), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 255), 2)
        cv2.putText(frame, f"   Y: {Y_coordinate:.1f} cm", (100, 130+x), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 255), 2)


""" draw dotted lines between two points
    Parameters:
    img (np.array): The image frame to draw the line on.
    pt1 (tuple): The (x, y) coordinates of the first point.
    pt2 (tuple): The (x, y) coordinates of the second point.
    color (tuple): The (B, G, R) color values for the line.
    thickness (int): The thickness of the line.
    gap (int): The gap between each dot on the line.

    Returns:
    None
"""

def draw_dotted_line(img, pt1, pt2, color, thickness=1, gap=20):
    if pt1 and pt2:
        dist = np.linalg.norm(np.array(pt1) - np.array(pt2))
        points_along_line = int(dist / gap)
        for i in range(points_along_line):
            start = (int(pt1[0] + (pt2[0] - pt1[0]) * i / points_along_line), int(pt1[1] + (pt2[1] - pt1[1]) * i / points_along_line))
            end = (int(pt1[0] + (pt2[0] - pt1[0]) * (i + 0.5) / points_along_line), int(pt1[1] + (pt2[1] - pt1[1]) * (i + 0.5) / points_along_line))
            cv2.line(img, start, end, color, thickness)


"""Given the cue ball, the pocket centers and the size of the raduis, this function would:
    - make a circle (robot raduis) around the cue bal
    - draw a line from the cue ball to the pocket center
    - calculate the intersection of the line and the circle which origin is the cue balls center and mark that point in brown

    Parameters:
    frame (np.array): The image frame for reference.
    pocket_center (tuple): The (x, y) coordinates of the pocket center.
    cue_center (tuple): The (x, y) coordinates of the cue ball center.
    cirle_center (tuple): The (x, y) coordinates of the circle center. (which is this case is the cue ball center)
    raduis (int): The radius of the circle.

    returns:
     the furthest intersection point (from the pocket center)
"""
def line_circle_intersection(frame, pocket_center, cue_center, cirle_center, raduis):

    # Draw a circle around the cue ball as the robot radius
    cv2.circle(frame, cue_center, int(raduis), (150, 150, 205), 2)
    x1, y1 = pocket_center
    x2, y2 = cue_center
    x0, y0 = cirle_center
    
    # Handle vertical line separately
    if x2 == x1:
        a = 1
        b = -2 * y0
        c = y0**2 + (x1 - x0)**2 - raduis**2
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            return []
        else:
            y1_root = (-b + math.sqrt(discriminant)) / (2*a) + y0
            y2_root = (-b - math.sqrt(discriminant)) / (2*a) + y0
            return [(x1, y1_root), (x1, y2_root)]
    else:
        # Non-vertical line
        m = (y2 - y1) / (x2 - x1)
        c = y1 - m * x1
        
        A = 1 + m**2
        B = 2*m*(c - y0) - 2*x0
        C = x0**2 + (c - y0)**2 - raduis**2

        discriminant = B**2 - 4*A*C
        if discriminant < 0:
            return []
        else:
            sqrt_disc = math.sqrt(discriminant)
            x1_root = (-B + sqrt_disc) / (2*A)
            x2_root = (-B - sqrt_disc) / (2*A)
            y1_root = m*x1_root + c
            y2_root = m*x2_root + c

            # calculate the distances between the pocket center and the intersection points and pick the furtest one
            distance1 = np.sqrt((x1 - x1_root)**2 + (y1 - y1_root)**2)
            distance2 = np.sqrt((x1 - x2_root)**2 + (y1 - y2_root)**2)
            if distance1 > distance2:
                ans= [(int(x1_root), int(y1_root))]
            else:
                ans= [(int(x2_root), int(y2_root))]

            # mark the intersection point in brown
            cv2.circle(frame, ans[0], 5, (0, 100, 100), -1)
            
            return ans
        

# find_reflection and find_intersection functions are used to calculate the reflection and intersection points of the cue ball with the table boundaries respectively.

""" reflect a point across a line defined by two points
    Parameters:
    ptA (tuple): The (x, y) coordinates of the point to reflect.
    pt1 (tuple): The (x, y) coordinates of the first point on the line.
    pt2 (tuple): The (x, y) coordinates of the second point on the line.

    returns:
    the reflected point
"""
def find_reflection(ptA,  pt1, pt2):
    
     # Unpack points
    x1, y1 = pt1
    x2, y2 = pt2
    x, y = ptA

    # Coefficients of the line equation: Ax + By + C = 0
    A = y2 - y1
    B = -(x2 - x1)
    C = x2*y1 - x1*y2

    # Calculate the reflection of point A across the line
    D = A**2 + B**2
    x_prime = x - 2 * (A*x + B*y + C) * A / D
    y_prime = y - 2 * (A*x + B*y + C) * B / D

    ans= (int(x_prime), int(y_prime))
    
    return ans


""" find the intersection point of two lines defined by two points each
    Parameters:
    ptA (tuple): The (x, y) coordinates of the first point on the first line.
    ptB (tuple): The (x, y) coordinates of the second point on the first line.
    pt1 (tuple): The (x, y) coordinates of the first point on the second line.
    pt2 (tuple): The (x, y) coordinates of the second point on the second line.

    returns:
    the intersection point"""

def find_intersection(ptA, ptB, pt1, pt2):
    # Calculate the coefficients for the equations of the lines
    ptA=find_reflection(ptA,  pt1, pt2)
    A1 = ptB[1] - ptA[1]
    B1 = ptA[0] - ptB[0]
    C1 = A1 * ptA[0] + B1 * ptA[1]
    
    A2 = pt2[1] - pt1[1]
    B2 = pt1[0] - pt2[0]
    C2 = A2 * pt1[0] + B2 * pt1[1]
    
    # Calculate the determinant of the coefficient matrix
    determinant = A1 * B2 - A2 * B1
    
    if determinant == 0:
        # Lines are parallel or coincident
        return None
    else:
        # Solve the system of equations to find the intersection point
        x = (B2 * C1 - B1 * C2) / determinant
        y = (A1 * C2 - A2 * C1) / determinant
        ans= (int(x), int(y))
        return ans



