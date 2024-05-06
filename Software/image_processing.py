import cv2
import numpy as np


"""
    Detects the largest white area in the frame, which is considered the background boundary.
    Uses HSV color space for better color segmentation and applies morphological operations 
    to clean up the mask.

    Parameters:
    frame (np.array): The input image frame in which to detect the background boundary.

    Returns:
    np.array: The largest contour found in the frame representing the background boundary.
    """
def detect_backgroud_boudary(frame,table_color_range):    
    
    # Convert to HSV for better color segmentation
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for white color
    lower_white = table_color_range[0]
    upper_white = table_color_range[1]

    # lower_white = np.array([0, 0, 100])
    # upper_white = np.array([179, 40, 255])

    # Create a mask for white color
    white_mask = cv2.inRange(hsv, lower_white, upper_white)
    # Apply morphology to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    # Find contours for the white area
    # cv2.imshow('white_mask', white_mask)
    contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 255), 3) # Yellow contour
        return largest_contour
    return None



"""
    Detects the largest pink area within a given white boundary in the frame.
    
    Parameters:
    frame (np.array): The input image frame in which to detect the pink paper.
    white_mask (np.array): The mask representing the white area to constrain the detection.

    Returns:
    np.array: The largest contour found representing the pink paper.
    """
def detect_pink_paper(frame, white_mask, robot_color_range):
     # Convert to HSV for better color segmentation
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Define range for pink color
    lower_pink = robot_color_range[0]
    upper_pink = robot_color_range[1]

    # Create a mask for pink color
    pink_mask = cv2.inRange(hsv, lower_pink, upper_pink)
    # cv2.imshow('pink_mask', pink_mask)
    # Apply the white area mask to the pink mask
    masked_pink = cv2.bitwise_and(pink_mask, pink_mask, mask=white_mask)
    # cv2.imshow('pink_mask', masked_pink)

    # Find contours of the pink paper
    contours, _ = cv2.findContours(masked_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(frame, [largest_contour], 0, (190, 90, 100), 2)  
        return largest_contour
    return None


"""
    Detects colored spots within a specified region of the frame. This function creates 
    a mask for the specified color and applies it to the given region mask.

    Parameters:
    frame (np.array): The input image frame in which to detect colored spots.
    color_mask (tuple): The lower and upper color range for spot detection.
    region_mask (np.array): The mask representing the region to constrain the detection.

    Returns:
    list: A list of contours representing the detected colored spots.
    """
def detect_colored_spots(frame, color_mask, region_mask):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Create a mask for colored spots
    colored_spots_mask = cv2.inRange(hsv, color_mask[0], color_mask[1])
    # cv2.imshow('colored_spots_mask', colored_spots_mask)
    # Apply the region mask to the colored spots mask
    masked_colored_spots = cv2.bitwise_and(colored_spots_mask, colored_spots_mask, mask=region_mask)
    # Find contours of the colored spots
    contours, _ = cv2.findContours(masked_colored_spots, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)
    return contours
#stupid function
def detect_colored_spots2(frame, color_mask, region_mask):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Create a mask for colored spots
    colored_spots_mask = cv2.inRange(hsv, color_mask[0], color_mask[1])
    # cv2.imshow('colored_spots_mask2', colored_spots_mask)
    # Apply the region mask to the colored spots mask
    masked_colored_spots = cv2.bitwise_and(colored_spots_mask, colored_spots_mask, mask=region_mask)
    # Find contours of the colored spots
    contours, _ = cv2.findContours(masked_colored_spots, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (255, 255, 0), 1)
    return contours


"""
    Detects balls on a table within a given color range.

    Parameters:
    frame (np.array): The image frame in which to detect the balls.
    table_contour (np.array): The contour that defines the boundary of the table.
    color_range (tuple): The lower and upper range for the ball color.
    min_contour_area (int): The minimum area threshold for a contour to be considered a ball.

    Returns:
    list: A list of tuples, each containing the center coordinates and radius of a detected ball.
    """
def detect_balls(frame, table_contour, color_range, min_contour_area=100): 
    
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the ball color
    ball_mask = cv2.inRange(hsv, color_range[0], color_range[1])

    # Create a mask from the table contour
    table_mask = np.zeros_like(frame[:, :, 0])
    cv2.drawContours(table_mask, [table_contour], -1, 255, -1)

    # Combine the table mask with the color mask
    combined_mask = cv2.bitwise_and(ball_mask, ball_mask, mask=table_mask)

    # Find contours for the balls
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    balls = []
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            balls.append(((int(x), int(y)), int(radius)))
    
    return balls

"""
    Detects the pockets on a pool table within a given color range.

    Parameters:
    frame (np.array): The image frame in which to detect the pockets.
    color_range (tuple): The lower and upper range for the pocket color.
    min_contour_area (int): The minimum area threshold for a contour to be considered a pocket.

    Returns:
    list: A list of tuples, each containing the center coordinates and radius of a detected pocket.
"""

def detect_pockets(frame, color_range, min_contour_area=100):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the pocket color
    pocket_mask = cv2.inRange(hsv, color_range[0], color_range[1])

    # Find contours for the pockets
    contours, _ = cv2.findContours(pocket_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    pockets = []
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            pockets.append(((int(x), int(y)), int(radius)))

    return pockets
