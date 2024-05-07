import cv2
import numpy as np
# Open the webcam
cap = cv2.VideoCapture(2)


def detect_pockets(hsv_frame, color_range, min_contour_area=100):
    # Threshold the HSV image to get only the target colors
    lower_color = np.array(color_range[0])
    upper_color = np.array(color_range[1])
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on the minimum area and count them
    large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
    count = len(large_contours)

    # Optionally, draw the contours on the original frame (for visualization)
    for contour in large_contours:
        cv2.drawContours(hsv_frame, [contour], -1, (0, 255, 0), 3)  # Green color in BGR

    return hsv_frame, count

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect colored pockets (example with a hypothetical color range for blue)
    processed_frame, pocket_count = detect_pockets(frame, ([0,0,47], [179,49,54]))

    # Display the processed frame
    cv2.imshow('Frame', processed_frame)
    print(f"Count of large contours: {pocket_count}")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


