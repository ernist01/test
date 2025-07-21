import cv2
import numpy as np

# Define a dictionary to store the names of the shapes
shape_names = {
    3: 'Triangle',
    4: 'Square/Rectangle',  # Squares and rectangles both have 4 sides
    5: 'Pentagon',
    6: 'Hexagon',
    8: 'Octagon',
    7: 'Star',  # Star is more complex to detect
    # Shapes with curved edges
    'ellipse': 'Ellipse',  # Ellipse
    'circle': 'Circle',    # Circle
}

# Function to detect the shape based on the number of vertices
def detect_shape(approx):
    # Based on the number of vertices, return the shape name
    if len(approx) == 3:
        return shape_names[3]  # Triangle
    elif len(approx) == 4:
        # Check if the shape is a square or rectangle
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        if aspect_ratio >= 0.95 and aspect_ratio <= 1.05:  # Square
            return 'Square'
        else:  # Rectangle
            return 'Rectangle'
    elif len(approx) == 5:
        return shape_names[5]  # Pentagon
    elif len(approx) == 6:
        return shape_names[6]  # Hexagon
    elif len(approx) > 6:
        # Use circularity to distinguish between ellipse and circle
        (x, y), (MA, ma), angle = cv2.fitEllipse(approx)
        aspect_ratio = MA / ma
        if aspect_ratio < 1.5:  # Circle
            return 'Circle'
        else:
            return 'Ellipse'
    return None

# Capture video feed from the camera
cap = cv2.VideoCapture(0)

while True:
    # Read frame by frame
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Blur the image to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply edge detection
    edges = cv2.Canny(blurred, 100, 200)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through the contours and identify the shapes
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter out small contours
            # Approximate the contour to a polygon
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Detect the shape
            shape = detect_shape(approx)
            
            if shape:
                # Get the bounding box for drawing
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, shape, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # Show the resulting frame
    cv2.imshow('Shape Detection', frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
