import cv2
import numpy as np
import math

# Initialize USB camera (usually device 0)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Give camera time to warm up
time.sleep(2)

def detect_shape(contour):
    shape = "unknown"
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
    vertices = len(approx)
    
    # Calculate area and bounding rectangle
    area = cv2.contourArea(contour)
    (x, y, w, h) = cv2.boundingRect(approx)
    aspect_ratio = w / float(h)
    
    # Calculate circularity
    circularity = (4 * math.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0
    
    # Shape detection logic
    if vertices == 3:
        shape = "triangle"
    elif vertices == 4:
        if 0.95 <= aspect_ratio <= 1.05:
            shape = "square"
        else:
            shape = "rectangle"
    elif vertices == 5:
        shape = "pentagon"
    elif vertices == 6:
        shape = "hexagon"
    elif vertices >= 8:
        if circularity > 0.85:
            shape = "circle"
        elif circularity > 0.75:
            shape = "ellipse"
        else:
            # Check for star or clover
            hull = cv2.convexHull(contour, returnPoints=False)
            defects = cv2.convexityDefects(contour, hull)
            
            if defects is not None and len(defects) > 4:
                # Count significant concavities
                concavities = 0
                for i in range(defects.shape[0]):
                    s, e, f, d = defects[i, 0]
                    if d > perimeter * 0.05:  # Minimum defect depth
                        concavities += 1
                
                if concavities >= 4:
                    shape = "four leaf clover"
                elif concavities >= 5:
                    shape = "star"
    
    # Special case for cylinder (appears as rectangle with ellipses in 3D)
    if shape == "rectangle" and area > 1000:
        # Look for parallel lines or other cylinder features
        # This might need adjustment based on how cylinders appear in your images
        lines = cv2.HoughLinesP(
            cv2.Canny(cv2.drawContours(np.zeros_like(frame), [contour], -1, 255, 2), 50, 150),
            1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=10
        )
        if lines is not None and len(lines) >= 2:
            # Check if lines are roughly parallel
            angles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angles.append(math.atan2(y2 - y1, x2 - x1))
            
            # Check if most lines are parallel
            angle_variance = np.var(angles)
            if angle_variance < 0.1:
                shape = "cylinder"
    
    return shape

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Pre-process the image (important for underwater conditions)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Adaptive thresholding works better than simple thresholding underwater
    thresh = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 11, 2
    )
    
    # Find contours
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    
    # Process each contour
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 300:  # Skip small contours (adjust based on your needs)
            continue
        
        shape = detect_shape(contour)
        
        # Draw the contour and label
        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
        
        # Get contour moments for centroid
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # Put the shape name
            cv2.putText(frame, shape, (cX - 20, cY - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # Display the resulting frame
    cv2.imshow('ROV Object Detection', frame)
    
    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()