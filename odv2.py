import cv2
import numpy as np
import math
import time

def detect_shape(contour):
    shape = "unknown"
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
    vertices = len(approx)
    
    area = cv2.contourArea(contour)
    (x, y, w, h) = cv2.boundingRect(approx)
    aspect_ratio = w / float(h)
    circularity = (4 * math.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0
    
    if vertices == 3:
        shape = "triangle"
    elif vertices == 4:
        shape = "square" if 0.95 <= aspect_ratio <= 1.05 else "rectangle"
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
            hull = cv2.convexHull(contour, returnPoints=False)
            defects = cv2.convexityDefects(contour, hull)
            if defects is not None:
                concavities = sum(1 for i in range(defects.shape[0]) if defects[i,0,3] > perimeter*0.05 else 0)
                if concavities >= 4:
                    shape = "four leaf clover" if concavities == 4 else "star"
    
    if shape == "rectangle" and area > 1000:
        edges = cv2.Canny(cv2.drawContours(np.zeros_like(frame), [contour], -1, 255, 2), 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 30, 10)
        if lines is not None and len(lines) >= 2:
            angles = [math.atan2(y2-y1, x2-x1) for line in lines for x1,y1,x2,y2 in line]
            if np.var(angles) < 0.1:
                shape = "cylinder"
    
    return shape

# Initialize USB camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
time.sleep(2)  # Camera warm-up

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # CLAHE Enhancement for underwater (on CPU)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl,a,b))
    enhanced_frame = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    
    # Convert to UMat for GPU acceleration
    gpu_frame = cv2.UMat(enhanced_frame)
    
    # Pre-processing
    gray = cv2.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY_INV, 11, 2)
    
    # Contour detection (on CPU)
    cpu_thresh = cv2.UMat.get(thresh)
    contours, _ = cv2.findContours(cpu_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Prepare output frame
    output_frame = cv2.UMat.get(gpu_frame)
    
    # Process contours
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 300:
            continue
            
        shape = detect_shape(contour)
        cv2.drawContours(output_frame, [contour], -1, (0, 255, 0), 2)
        
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.putText(output_frame, shape, (cX - 20, cY - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # Display
    cv2.imshow('ROV Object Detection', output_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()