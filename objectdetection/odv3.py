import cv2
import numpy as np
import math
import time
from datetime import datetime
import os

# Configuration
SHAPES = ["triangle", "star", "square", "ellipse", "cylinder", 
          "pentagon", "four leaf clover", "hexagon", "rectangle", "circle"]
RECORDING_FPS = 15
RESOLUTION = (1280, 720)  # Higher resolution for better quality
MIN_CONTOUR_AREA = 300
DETECTION_COLOR = (0, 255, 0)  # Green
TEXT_COLOR = (255, 255, 255)  # White

class ShapeDetector:
    def __init__(self):
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=False)
        
    def preprocess(self, frame):
        # Underwater enhancement
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        cl = self.clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        
        # Background subtraction to focus on moving objects
        fgmask = self.fgbg.apply(enhanced)
        return enhanced, fgmask

    def detect_shape(self, contour):
        shape = "unknown"
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        vertices = len(approx)
        
        area = cv2.contourArea(contour)
        (x, y, w, h) = cv2.boundingRect(approx)
        aspect_ratio = w / float(h)
        circularity = (4 * math.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0
        
        # Shape classification logic
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
                    concavities = sum(1 for i in range(defects.shape[0]) if defects[i,0,3] > perimeter*0.05)
                    if concavities >= 4:
                        shape = "four leaf clover" if concavities == 4 else "star"
        
        # Special case for cylinder
        if shape == "rectangle" and area > 1000:
            edges = cv2.Canny(cv2.drawContours(np.zeros_like(frame), [contour], -1, 255, 2), 50, 150)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 30, 10)
            if lines is not None and len(lines) >= 2:
                angles = [math.atan2(y2-y1, x2-x1) for line in lines for x1,y1,x2,y2 in line]
                if np.var(angles) < 0.1:
                    shape = "cylinder"
        
        return shape if shape in SHAPES else "unknown"

def initialize_camera():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(2)  # Camera warm-up
    return cap

def initialize_recording():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"rov_recording_{timestamp}.avi"
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(filename, fourcc, RECORDING_FPS, RESOLUTION)
    return out, filename

def main():
    cap = initialize_camera()
    detector = ShapeDetector()
    recording = False
    out = None
    
    detected_objects = []
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Process frame
            enhanced, fgmask = detector.preprocess(frame)
            thresh = cv2.adaptiveThreshold(cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY), 
                                        255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                        cv2.THRESH_BINARY_INV, 11, 2)
            
            # Combine with motion mask
            combined = cv2.bitwise_and(thresh, thresh, mask=fgmask)
            
            # Find contours
            contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process contours
            processed_frame = enhanced.copy()
            current_frame_objects = []
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < MIN_CONTOUR_AREA:
                    continue
                
                shape = detector.detect_shape(contour)
                if shape != "unknown":
                    # Draw detection
                    cv2.drawContours(processed_frame, [contour], -1, DETECTION_COLOR, 2)
                    
                    # Add label
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.putText(processed_frame, shape, (cX - 20, cY - 20),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 2)
                        
                        # Record detection
                        current_frame_objects.append({
                            "frame": frame_count,
                            "shape": shape,
                            "position": (cX, cY),
                            "timestamp": time.time()
                        })
            
            # Display
            cv2.imshow('ROV Object Detection', processed_frame)
            
            # Recording control
            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                if not recording:
                    out, filename = initialize_recording()
                    recording = True
                    print(f"Started recording: {filename}")
                else:
                    out.release()
                    recording = False
                    print(f"Stopped recording. Saved as {filename}")
            
            if recording:
                out.write(processed_frame)
                detected_objects.extend(current_frame_objects)
                frame_count += 1
            
            if key == ord('q'):
                break
    
    finally:
        cap.release()
        if recording:
            out.release()
        cv2.destroyAllWindows()
        
        # Save detection log
        if detected_objects:
            log_filename = f"detections_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(log_filename, 'w') as f:
                f.write("frame,shape,x,y,timestamp\n")
                for obj in detected_objects:
                    f.write(f"{obj['frame']},{obj['shape']},{obj['position'][0]},{obj['position'][1]},{obj['timestamp']}\n")
            print(f"Detection log saved as {log_filename}")

if __name__ == "__main__":
    main()