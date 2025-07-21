import cv2
import torch
import requests
import json

# Initialize YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Load YOLOv5 small model

# Initialize camera (0 is the default camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not access the camera.")
    exit()

# External server URL (replace with your external server URL)
server_url = "http://your-server.com/api/receive_data"

def send_to_server(detections):
    """Send detection results to external server via HTTP POST request."""
    payload = json.dumps({"detections": detections})
    headers = {"Content-Type": "application/json"}
    response = requests.post(server_url, data=payload, headers=headers)
    if response.status_code == 200:
        print("Detection data sent successfully.")
    else:
        print(f"Failed to send data: {response.status_code}")

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Perform inference with YOLOv5
    results = model(frame)

    # Parse the results
    detections = []
    for *xyxy, conf, cls in results.xywh[0]:
        label = model.names[int(cls)]
        confidence = float(conf)
        if confidence > 0.5:  # Threshold for confidence
            x1, y1, x2, y2 = map(int, xyxy)
            detections.append({
                "label": label,
                "confidence": confidence,
                "coordinates": [x1, y1, x2, y2]
            })
            # Draw bounding boxes around detected objects
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

    # Send the results to the external server
    if detections:
        send_to_server(detections)

    # Display the resulting frame
    cv2.imshow('YOLOv5 Object Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
