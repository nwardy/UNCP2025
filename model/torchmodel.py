import cv2
import torch
import numpy as np

# Load a pre-trained YOLOv5 model.
# For fire detection, you might have custom weights trained on fire images.
# For example, if you have custom weights, replace 'yolov5s' with 'custom'
# and provide the path to your weights file.
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='path/to/fire_weights.pt')
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.eval()  # set to evaluation mode

# Define thresholds.
SMALL_BBOX_THRESHOLD = 0.02  # Only consider detections with bbox area < 1% of frame area.
ORANGE_PIXEL_THRESHOLD = 0.1  # At least 20% of the ROI pixels should be orange.

# Video file path.
video_path = "software/fire6.mov"  # Replace with your video file path.
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error opening video file")
    exit()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the current frame.
    results = model(frame)
    # results.xyxy[0] contains detections as [x1, y1, x2, y2, confidence, class]
    detections = results.xyxy[0]
    frame_area = frame.shape[0] * frame.shape[1]

    for *box, conf, cls in detections:
        x1, y1, x2, y2 = map(int, box)
        bbox_area = (x2 - x1) * (y2 - y1)
        relative_area = bbox_area / frame_area

        # Only process detections with very small bounding box area.
        if relative_area < SMALL_BBOX_THRESHOLD:
            # Extract the region of interest (ROI).
            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                continue  # Skip if ROI is empty.

            # Convert ROI to HSV for color detection.
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            # Define an HSV range for a typical orange fire color.
            lower_orange = np.array([5, 100, 100])
            upper_orange = np.array([15, 255, 255])
            mask = cv2.inRange(hsv_roi, lower_orange, upper_orange)
            orange_ratio = cv2.countNonZero(mask) / (roi.shape[0] * roi.shape[1] + 1e-5)

            # Only draw detection if enough of the ROI is orange.
            if orange_ratio > ORANGE_PIXEL_THRESHOLD:
                # Draw bounding box and label in an orange color (BGR: 0,165,255).
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
                label = f"Fire: {conf:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

    cv2.imshow("Fire Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
