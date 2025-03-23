import cv2
import numpy as np

video_path = "software/IMG_1240 (1).mov"
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error opening video file")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame from BGR to HSV for color thresholding
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Refined color range for a bright orange/yellow flame
    # Adjust as necessary if your flame is more reddish or more yellowish
    lower_orange = np.array([20, 90, 180])   # e.g., [10, 150, 150]
    upper_orange = np.array([25, 180, 255])   # e.g., [25, 255, 255]

    # Create a binary mask
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Optional: Clean up noise in the mask with morphological operations
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        # Filter out very small contours so random noise isn't counted as fire
        if area < 20:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        # Crop the region of interest in HSV
        roi_hsv = hsv[y:y+h, x:x+w]
        if roi_hsv.size == 0:
            continue

        # Calculate the average brightness (V channel) in the ROI
        avg_brightness = np.mean(roi_hsv[:, :, 2])

        # Only proceed if the region is bright enough
        # (tweak 130-160 range depending on lighting conditions)
        if avg_brightness < 150:
            continue

        # Draw a bounding box around the detected region
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
        cv2.putText(frame, "Fire", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

    cv2.imshow("Small Flame Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
