import cv2
import numpy as np

# Load video
cap = cv2.VideoCapture("./software/fire7.mov")

# Get FPS for proper playback speed

while True:
    ret, frame = cap.read()
    print("?")
    if not ret:
        break
    
    print("?")
    # Resize for performance (optional)

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Smooth image to reduce noise
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

    print("?")
    # Fire color range (tuned)
    lower_fire = np.array([0, 20, 230])     # Includes low saturation flames
    upper_fire = np.array([50, 150, 255])   # Up to bright yellow flames

    # Create mask
    mask = cv2.inRange(blurred, lower_fire, upper_fire)
    print("?")
    # Morphological operations to clean noise
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find fire contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print("?")
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 150:
            # Bounding box
            x, y, w, h = cv2.boundingRect(cnt)
            center_x = x + w // 2
            center_y = y + h // 2

            # Extract fire ROI
            fire_roi = frame[y:y+h, x:x+w]
            roi_hsv = cv2.cvtColor(fire_roi, cv2.COLOR_BGR2HSV)
            v_channel = roi_hsv[:, :, 2]

            # Find brightest pixel in ROI
            _, _, _, maxLoc = cv2.minMaxLoc(v_channel)
            bright_x = x + maxLoc[0]
            bright_y = y + maxLoc[1]

            # Draw bounding box (red)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # Draw center (green)
            cv2.circle(frame, (center_x, center_y), 4, (0, 255, 0), -1)

            # Draw brightest point (blue)
            cv2.circle(frame, (bright_x, bright_y), 4, (255, 0, 0), -1)

            # Label
            cv2.putText(frame, "🔥 Fire", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Console output
            print(f"🔥 Fire center: ({center_x}, {center_y}), Brightest point: ({bright_x}, {bright_y})")

    # Show frame
    cv2.imshow("Fire Detection", frame)

    # Quit on 'q'
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# Release and cleanup
cap.release()
cv2.destroyAllWindows()
