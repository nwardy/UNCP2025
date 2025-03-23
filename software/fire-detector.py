import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

############################################
# 1. Setup for Stepper Motor Control
############################################

# Define GPIO pins for the stepper motor
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

# Setup GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Define the step sequence for the stepper motor (example for a 28BYJ-48)
step_seq = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

def step_motor(steps, direction=1, delay=0.002):
    """
    Rotate the stepper motor a given number of steps.
    :param steps: Number of steps to move.
    :param direction: 1 for one direction, -1 for the opposite.
    :param delay: Delay between steps.
    """
    # Select sequence order based on desired direction
    seq = step_seq if direction == 1 else step_seq[::-1]
    for _ in range(steps):
        for half_step in seq:
            GPIO.output(IN1, half_step[0])
            GPIO.output(IN2, half_step[1])
            GPIO.output(IN3, half_step[2])
            GPIO.output(IN4, half_step[3])
            time.sleep(delay)

############################################
# 2. Fire Detection Code with OpenCV
############################################

# Load the video file
cap = cv2.VideoCapture("./software/fire7.mov")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame from BGR to HSV and smooth to reduce noise
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Define the fire color range (HSV)
    # Adjusted values to capture low-saturation but very bright fire regions
    lower_fire = np.array([0, 20, 230])     
    upper_fire = np.array([50, 150, 255])   

    # Create a binary mask where pixels within the range are white
    mask = cv2.inRange(blurred, lower_fire, upper_fire)

    # Clean up noise with morphological operations
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    fire_centers = []  # To store the y-coordinates of detected fires

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 150:  # Filter out very small regions
            # Get bounding box and compute center of the detected fire region
            x, y, w, h = cv2.boundingRect(cnt)
            center_x = x + w // 2
            center_y = y + h // 2
            fire_centers.append(center_y)

            # Further process ROI to find the brightest point (using V channel)
            fire_roi = frame[y:y+h, x:x+w]
            roi_hsv = cv2.cvtColor(fire_roi, cv2.COLOR_BGR2HSV)
            v_channel = roi_hsv[:, :, 2]
            _, _, _, maxLoc = cv2.minMaxLoc(v_channel)
            bright_x = x + maxLoc[0]
            bright_y = y + maxLoc[1]

            # Draw detection results on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red box
            cv2.circle(frame, (center_x, center_y), 4, (0, 255, 0), -1)      # Green center
            cv2.circle(frame, (bright_x, bright_y), 4, (255, 0, 0), -1)        # Blue brightest point
            cv2.putText(frame, "🔥 Fire", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2)
            print(f"🔥 Fire center: ({center_x}, {center_y}), Brightest point: ({bright_x}, {bright_y})")

    ############################################
    # 3. Stepper Motor Control for Pitch Adjustment
    ############################################

    # If fire detections exist, adjust pitch based on average vertical position
    if fire_centers:
        avg_fire_y = int(sum(fire_centers) / len(fire_centers))
        frame_center_y = frame.shape[0] // 2  # Vertical center of the frame
        tolerance = 10  # Pixel tolerance to avoid jittering

        # If the fire is above the frame's center, move pitch up (step motor in one direction)
        if avg_fire_y < frame_center_y - tolerance:
            print("Moving pitch up")
            step_motor(steps=5, direction=1)
        # If the fire is below the frame's center, move pitch down (step motor in opposite direction)
        elif avg_fire_y > frame_center_y + tolerance:
            print("Moving pitch down")
            step_motor(steps=5, direction=-1)

    # Display the processed frame
    cv2.imshow("Fire Detection", frame)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# Clean up resources
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
