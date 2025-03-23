import cv2
import numpy as np
import time
import board
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

############################################
# 1. Initialize MotorKit for Motor Control
############################################

# Create a MotorKit instance (uses I2C on board.I2C())
kit = MotorKit()

############################################
# 2. Define DC Motor (Wheel) Control Functions for Two Wheels
############################################

def spin_one_wheel(wheel, direction, duration, throttle=1.0):
    """
    Run a single wheel motor for a specified duration.
    
    :param wheel: int, 1 for left wheel or 2 for right wheel.
    :param direction: 'forward' or 'backward'
    :param duration: float, seconds to run the motor.
    :param throttle: float, motor power (0.0 to 1.0).
    """
    power = throttle if direction == 'forward' else -throttle
    if wheel == 1:
        kit.motor1.throttle = power
    elif wheel == 2:
        kit.motor2.throttle = power
    time.sleep(duration)
    # Stop the motor after running
    if wheel == 1:
        kit.motor1.throttle = 0
    elif wheel == 2:
        kit.motor2.throttle = 0

def spin_wheels_sequence():
    """
    Spin each wheel individually in sequence.
    Running one wheel at a time causes the robot to pivot.
    """
    print("Spinning left wheel...")
    spin_one_wheel(wheel=1, direction='forward', duration=0.3, throttle=1.0)
    time.sleep(0.1)  # Short pause between wheels
    print("Spinning right wheel...")
    spin_one_wheel(wheel=2, direction='forward', duration=0.3, throttle=1.0)
    time.sleep(0.1)

############################################
# 3. Fire Detection Code with OpenCV
############################################

# Load the video file
cap = cv2.VideoCapture("./software/fire7.mov")

# To prevent continuous wheel activation, track the last spin time.
last_wheel_spin = time.time() - 10  # Initialize to allow immediate spin

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame from BGR to HSV and smooth to reduce noise
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Define the fire color range (HSV)
    # These values capture low-saturation, very bright fire regions.
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
    fire_centers = []  # To store the y-coordinates of detected fire regions

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 150:  # Filter out small regions
            # Get bounding box and compute the center of the detected fire region
            x, y, w, h = cv2.boundingRect(cnt)
            center_x = x + w // 2
            center_y = y + h // 2
            fire_centers.append(center_y)

            # Process ROI to find the brightest point (using V channel)
            fire_roi = frame[y:y+h, x:x+w]
            roi_hsv = cv2.cvtColor(fire_roi, cv2.COLOR_BGR2HSV)
            v_channel = roi_hsv[:, :, 2]
            _, _, _, maxLoc = cv2.minMaxLoc(v_channel)
            bright_x = x + maxLoc[0]
            bright_y = y + maxLoc[1]

            # Draw detection results on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red bounding box
            cv2.circle(frame, (center_x, center_y), 4, (0, 255, 0), -1)      # Green center
            cv2.circle(frame, (bright_x, bright_y), 4, (255, 0, 0), -1)        # Blue brightest point
            cv2.putText(frame, "🔥 Fire", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2)
            print(f"🔥 Fire center: ({center_x}, {center_y}), Brightest point: ({bright_x}, {bright_y})")

    ############################################
    # 4. Stepper Motor Control for Pitch Adjustment
    ############################################

    if fire_centers:
        avg_fire_y = int(sum(fire_centers) / len(fire_centers))
        frame_center_y = frame.shape[0] // 2  # Vertical center of the frame
        tolerance = 10  # Pixel tolerance to avoid jitter

        if avg_fire_y < frame_center_y - tolerance:
            print("Moving pitch up")
            for _ in range(5):  # Adjust the number of steps as needed
                kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
                time.sleep(0.01)
        elif avg_fire_y > frame_center_y + tolerance:
            print("Moving pitch down")
            for _ in range(5):
                kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
                time.sleep(0.01)

    ############################################
    # 5. DC Motor (Wheel) Control for Spinning the Chassis
    ############################################

    # If a fire is detected and sufficient time has passed since the last wheel spin,
    # run a sequence that drives one wheel at a time.
    if fire_centers and (time.time() - last_wheel_spin > 5):
        spin_wheels_sequence()
        last_wheel_spin = time.time()

    # Display the processed frame
    cv2.imshow("Fire Detection", frame)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# Clean up resources
cap.release()
cv2.destroyAllWindows()
