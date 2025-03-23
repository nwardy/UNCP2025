import cv2
import numpy as np
import time
import board
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

############################################
# 1. Initialize MotorKit Instances
############################################

# Primary MotorKit for wheels and stepper (assumed default address)
kit = MotorKit()

# Separate MotorKit for water pump control (update address if needed)
pump_kit = MotorKit(address=0xA0)

############################################
# 2. Define DC Motor (Wheel) Control Functions for Two-Wheel Drive
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
# 3. Define Water Pump Control Function
############################################

def activate_water_pump(duration, throttle=1.0):
    """
    Activate the water pump for a specified duration.
    Assumes the water pump is controlled via pump_kit.motor1.
    :param duration: float, seconds to run the pump.
    :param throttle: float, pump power (0.0 to 1.0).
    """
    print("Activating water pump...")
    pump_kit.motor1.throttle = throttle
    time.sleep(duration)
    pump_kit.motor1.throttle = 0
    print("Water pump deactivated.")

############################################
# 4. Fire Detection Code with OpenCV
############################################

# Open the video file
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

    
# Variables for tracking fire stability
prev_fire_center = None       # Tuple (avg_x, avg_y) of previous frame's fire center
fire_stable_start_time = None # Time when the fire was first detected as stable
water_pump_active = False     # Flag to prevent repeated pump activation
water_pump_cooldown = 10      # Cooldown in seconds after pump activation
last_pump_time = 0            # Time when pump was last fired

# To prevent continuous wheel activation, track the last spin time.
last_wheel_spin = time.time() - 10

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame from BGR to HSV and apply Gaussian blur
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Define the fire color range (HSV)
    # These values capture low-saturation, very bright fire regions
    lower_fire = np.array([0, 20, 230])
    upper_fire = np.array([50, 150, 255])

    # Create a binary mask for fire-colored pixels
    mask = cv2.inRange(blurred, lower_fire, upper_fire)

    # Clean up noise using morphological operations
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours from the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    fire_centers_x = []
    fire_centers_y = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 150:  # Filter out small regions
            x, y, w, h = cv2.boundingRect(cnt)
            center_x = x + w // 2
            center_y = y + h // 2
            fire_centers_x.append(center_x)
            fire_centers_y.append(center_y)
            
            # Process ROI to find the brightest point (using V channel)
            fire_roi = frame[y:y+h, x:x+w]
            roi_hsv = cv2.cvtColor(fire_roi, cv2.COLOR_BGR2HSV)
            v_channel = roi_hsv[:, :, 2]
            _, _, _, maxLoc = cv2.minMaxLoc(v_channel)
            bright_x = x + maxLoc[0]
            bright_y = y + maxLoc[1]
            
            # Draw detection results
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(frame, (center_x, center_y), 4, (0, 255, 0), -1)
            cv2.circle(frame, (bright_x, bright_y), 4, (255, 0, 0), -1)
            cv2.putText(frame, "🔥 Fire", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2)
            print(f"🔥 Fire center: ({center_x}, {center_y}), Brightest point: ({bright_x}, {bright_y})")
    
    ############################################
    # 5. Stepper Motor Control for Pitch Adjustment
    ############################################

    if fire_centers_y:
        avg_fire_y = int(sum(fire_centers_y) / len(fire_centers_y))
        frame_center_y = frame.shape[0] // 2
        tolerance = 10

        if avg_fire_y < frame_center_y - tolerance:
            print("Moving pitch up")
            # Increase steps per frame and reduce delay for faster pitch movement
            for _ in range(7):
                kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
                time.sleep(0.005)
        elif avg_fire_y > frame_center_y + tolerance:
            print("Moving pitch down")
            for _ in range(7):
                kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
                time.sleep(0.005)
    
    ############################################
    # 6. DC Motor (Wheel) Control for Chassis Spinning
    ############################################

    if fire_centers_x and (time.time() - last_wheel_spin > 5):
        spin_wheels_sequence()
        last_wheel_spin = time.time()

    ############################################
    # 7. Water Pump Control (Activate Only if Fire is Stable)
    ############################################

    # If at least one fire region is detected, compute average center coordinates.
    if fire_centers_x and fire_centers_y:
        avg_fire_x = int(sum(fire_centers_x) / len(fire_centers_x))
        avg_fire_y = int(sum(fire_centers_y) / len(fire_centers_y))
        current_time = time.time()
        stability_tolerance = 20  # Tolerance in pixels for stability

        if prev_fire_center is None:
            prev_fire_center = (avg_fire_x, avg_fire_y)
            fire_stable_start_time = current_time
        else:
            dx = abs(avg_fire_x - prev_fire_center[0])
            dy = abs(avg_fire_y - prev_fire_center[1])
            # If fire center is within tolerance, consider it stable.
            if dx < stability_tolerance and dy < stability_tolerance:
                # Check if it has been stable for at least 4 seconds.
                if current_time - fire_stable_start_time >= 4:
                    # Also check for a cooldown period after firing the pump.
                    if current_time - last_pump_time > water_pump_cooldown:
                        activate_water_pump(duration=2, throttle=1.0)
                        last_pump_time = current_time
            else:
                # Reset stability tracking if the fire moves significantly.
                prev_fire_center = (avg_fire_x, avg_fire_y)
                fire_stable_start_time = current_time

    # Display the processed frame
    cv2.imshow("Fire Detection", frame)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# Clean up resources
cap.release()
cv2.destroyAllWindows()
