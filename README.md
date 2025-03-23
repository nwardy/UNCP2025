# Fire Detection and Robot Control System

This project integrates computer vision with hardware control to detect fire and respond with coordinated motor actions. The system:
- Uses OpenCV to detect fire based on HSV color thresholds.
- Adjusts the pitch (e.g., water nozzle angle) using a stepper motor.
- Pivots the robot chassis by running one wheel at a time (two-wheel drive).
- Activates a water pump (via a separate DC motor HAT on I²C address `0xA0`) only if a stable fire is detected for at least 4 seconds.

---

## System Components

- **Computer Vision (OpenCV):** Detects fire regions by analyzing video frames.
- **Stepper Motor:** Adjusts the pitch based on the vertical position of the detected fire.
- **DC Motors (Wheels):** Control the chassis; each motor powers one wheel so that the robot can pivot.
- **Water Pump:** Controlled by a separate DC motor HAT and activated when the fire remains stable.

---

## Code Overview

### 1. Import Libraries and Initialize MotorKit Instances

- **Imports:**  
  The code uses:
  - OpenCV and NumPy for image processing.
  - `time` for time tracking.
  - `board`, `MotorKit`, and `stepper` (from Adafruit) for hardware control.

- **MotorKit Instances:**  
  - **Primary MotorKit (`kit`):** Controls the wheels (DC motors) and the stepper motor for pitch adjustment.
  - **Pump MotorKit (`pump_kit`):** Initialized with I²C address `0xA0`, dedicated to controlling the water pump.

### 2. DC Motor (Wheel) Control Functions for Two-Wheel Drive

- **`spin_one_wheel`:**  
  Runs a single wheel motor for a specified duration.
  - **Parameters:**  
    - `wheel`: 1 for the left wheel or 2 for the right wheel.
    - `direction`: `'forward'` or `'backward'`.
    - `duration`: Time in seconds to run the motor.
    - `throttle`: Motor power (0.0 to 1.0).
  - **Behavior:**  
    Sets the motor’s throttle (positive for forward, negative for backward), waits, and then stops the motor.

- **`spin_wheels_sequence`:**  
  Activates each wheel individually in sequence.
  - Running one wheel at a time causes the robot to pivot, which helps spread the water more effectively.

### 3. Water Pump Control Function

- **`activate_water_pump`:**  
  Activates the water pump via the pump HAT.
  - **Parameters:**  
    - `duration`: Seconds to run the pump.
    - `throttle`: Pump power (0.0 to 1.0).
  - **Behavior:**  
    Sets the pump motor’s throttle, runs it for the specified duration, and then turns it off.

### 4. Fire Detection Code with OpenCV

- **Video Capture:**  
  Opens the video file (`./software/fire7.mov`) for processing.

- **Preprocessing:**  
  - Converts each frame from BGR to HSV.
  - Applies Gaussian blur to reduce noise.

- **Thresholding:**  
  - Defines an HSV range (with `lower_fire` and `upper_fire`) to isolate low-saturation, very bright fire-like regions.
  - Creates a binary mask for these regions.

- **Morphological Operations:**  
  Cleans the mask using opening and closing to remove noise.

- **Contour Detection:**  
  - Extracts contours from the mask.
  - For contours with an area greater than 150 pixels:
    - Calculates the bounding box, center coordinates, and the brightest point (using the V channel).
    - Draws these details on the frame and prints them to the console.

### 5. Stepper Motor Control for Pitch Adjustment

- **Purpose:**  
  Adjust the pitch of the water nozzle (or camera) based on the vertical position of the detected fire.
  
- **Implementation:**  
  - Computes the average y-coordinate of the detected fire regions.
  - Compares it to the frame’s vertical center.
  - If the fire is above the center (beyond a small tolerance), the stepper motor steps forward (pitch up). If below, it steps backward (pitch down).

### 6. DC Motor (Wheel) Control for Chassis Spinning

- **Purpose:**  
  To pivot the robot so that water can be spread more efficiently over the fire.
  
- **Implementation:**  
  - If fire is detected and a minimum interval (e.g., 5 seconds) has passed, the `spin_wheels_sequence()` function is called.
  - This function activates each wheel sequentially, causing a pivoting motion.

### 7. Water Pump Control (Activation Only if Fire is Stable)

- **Stability Tracking:**  
  - Computes the average fire center (both x and y coordinates) from the detected regions.
  - Uses a pixel tolerance (20 pixels) to determine if the fire’s position is stable between frames.
  - If no previous center is recorded, the current center and time are saved.

- **Stability Check and Activation:**  
  - If the fire remains within the tolerance for at least 4 seconds, and if a cooldown period (10 seconds) has passed since the last activation, the water pump is activated for 2 seconds at full throttle.
  - If the fire moves significantly, the stability timer resets.

### Main Loop and Cleanup

- **Frame Processing:**  
  The script processes each video frame, running fire detection, adjusting the pitch, pivoting the chassis, and managing the water pump activation.

- **Display and Exit:**  
  The processed frame is shown in a window, and the loop continues until the user presses `q`.

- **Cleanup:**  
  Once the loop exits, the video capture is released and all OpenCV windows are closed.

---

## How to Use

1. **Hardware Setup:**  
   - Connect the primary MotorKit for wheels and the stepper motor to the default I²C address.
   - Connect the water pump HAT to I²C address `0xA0` (adjust if necessary).
   - Place the video file `fire7.mov` in the `./software/` directory (or update the path in the code).

2. **Install Dependencies:**  
   ```bash
   pip install opencv-python numpy adafruit-circuitpython-motorkit adafruit-circuitpython-motor
