"""
ELEC4020 - Embedded System Final Project: Human Arm Motion Recognition and Communication
Team Members: Chau Hoang Phuc, Nguyen Ngoc Phuc Tien, Vu Duc Duy
Cohort: VinUniversity - Electrical Engineering - Cohort 3

Description:
This Python script processes data from sensors to recognize human arm motions and sends 
the processed data via UART to the TIVA board. The data format is packaged as:
    [elbow_angle, wrist_angle, thumb_index_distance, base_angle]

Key Features:
1. Detects elbow angle, wrist angle, distance between thumb and index finger, and base angle.
2. Packages the data into the specified format.
3. Sends the packaged data to the TIVA board for further processing.

INSTRUCTION:

* Press 'T' to show the line of the wrist angle
* Press 'S' & 'D' to change the base angle
* Press 'P' to start/stop sending data to UART
* Press 'Q' to quit the program

"Click the program window to make sure those functions work."


WIRING:
* Servo 1 - Grabber: White ---> PD0
* Servo 2 - Wrist: Yellow ---> PD1
* Servo 3 - Elbow: Green ---> PB4
* Servo 4 - Base: Black ---> PB5
* GND - Yellow ---> GND on Tiva
"""


import numpy as np
import cv2
import mediapipe as mp
import math
import serial

""" 
====================================================
Kalman for points on camera
====================================================
"""
#Initialize Kalman Filter

kalman = cv2.KalmanFilter(4, 2)  # State: [x, y, dx, dy], Measurement: [x, y]

# State Transition Matrix (4x4)
kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)

# Measurement Matrix (2x4)
kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)

# Process Noise Covariance (4x4)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05

# Measurement Noise Covariance (2x2)
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.4

# Error Covariance Post (4x4)
kalman.errorCovPost = np.eye(4, dtype=np.float32) * 0.1

# Initial State
kalman.statePost = np.zeros((4, 1), dtype=np.float32)

""" 
====================================================
"""



""" 
====================================================
Kalman for output angles
====================================================
"""
# Kalman Filter for Angle Smoothing
kalman_angle = cv2.KalmanFilter(2, 1)  # State: [angle, angular_velocity], Measurement: [measured_angle]

# State Transition Matrix (2x2)
kalman_angle.transitionMatrix = np.array([[1, 1],  # angle' = angle + angular_velocity
                                          [0, 1]], np.float32)  # angular_velocity' = angular_velocity

# Measurement Matrix (1x2)
kalman_angle.measurementMatrix = np.array([[1, 0]], np.float32)  # Map angle to measurement

# Process Noise Covariance (2x2)
kalman_angle.processNoiseCov = np.array([[1e-2, 0],
                                         [0, 1e-2]], np.float32)

# Measurement Noise Covariance (1x1)
kalman_angle.measurementNoiseCov = np.array([[0.009]], np.float32)

# Error Covariance Post (2x2)
kalman_angle.errorCovPost = np.array([[1, 0],
                                      [0, 1]], np.float32)

# Initial State
kalman_angle.statePost = np.zeros((2, 1), dtype=np.float32)

""" 
====================================================
"""

angle_elbow_wrist =0
angle_ABC =0
send_flag = False
output  = 0
angle_elbow =0
distance_thumb_index =0
filtered_real_angles = [0, 0, 0, 0]
base_angle = 135
base_add = 0
draw = False
send_t = 0
pulse_out = [0, 0, 0, 0]
previous_pulse = [0, 0, 0, 0]
previous_angle = [0, 0, 0, 0]
wrist_coords =(0, 0)

    # --- CONSTANTS ---
alpha_angles = [0.2, 0.01, 0.08]  # EMA alpha values for Grabber, Wrist, Elbow
alpha_pulses = [0.5, 0.05, 0.085, 0.1]  # EMA for Pulse Outputs
angle_jump_thresholds = [15, 15, 15, 10]  # Max angle jump thresholds to prevent sudden changes


# Exponential Moving Average for Angle Smoothing
def smooth_with_ema(current_value, previous_value, alpha=0.2):
    """
    Smooth a value using Exponential Moving Average (EMA).

    Parameters:
    - current_value (float): The current angle value.
    - previous_value (float): The previous smoothed angle value.
    - alpha (float): Smoothing factor (0 to 1). Higher = more responsive, Lower = smoother.

    Returns:
    - float: Smoothed angle value.
    """
    return alpha * current_value + (1 - alpha) * previous_value

def smooth_angle_with_kalman(angle):
    """
    Smooth an angle using the Kalman Filter.

    Parameters:
    - angle (float): The current angle in degrees.

    Returns:
    - float: Smoothed angle in degrees.
    """
    # Ensure the angle is within the range [0, 360)
    # angle = angle % 360
    
    # Kalman Filter Correction
    measurement = np.array([[np.float32(angle)]])
    kalman_angle.correct(measurement)
    
    # Kalman Filter Prediction
    predicted = kalman_angle.predict()
    smoothed_angle = predicted[0][0] # Ensure output wraps around [0, 360)
    
    return int(smoothed_angle)

def smooth_with_kalman(point):
    """
    Smooth a point using the Kalman Filter.

    Parameters:
    - point (tuple): The current (x, y) point.

    Returns:
    - tuple: Smoothed (x, y) point.
    """
    # Prepare measurement
    measurement = np.array([[np.float32(point[0])], [np.float32(point[1])]])
    
    # Correct the Kalman Filter with the measurement
    kalman.correct(measurement)
    
    # Predict the next state
    predicted = kalman.predict()
    
    # Extract predicted x and y
    x, y = predicted[0], predicted[1]
    
    return (int(x), int(y))

# Function to calculate angle with horizontal
def calculate_angle(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y1 - y2  # Reverse y-axis since OpenCV's origin is top-left
    angle = math.degrees(math.atan2(delta_y, delta_x))  # atan2 for accurate angle
    return angle if angle >= 0 else angle + 360  # Normalize angle to [0, 360]

# Function to calculate distance between two points
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Function to compute the average of the last 3 values
def compute_averaged_angles(buffer):
    return np.mean(buffer[-3:], axis=0)

def calculate_angle_between_vectors(pointA, pointB, pointC):
    """
    Calculate the angle between two vectors AB and AC.

    Parameters:
    - pointA (tuple): Coordinates of point A (x, y)
    - pointB (tuple): Coordinates of point B (x, y)
    - pointC (tuple): Coordinates of point C (x, y)

    Returns:
    - float: Angle between AB and AC in degrees.
    """
    # Calculate vectors AB and AC
    AB = (pointB[0] - pointA[0], pointB[1] - pointA[1])
    AC = (pointC[0] - pointA[0], pointC[1] - pointA[1])
    
    # Calculate dot product and magnitudes
    dot_product = AB[0] * AC[0] + AB[1] * AC[1]
    magnitude_AB = math.sqrt(AB[0]**2 + AB[1]**2)
    magnitude_AC = math.sqrt(AC[0]**2 + AC[1]**2)
    
    # Avoid division by zero
    if magnitude_AB == 0 or magnitude_AC == 0:
        return 0.0
    
    # Calculate the angle (in radians)
    angle_rad = math.acos(dot_product / (magnitude_AB * magnitude_AC))
    
    # Convert to degrees
    angle_deg = math.degrees(angle_rad)
    return angle_deg

def calculate_signed_angle_between_lines(pointA, pointB, pointC, reverse=False, rotate = 0):
    """
    Calculate the signed angle between two lines AB and AC, with an option to reverse rotation.

    Parameters:
    - pointA (tuple): Coordinates of point A (x, y)
    - pointB (tuple): Coordinates of point B (x, y)
    - pointC (tuple): Coordinates of point C (x, y)
    - reverse (bool): If True, reverse the sign of the angle.

    Returns:
    - float: Signed angle between AB and AC in degrees.
    """
    # Calculate vectors AB and AC
    AB = (pointB[0] - pointA[0], pointB[1] - pointA[1])
    AC = (pointC[0] - pointA[0], pointC[1] - pointA[1])
    
    # Calculate the cross product (to determine sign)
    cross_product = (AB[0] * AC[1]) - (AB[1] * AC[0])
    
    # Calculate the dot product (to determine angle)
    dot_product = (AB[0] * AC[0]) + (AB[1] * AC[1])
    
    # Calculate the angle in radians and convert to degrees
    angle_rad = math.atan2(cross_product, dot_product)
    angle_deg = math.degrees(angle_rad)
    
    # Reverse the angle if the reverse flag is set
    if reverse:
        angle_deg *= -1

    if rotate and angle_deg < 0:
        angle_deg = 180 + (180+angle_deg)
    
    return angle_deg

def send_angles_to_tiva(current_angles):
    # Convert angles to a comma-separated string with floats
   
    data_frame = f"[{int(current_angles[0])},{int(current_angles[1])},{int(current_angles[2])},{int(current_angles[3])}]"
    # uart.write(data_frame.encode())  # Send data over UART

    for char in data_frame:
        uart.write(char.encode())  # Send the ASCII character
                # Short delay so you can see step-by-step sending (can remove or reduce if too slow)
        # time.sleep(0.015)
            # Optional: small delay between frames so your device has time to parse
            # time.sleep(0.01)
        print(f"Sent all characters to Tiva: {data_frame}")

def map_range(value, in_min=10, in_max=100, out_min=0, out_max=270, reverse=0):
    """
    Maps a value from one range to another, with an option to reverse the output range.

    Parameters:
    - value (float): The input value to map.
    - in_min (float): Minimum value of the input range.
    - in_max (float): Maximum value of the input range.
    - out_min (float): Minimum value of the output range.
    - out_max (float): Maximum value of the output range.
    - reverse (int): 0 for normal mapping, 1 for reverse mapping.

    Returns:
    - float: Mapped value in the output range.
    """
    if value == 0:
        return value


    if value >= in_max:
        value = in_max

    if value <= in_min:
        value = in_min

    if reverse == 1:
        # Reverse Mapping
        return (value - in_min) * (out_min - out_max) / (in_max - in_min) + out_max
    else:
        # Normal Mapping
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def map_angle(value, in_min, in_max, out_min, out_max):
    """
    Maps a value from one range to another.

    Parameters:
    - value (float): Input angle.
    - in_min (float): Minimum of the input range.
    - in_max (float): Maximum of the input range.
    - out_min (float): Minimum of the output range.
    - out_max (float): Maximum of the output range.

    Returns:
    - float: Mapped angle.
    """
    # Clamp the value to the input range
    value = max(min(value, in_max), in_min)
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def point_to_line_distance(point, line_point1, line_point2, sign=False):
    """
    Calculate the perpendicular distance from a point to a line defined by two points,
    with an optional signed output.

    Parameters:
    - point (tuple): Coordinates of the point (x0, y0).
    - line_point1 (tuple): First point on the line (x1, y1).
    - line_point2 (tuple): Second point on the line (x2, y2).
    - sign (bool): Whether to return the signed distance (True) or absolute distance (False).

    Returns:
    - float: The perpendicular distance from the point to the line (signed or absolute).
    """
    x0, y0 = point
    x1, y1 = line_point1
    x2, y2 = line_point2
    
    # Numerator using the signed distance formula
    numerator = (y2 - y1) * (x0 - x1) - (x2 - x1) * (y0 - y1)
    
    # Denominator is the length of the line segment
    denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    
    if denominator == 0:
        raise ValueError("The two points defining the line must not be identical.")
    
    # Calculate distance
    distance = numerator / denominator
    
    if not sign:
        return abs(distance)  # Return absolute distance if sign flag is False
    
    return distance  # Return signed distance if sign flag is True


# Mediapipe initialization
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.4)
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Open webcam
cap = cv2.VideoCapture(0)

current_angles = np.zeros(4)  # Starting current angles

# Initialize PID controllers for each axis

angle_wrist =0

# Simulated testing loop
t = 0  # Frame counter
uart = serial.Serial(port='COM5', baudrate=115200, timeout=1)  # Use your Tiva's COM port

while cap.isOpened():

    # uart = serial.Serial(port='COM5', baudrate=9600, timeout=1)  # Use your Tiva's COM port
    # uart.close()
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)

   
    if not ret:
        print("Failed to capture frame. Exiting.")
        break

    # Convert BGR to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process Pose and Hands
    pose_results = pose.process(frame_rgb)
    hand_results = hands.process(frame_rgb)

    # Get frame dimensions
    h, w, _ = frame.shape

    real_time_angles = [0, 0, 0]  # Placeholder for angles

    wrist_extend_circle = (0, 0)
    index_meta_coords =(0, 0)
    # raw_wrist_coords =(0, 0)


    if pose_results.pose_landmarks:
        landmarks = pose_results.pose_landmarks.landmark

        # Extract keypoints for the right arm (shoulder, elbow, wrist)
        shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
        elbow = landmarks[mp_pose.PoseLandmark.LEFT_ELBOW]
        wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]

        # Convert to pixel coordinates
        shoulder_coords = (int(shoulder.x * w), int(shoulder.y * h))
        elbow_coords = (int(elbow.x * w), int(elbow.y * h))
        raw_wrist_coords = (int(wrist.x * w), int(wrist.y * h))

        wrist_coords = smooth_with_kalman(raw_wrist_coords)


        # 🛠️ Extend the wrist beyond its original position
        # Calculate direction vector (elbow → wrist)
        wrist_direction = (wrist_coords[0] - elbow_coords[0], wrist_coords[1] - elbow_coords[1])
        
        # Calculate the extended wrist point
        extension_factor = 1.1  # Scale this factor for longer or shorter extension
        extension_factor_circle = 0.5  # Scale this factor for longer or shorter extension

        wrist_extend = (
            int(wrist_coords[0] + wrist_direction[0] * extension_factor),
            int(wrist_coords[1] + wrist_direction[1] * extension_factor)
        )

        wrist_extend_circle = (
            int(wrist_coords[0] + wrist_direction[0] * extension_factor_circle),
            int(wrist_coords[1] + wrist_direction[1] * extension_factor_circle)
        )


        # 🧠 Calculate arm angles
        angle_shoulder_elbow = calculate_angle(*shoulder_coords, *elbow_coords)
        angle_elbow_wrist = calculate_angle(*elbow_coords, *wrist_coords)

        # 🎨 Draw connections (shoulder → elbow → wrist → extended wrist)
        cv2.line(frame, shoulder_coords, elbow_coords, (0, 255, 0), 3)  # Shoulder → Elbow
        cv2.line(frame, elbow_coords, wrist_coords, (0, 255, 0), 3)     # Elbow → Wrist
        # cv2.line(frame, wrist_coords, wrist_extend, (0, 0, 255), 3)     # Wrist → Extended Wrist


        # 🟢 Draw keypoints
        cv2.circle(frame, shoulder_coords, 5, (255, 0, 0), -1)  # Shoulder
        cv2.circle(frame, elbow_coords, 5, (255, 0, 0), -1)     # Elbow
        cv2.circle(frame, wrist_coords, 5, (255, 0, 0), -1)     # Wrist
        # cv2.circle(frame, wrist_extend, 5, (0, 0, 255), -1)     # Extended Wrist
        # cv2.circle(frame, wrist_extend_circle, 5, (0, 255, 255), -1)     # Extended Wrist Circle



    if hand_results.multi_hand_landmarks:
        hand_landmarks = hand_results.multi_hand_landmarks[0]
        
        # 🖐️ Thumb Landmarks
        thumb_tip = hand_landmarks.landmark[4]
        thumb_middle = hand_landmarks.landmark[3]
        thumb_wrist = hand_landmarks.landmark[0]
        
        # 🖐️ Index Finger Landmarks
        index_meta = hand_landmarks.landmark[5]  # MCP Joint
        index_tip = hand_landmarks.landmark[8]   # Tip
        
        # Convert Thumb landmarks to pixel coordinates
        thumb_tip_coords = (int(thumb_tip.x * w), int(thumb_tip.y * h))
        thumb_middle_coords = (int(thumb_middle.x * w), int(thumb_middle.y * h))
        thumb_wrist_coords = (int(thumb_wrist.x * w), int(thumb_wrist.y * h))
        
        # Convert Index Finger landmarks to pixel coordinates
        index_meta_coords = (int(index_meta.x * w), int(index_meta.y * h))
        index_tip_coords = (int(index_tip.x * w), int(index_tip.y * h))
        
        # 🟡 Thumb Drawing
 
        cv2.circle(frame, thumb_tip_coords, 5, (0, 0, 255), -1)  # Red circle on Thumb Tip
        
        # 🟢 Index Finger Drawing
        cv2.line(frame, index_meta_coords, wrist_coords, (0, 0, 255), 3)     # Wrist → Extended Wrist


        cv2.circle(frame, index_tip_coords, 5, (0, 0, 255), -1)  # Red circle on Index Tip
        cv2.circle(frame, index_meta_coords, 8, (255, 0, 0), -1)  # Blue circle on Index MCP (for later use)
        
        # 🔄 Angle and Distance Calculations
        angle_thumb_index = calculate_angle(thumb_tip_coords[0], thumb_tip_coords[1],
                                            index_tip_coords[0], index_tip_coords[1])
        distance_thumb_index = calculate_distance(*thumb_tip_coords, *index_tip_coords)
        

        distance_index_meta_wrist = point_to_line_distance(index_meta_coords, wrist_coords, wrist_extend, True)
    
    # Define the three points
        pointA = wrist_coords       # A
        pointB = index_meta_coords  # B
        pointC = wrist_extend_circle  # C
        angle_ABC = calculate_signed_angle_between_lines(pointA, pointB, pointC, True)

        angle_elbow = calculate_signed_angle_between_lines(elbow_coords, shoulder_coords, wrist_coords, rotate=1)
        


        real_time_angles[2] = angle_elbow
        real_time_angles[1] = distance_index_meta_wrist      #wrist
        real_time_angles[0] = distance_thumb_index

        # Calculate the angle

    key = cv2.waitKey(1) & 0xFF
    if base_angle >=270:
        base_angle = 270
    elif base_angle <= 1:
        base_angle = 1

    if base_add >=7:
        base_add = 7
    elif base_add <= 0:
        base_add = 0



    if key == ord('d'):
        base_add +=1
        base_angle += 15*base_add

    if key == ord('s'):
        base_add -=1
        base_angle -= 15*abs(base_add)

        
    if key == ord('t'):
        draw = not draw
    if draw:
        cv2.line(frame, wrist_coords, wrist_extend, (0, 0, 255), 3)     # Wrist → Extended Wrist
        cv2.circle(frame, wrist_extend, 5, (0, 0, 255), -1)     # Extended Wrist
        cv2.circle(frame, wrist_extend_circle, 5, (0, 255, 255), -1)     # Extended Wrist Circle
        cv2.line(frame, wrist_extend_circle, index_meta_coords, (0, 0, 255), 2)


# Map filtered_real_angles to current_angles
    current_angles[0] = map_range(real_time_angles[0], in_max=70, in_min=30, reverse=1)  # Grabber
    current_angles[1] = map_range(real_time_angles[1], in_min=-45, in_max=65, reverse =1)  # Wrist
    current_angles[2] = map_range(real_time_angles[2], in_min=30, in_max=170, out_max=270)  # Elbow
    current_angles[3] = base_angle  # Base



# --- SMOOTH ANGLES WITH KALMAN FILTER ---
    for i in range(3):
        filtered_real_angles[i] = smooth_angle_with_kalman(current_angles[i])

# --- APPLY ANGLE JUMP THRESHOLD FILTER ---
    for i in range(3):
        if abs(filtered_real_angles[i] - previous_angle[i]) <= angle_jump_thresholds[i]:
            filtered_real_angles[i] = previous_angle[i]
        previous_angle[i] = filtered_real_angles[i]

# --- MAP ANGLES TO PULSES ---
    pulse_out[0] = map_range(filtered_real_angles[0], in_min=70, in_max=270, out_min=625, out_max=1070)
    pulse_out[1] = map_range(filtered_real_angles[1], in_min=70, in_max=270, out_min=300, out_max=1625)
    pulse_out[2] = map_range(filtered_real_angles[2], in_min=70, in_max=260, out_min=400, out_max=1625)
    pulse_out[3] = map_range(base_angle, in_min = 1, in_max= 270, out_min = 225, out_max = 1625)

    # --- APPLY EMA TO PULSE OUTPUTS ---
    for i in range(4):
        pulse_out[i] = smooth_with_ema(pulse_out[i], previous_pulse[i], alpha=alpha_pulses[i])
        previous_pulse[i] = pulse_out[i]


    
    # Display the frame
    cv2.imshow('Right Arm and Hand Detection', frame)

    # Increment frame counter
    t += 1




    if key == ord('q'):  # Quit when 'q' is pressed
        print("Program terminated manually.")
        uart.close()
        break


    if key == ord('p'):  # Toggle send_flag when 'P' is pressed
        send_flag = not send_flag

    if send_flag:
        send_angles_to_tiva(pulse_out)
    else:
  
        # data_frame = f"[{int(pulse_out[0])},{int(pulse_out[1])},{int(pulse_out[2])},{int(pulse_out[3])}]" \\FOR DEBUGGING PURPOSE
        data_to_user = f"[{int(current_angles[0])},{int(current_angles[1])},{int(current_angles[2])},{int(current_angles[3])}]"

        print(f"Current angles: {data_to_user}")

cap.release()
cv2.destroyAllWindows()