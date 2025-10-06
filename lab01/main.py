#!/usr/bin/env pybricks-micropython
"""
EV3 MicroPython program for a three-part robotics challenge.
- Objective 1: Drive forward a set distance (Odometry).
- Objective 2: Drive forward and stop a set distance from a wall (Ranging).
- Objective 3: Drive forward, bump the wall, and reverse to the set distance (Bumping).

This code is refactored to comply with the assignment's module restrictions,
explicitly avoiding the 'pybricks.robotics' module.
"""

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait

# ==============================================================================
# region: --- CONFIGURATION ---
# You should only need to modify the values in this section.
# ==============================================================================

# -- Port Assignments
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_SENSOR_PORT = Port.S1
ULTRASONIC_SENSOR_PORT = Port.S4

# -- Robot Physical Parameters
WHEEL_DIAMETER_MM = 56.0  # Diameter of the driving wheels in millimeters.
AXLE_TRACK_MM = 114.0     # Distance between the centers of the two wheels.

# -- Movement Speeds
CRUISE_SPEED_MM_S = 220   # Speed for driving straight in Objective 1 (mm/s).
APPROACH_SPEED_MM_S = 140   # Speed for approaching and reversing from the wall (mm/s).

# -- Objective Parameters
# The distance from the nearest point on the robot to the wall.
TARGET_DISTANCE_FROM_WALL_MM = 400

# The distance from the ultrasonic sensor to the front-most point of the robot (e.g., the touch sensor).
# If the ultrasonic sensor is recessed behind the front bumper, enter a positive value.
SENSOR_TO_FRONT_OFFSET_MM = 30

# -- Sensor Polling and Tolerance
DISTANCE_TOLERANCE_MM = 10      # Allowed error margin for distance measurements.
SENSOR_POLL_INTERVAL_MS = 30    # How often to check the sensor values (milliseconds).
# endregion

# ==============================================================================
# region: --- INITIALIZATION ---
# ==============================================================================

# --- Initialize Hardware
ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
touch_sensor = TouchSensor(TOUCH_SENSOR_PORT)
ultrasonic_sensor = UltrasonicSensor(ULTRASONIC_SENSOR_PORT)

# --- Pre-calculate Constants
# The target distance for the ultrasonic sensor to read, accounting for its offset.
TARGET_SENSOR_DISTANCE_MM = TARGET_DISTANCE_FROM_WALL_MM - SENSOR_TO_FRONT_OFFSET_MM
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM
# endregion

# ==============================================================================
# region: --- LOW-LEVEL ROBOT CONTROL FUNCTIONS (DriveBase Replacement) ---
# ==============================================================================

def drive_straight(distance_mm: int, speed_mm_s: int):
    """
    Drives the robot straight for a given distance.
    This function replaces `DriveBase.straight()`.

    Args:
        distance_mm: The distance to travel in millimeters.
        speed_mm_s: The speed at which to travel in mm/s.
    """
    # Calculate the required rotation angle for the wheels.
    angle = (distance_mm / WHEEL_CIRCUMFERENCE_MM) * 360
    speed_deg_s = (speed_mm_s / WHEEL_CIRCUMFERENCE_MM) * 360

    # Run both motors to the target angle.
    # The `wait=False` on the first motor allows both to start simultaneously.
    # The `wait=True` on the second motor makes the function block until the move is complete.
    left_motor.run_target(speed_deg_s, angle, then=Stop.BRAKE, wait=False)
    right_motor.run_target(speed_deg_s, angle, then=Stop.BRAKE, wait=True)

def start_driving(speed_mm_s: int):
    """
    Starts driving the robot straight indefinitely.
    This function replaces `DriveBase.drive(speed, 0)`.

    Args:
        speed_mm_s: The speed at which to travel in mm/s.
    """
    speed_deg_s = (speed_mm_s / WHEEL_CIRCUMFERENCE_MM) * 360
    left_motor.run(speed_deg_s)
    right_motor.run(speed_deg_s)

def stop_driving():
    """
    Stops both motors with a brake.
    This function replaces `DriveBase.stop()`.
    """
    left_motor.brake()
    right_motor.brake()

# endregion

# ==============================================================================
# region: --- UTILITY AND OBJECTIVE FUNCTIONS ---
# ==============================================================================

def wait_for_center_button_press(prompt_message: str):
    """
    Displays a message and waits for the center EV3 button to be pressed and released.
    Includes debouncing logic and provides auditory feedback.
    """
    ev3.screen.clear()
    ev3.screen.print(prompt_message)
    # Wait for any current press to be released.
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    # Wait for a new press.
    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)
    # Wait for the new press to be released (debouncing).
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    ev3.speaker.beep()

def drive_to_distance(target_sensor_mm: int, speed_mm_s: int):
    """
    Drives forward or backward until the ultrasonic sensor reaches the target distance.
    Includes logic to handle sensor reading fluctuations.

    Args:
        target_sensor_mm: The target distance for the sensor in millimeters.
        speed_mm_s: The speed of movement. Positive for forward, negative for backward.
    """
    is_driving_forward = speed_mm_s > 0
    
    consecutive_hits = 0
    required_hits = 3  # Number of consecutive valid readings needed to stop.

    start_driving(speed_mm_s)

    while True:
        current_distance = ultrasonic_sensor.distance()
        
        # Determine if the robot has reached or passed the target distance.
        is_target_reached = False
        if is_driving_forward:
            if current_distance <= target_sensor_mm + DISTANCE_TOLERANCE_MM:
                is_target_reached = True
        else: # Driving backward
            if current_distance >= target_sensor_mm - DISTANCE_TOLERANCE_MM:
                is_target_reached = True

        # Check for consecutive hits to filter out sensor noise.
        if is_target_reached:
            consecutive_hits += 1
        else:
            consecutive_hits = 0

        if consecutive_hits >= required_hits:
            break

        # Optional: Display debug info on the screen
        ev3.screen.clear()
        # --- FIXED LINES HERE ---
        ev3.screen.print("Dist(mm): {}".format(current_distance))
        ev3.screen.print("Target: {}".format(target_sensor_mm))
        # ------------------------
        wait(SENSOR_POLL_INTERVAL_MS)

    stop_driving()
    
def drive_until_bump(speed_mm_s: int):
    """
    Drives forward until the touch sensor is pressed.

    Args:
        speed_mm_s: The forward speed in mm/s.
    """
    start_driving(speed_mm_s)
    while not touch_sensor.pressed():
        wait(SENSOR_POLL_INTERVAL_MS)
    stop_driving()

# endregion

# ==============================================================================
# region: --- MAIN PROGRAM ---
# ==============================================================================

def main():
    """
    Executes the three objectives of the challenge in sequence.
    """
    ev3.speaker.beep() # Signal that the program is ready.

    # --- Objective 1: Drive 1.4m (Odometry) ---
    wait_for_center_button_press("Obj 1: Go 1.4m")
    drive_straight(1400, CRUISE_SPEED_MM_S)
    ev3.speaker.beep(frequency=1000, duration=200)

    # --- Objective 2: Approach Wall (Ranging) ---
    wait_for_center_button_press("Obj 2: Approach wall")
    drive_to_distance(TARGET_SENSOR_DISTANCE_MM, APPROACH_SPEED_MM_S)
    ev3.speaker.beep(frequency=1200, duration=200)

    # --- Objective 3: Bump and Reverse ---
    wait_for_center_button_press("Obj 3: Bump & back up")
    # Bump the wall
    drive_until_bump(APPROACH_SPEED_MM_S)
    ev3.speaker.beep(frequency=800, duration=150)
    # Reverse to the target distance
    drive_to_distance(TARGET_SENSOR_DISTANCE_MM, -APPROACH_SPEED_MM_S) # Negative speed for reverse
    
    # --- Completion ---
    stop_driving()
    ev3.speaker.beep(frequency=600, duration=300)
    ev3.screen.clear()
    ev3.screen.print("Challenge complete.")

if __name__ == "__main__":
    main()
# endregion