#!/usr/bin/env pybricks-micropython
"""
EV3 MicroPython program for a three-part robotics challenge (Odometry, Ranging, Bumping).
"""

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait

# ==============================================================================
# region: --- CONFIGURATION ---
# ==============================================================================

# -- Port Assignments
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_SENSOR_PORT = Port.S1
ULTRASONIC_SENSOR_PORT = Port.S4

# -- Robot Physical Parameters
WHEEL_DIAMETER_MM = 56.0  # Wheel diameter (mm)
AXLE_TRACK_MM = 114.0     # Distance between wheels (mm)

# -- Movement Speeds
CRUISE_SPEED_MM_S = 220   # Speed for Obj 1 (mm/s)
APPROACH_SPEED_MM_S = 140   # Speed for Obj 2 & 3 (mm/s)

# -- Objective Parameters
TARGET_DISTANCE_FROM_WALL_MM = 400
# Offset from sensor to the front of the robot (mm).
# Use a positive value if the sensor is recessed behind the bumper.
SENSOR_TO_FRONT_OFFSET_MM = 30

# -- Sensor Polling and Tolerance
DISTANCE_TOLERANCE_MM = 10      # Allowed error margin for distance sensor (mm)
SENSOR_POLL_INTERVAL_MS = 30    # How often to check sensors (ms)
# endregion

# ==============================================================================
# region: --- INITIALIZATION ---
# ==============================================================================

ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
touch_sensor = TouchSensor(TOUCH_SENSOR_PORT)
ultrasonic_sensor = UltrasonicSensor(ULTRASONIC_SENSOR_PORT)

# Calculate the target distance for the sensor to read, accounting for its offset.
TARGET_SENSOR_DISTANCE_MM = TARGET_DISTANCE_FROM_WALL_MM - SENSOR_TO_FRONT_OFFSET_MM
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM
# endregion

# ==============================================================================
# region: --- LOW-LEVEL ROBOT CONTROL FUNCTIONS ---
# ==============================================================================

def drive_straight(distance_mm: int, speed_mm_s: int):
    """Drives the robot straight for a given distance."""
    # Convert distance and speed to motor angle and rotational speed.
    angle = (distance_mm / WHEEL_CIRCUMFERENCE_MM) * 360
    speed_deg_s = (speed_mm_s / WHEEL_CIRCUMFERENCE_MM) * 360

    # Start both motors; wait=False runs them in parallel.
    # The final wait=True blocks until the movement is complete.
    left_motor.run_target(speed_deg_s, angle, then=Stop.BRAKE, wait=False)
    right_motor.run_target(speed_deg_s, angle, then=Stop.BRAKE, wait=True)

def start_driving(speed_mm_s: int):
    """Starts the robot driving straight indefinitely."""
    speed_deg_s = (speed_mm_s / WHEEL_CIRCUMFERENCE_MM) * 360
    left_motor.run(speed_deg_s)
    right_motor.run(speed_deg_s)

def stop_driving():
    """Stops both motors with a brake."""
    left_motor.brake()
    right_motor.brake()

# endregion

# ==============================================================================
# region: --- UTILITY AND OBJECTIVE FUNCTIONS ---
# ==============================================================================

def wait_for_center_button_press(prompt_message: str):
    """Displays a message and waits for the center button to be pressed and released."""
    ev3.screen.clear()
    ev3.screen.print(prompt_message)
    
    # Debouncing logic: wait for release, then press, then release again.
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    ev3.speaker.beep()

def drive_to_distance(target_sensor_mm: int, speed_mm_s: int):
    """Drives until the ultrasonic sensor measures the target distance."""
    is_driving_forward = speed_mm_s > 0
    consecutive_hits = 0
    required_hits = 3  # Require 3 consecutive valid readings to filter out noise.

    start_driving(speed_mm_s)

    while True:
        current_distance = ultrasonic_sensor.distance()
        
        # Check if the target distance has been reached.
        is_target_reached = False
        if is_driving_forward:
            if current_distance <= target_sensor_mm + DISTANCE_TOLERANCE_MM:
                is_target_reached = True
        else: # Driving backward
            if current_distance >= target_sensor_mm - DISTANCE_TOLERANCE_MM:
                is_target_reached = True

        if is_target_reached:
            consecutive_hits += 1
        else:
            consecutive_hits = 0

        if consecutive_hits >= required_hits:
            break

        # Display debug info on the screen.
        ev3.screen.clear()
        ev3.screen.print("Dist(mm): {}".format(current_distance))
        ev3.screen.print("Target: {}".format(target_sensor_mm))
        wait(SENSOR_POLL_INTERVAL_MS)

    stop_driving()
    
def drive_until_bump(speed_mm_s: int):
    """Drives forward until the touch sensor is pressed."""
    start_driving(speed_mm_s)
    while not touch_sensor.pressed():
        wait(SENSOR_POLL_INTERVAL_MS)
    stop_driving()

# endregion

# ==============================================================================
# region: --- MAIN PROGRAM ---
# ==============================================================================

def main():
    """Executes the three objectives in sequence."""
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
    drive_until_bump(APPROACH_SPEED_MM_S)
    ev3.speaker.beep(frequency=800, duration=150)
    drive_to_distance(TARGET_SENSOR_DISTANCE_MM, -APPROACH_SPEED_MM_S) # Negative speed for reverse
    
    # --- Completion ---
    stop_driving()
    ev3.speaker.beep(frequency=600, duration=300)
    ev3.screen.clear()
    ev3.screen.print("Challenge complete.")

if __name__ == "__main__":
    main()
# endregion