#!/usr/bin/env pybricks-micropython

# Authors: Lianrui Geng && Xinyi Guo
# Course:  UNC COMP 581
# Lab:     Lab 01
# Date:    October 6, 2025

"""
EV3 MicroPython program for a three-part robotics lab.
This version uses continuous motor running and the hold() command for a decisive, synchronous stop.
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
LEFT_TOUCH_SENSOR_PORT = Port.S1
RIGHT_TOUCH_SENSOR_PORT = Port.S3
ULTRASONIC_SENSOR_PORT = Port.S4

# -- Robot Physical Parameters
WHEEL_DIAMETER_MM = 56.0
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM

# -- Movement Speeds
CRUISE_SPEED_MM_S = 200
APPROACH_SPEED_MM_S = 100

# -- Objective Parameters
TARGET_DISTANCE_FROM_WALL_MM = 400
SENSOR_TO_FRONT_OFFSET_MM = 30
TARGET_SENSOR_DISTANCE_MM = TARGET_DISTANCE_FROM_WALL_MM - SENSOR_TO_FRONT_OFFSET_MM

# endregion

# ==============================================================================
# region: --- INITIALIZATION ---
# ==============================================================================

ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
left_touch_sensor = TouchSensor(LEFT_TOUCH_SENSOR_PORT)
right_touch_sensor = TouchSensor(RIGHT_TOUCH_SENSOR_PORT)
ultrasonic_sensor = UltrasonicSensor(ULTRASONIC_SENSOR_PORT)

# endregion

# ==============================================================================
# region: --- HELPER FUNCTIONS ---
# ==============================================================================

def drive_straight(distance_mm: int, speed_mm_s: int):
    """Drives the robot straight for a specific distance. Used for Objective 1."""
    angle = (distance_mm / WHEEL_CIRCUMFERENCE_MM) * 360
    speed_deg_s = (speed_mm_s / WHEEL_CIRCUMFERENCE_MM) * 360
    left_motor.run_target(speed_deg_s, angle, then=Stop.BRAKE, wait=False)
    right_motor.run_target(speed_deg_s, angle, then=Stop.BRAKE, wait=True)

def wait_for_center_button_press(prompt_message: str):
    """Displays a message and waits for the center button to be pressed and released."""
    ev3.screen.clear()
    ev3.screen.print(prompt_message)
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    ev3.speaker.beep()

# endregion

# ==============================================================================
# region: --- MAIN PROGRAM ---
# ==============================================================================

def main():
    """Executes the three objectives in sequence."""
    ev3.speaker.beep()
    
    # Convert approach speed to degrees per second
    approach_speed_deg_s = (APPROACH_SPEED_MM_S / WHEEL_CIRCUMFERENCE_MM) * 360

    # --- Objective 1: Drive 1.4m (Odometry) ---
    wait_for_center_button_press("Obj 1: Go 1.4m")
    drive_straight(1400, CRUISE_SPEED_MM_S)
    ev3.speaker.beep(frequency=1000, duration=200)

    # --- Objective 2: Approach Wall to 40cm ---
    wait_for_center_button_press("Obj 2: Approach wall")
    
    # Start continuous running
    left_motor.run(approach_speed_deg_s)
    right_motor.run(approach_speed_deg_s)

    # Loop until the robot is close enough
    while ultrasonic_sensor.distance() > TARGET_SENSOR_DISTANCE_MM:
        wait(10) # Small delay to prevent overwhelming the processor

    # Use hold() for a strong, synchronized stop
    left_motor.hold()
    right_motor.hold()
    ev3.speaker.beep(frequency=1200, duration=200)

    # --- Objective 3: Bump and Reverse ---
    wait_for_center_button_press("Obj 3: Bump & back up")
    
    # Part A: Drive forward until a bumper is pressed
    left_motor.run(approach_speed_deg_s)
    right_motor.run(approach_speed_deg_s)

    while not left_touch_sensor.pressed() and not right_touch_sensor.pressed():
        wait(10)
    
    left_motor.hold()
    right_motor.hold()
    ev3.speaker.beep(frequency=800, duration=150)
    
    # Part B: Reverse until 40cm from the wall
    wait(100) # Small delay before reversing
    
    left_motor.run(-approach_speed_deg_s)
    right_motor.run(-approach_speed_deg_s)
    
    while ultrasonic_sensor.distance() < TARGET_SENSOR_DISTANCE_MM:
        wait(10)

    left_motor.hold()
    right_motor.hold()
    
    # --- Completion ---
    ev3.speaker.beep(frequency=600, duration=300)
    ev3.screen.clear()
    ev3.screen.print("Lab complete.")

if __name__ == "__main__":
    main()
# endregion