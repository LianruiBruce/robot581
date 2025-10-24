#!/usr/bin/env pybricks-micropython
# ======================================================================
# COMP 581 - Lab 02: Wall Following with Right Turn
# Authors: [Your Names]
# PIDs: [Your PIDs]
# Team: [Your Team Number]
# Date: October 2025
#
# Hardware Configuration:
# - Left Motor: Port B
# - Right Motor: Port C
# - Touch Sensor Left (front): Port S1
# - Touch Sensor Right (front): Port S3
# - Gyro Sensor: Port S2
# - Ultrasonic Sensor (facing LEFT): Port S4
#
# API Policy: Only using ev3brick, ev3devices, parameters, tools
# ======================================================================

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch

# ============================ CONFIGURATION =============================

# Hardware Ports
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_LEFT_PORT = Port.S1
TOUCH_RIGHT_PORT = Port.S3
GYRO_PORT = Port.S2
ULTRA_PORT = Port.S4  # Facing LEFT side

# Robot Geometry
WHEEL_DIAMETER_MM = 56.0
AXLE_TRACK_MM = 125.0

# Speed Parameters (可根据实际调整)
APPROACH_SPEED = 160        # Speed when approaching wall (mm/s)
CRUISE_SPEED = 180          # Speed when following wall (mm/s)
MAX_SPEED = 300             # Maximum speed cap (mm/s)
TURN_SPEED = 130            # Speed for turning (deg/s)

# Task Parameters
WALL_DETECT_DISTANCE = 300  # Threshold to detect wall (30cm)
TARGET_WALL_DISTANCE = 220  # Target distance from wall (22cm, within 30cm requirement)
WALL_FOLLOW_DISTANCE = 2200 # Distance to travel along wall (2.2m)
MAX_WALL_DISTANCE = 300     # Maximum allowed distance from wall (30cm)
TIMEOUT_MS = 90000          # 90 second timeout

# Sensor Smoothing
ULTRA_FILTER_ALPHA = 0.65   # Smoothing factor (0-1, higher = smoother)

# PID Parameters for Wall Following (需要调优)
KP = 0.65                   # Proportional gain
KI = 0.0001                 # Integral gain (很小，防止累积误差)
KD = 0.12                   # Derivative gain

# Heading Control
K_HEADING = 1.0             # Heading correction gain
K_ADAPTIVE = 0.006          # Adaptive heading adjustment rate

# Control Limits
STEER_LIMIT = 0.55          # Maximum steering value

# ============================== UTILITIES ===============================

def clamp(value, min_val, max_val):
    """Clamp value between min and max"""
    return max(min_val, min(max_val, value))

def mm_per_sec_to_deg_per_sec(speed_mm_s):
    """Convert linear speed (mm/s) to wheel angular speed (deg/s)"""
    circumference = math.pi * WHEEL_DIAMETER_MM
    return (speed_mm_s / circumference) * 360.0

def set_motor_speeds(left_speed_mm_s, right_speed_mm_s):
    """Set motor speeds in mm/s"""
    left_speed = clamp(left_speed_mm_s, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed_mm_s, -MAX_SPEED, MAX_SPEED)
    
    left_motor.run(mm_per_sec_to_deg_per_sec(left_speed))
    right_motor.run(mm_per_sec_to_deg_per_sec(right_speed))

def stop_motors():
    """Stop both motors and hold position"""
    left_motor.hold()
    right_motor.hold()

def smooth_ultrasonic(previous_value):
    """Read and smooth ultrasonic sensor value with exponential filter"""
    raw_value = ultrasonic.distance()
    
    # Handle None values (sensor error)
    if raw_value is None:
        return previous_value if previous_value > 0 else 1000
    
    # First reading - no filtering needed
    if previous_value <= 0:
        return raw_value
    
    # Exponential moving average
    return ULTRA_FILTER_ALPHA * previous_value + (1 - ULTRA_FILTER_ALPHA) * raw_value

def normalize_angle(angle):
    """Normalize angle to [-180, 180] range"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# ============================== INITIALIZATION ==========================

ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
touch_left = TouchSensor(TOUCH_LEFT_PORT)
touch_right = TouchSensor(TOUCH_RIGHT_PORT)
gyro = GyroSensor(GYRO_PORT)
ultrasonic = UltrasonicSensor(ULTRA_PORT)

# ============================== CONTROL FUNCTIONS =======================

def drive_straight_with_heading(speed_mm_s, target_heading):
    """Drive straight while maintaining a specific heading using gyro"""
    current_heading = gyro.angle()
    heading_error = normalize_angle(target_heading - current_heading)
    
    # Calculate steering correction (positive = turn left)
    # Small proportional control for heading hold
    correction = (K_HEADING * heading_error) / 100.0
    correction = clamp(correction, -0.25, 0.25)
    
    # Apply differential steering
    left_speed = speed_mm_s * (1 + correction)
    right_speed = speed_mm_s * (1 - correction)
    
    set_motor_speeds(left_speed, right_speed)

def turn_to_heading(target_heading, tolerance=2.0):
    """Turn in place to reach target heading using PD control"""
    Kp = 2.8
    Kd = 7.0
    
    last_error = 0
    timer = StopWatch()
    timer.reset()
    
    while True:
        current_heading = gyro.angle()
        error = normalize_angle(target_heading - current_heading)
        
        # Check if reached target
        if abs(error) < tolerance:
            break
        
        # Calculate turn rate with PD control
        dt = max(timer.time(), 1) / 1000.0
        timer.reset()
        
        derivative = (error - last_error) / dt
        turn_rate = Kp * error + Kd * derivative
        turn_rate = clamp(turn_rate, -TURN_SPEED, TURN_SPEED)
        
        # Apply turn (positive error = turn left)
        left_motor.run(turn_rate)
        right_motor.run(-turn_rate)
        
        last_error = error
        wait(10)
    
    stop_motors()

# ============================== STAGE FUNCTIONS =========================

def stage_1_approach_wall():
    """
    Stage 1: Drive straight forward until collision with wall
    Since ultrasonic faces LEFT, we use touch sensors to detect front wall
    """
    ev3.screen.clear()
    ev3.screen.print("Stage 1: Approach")
    ev3.screen.print("")
    ev3.screen.print("Press center button")
    ev3.screen.print("to start")
    
    # Reset and calibrate gyro
    gyro.reset_angle(0)
    wait(300)
    
    # Wait for clean button press
    while ev3.buttons.pressed():
        wait(10)
    while not ev3.buttons.pressed():
        wait(10)
    while ev3.buttons.pressed():
        wait(10)
    
    ev3.speaker.beep()
    ev3.screen.clear()
    ev3.screen.print("Approaching wall...")
    
    # Drive forward maintaining straight line
    initial_heading = gyro.angle()
    
    while True:
        drive_straight_with_heading(APPROACH_SPEED, initial_heading)
        
        # Stop when touch sensor detects wall
        if touch_left.pressed() or touch_right.pressed():
            stop_motors()
            ev3.speaker.beep(frequency=800, duration=100)
            break
        
        wait(10)
    
    # Back up slightly to avoid being too close
    set_motor_speeds(-100, -100)
    wait(350)
    stop_motors()
    wait(100)

def stage_2_turn_right():
    """Stage 2: Turn right 90 degrees"""
    ev3.screen.clear()
    ev3.screen.print("Stage 2: Turning Right")
    
    current_heading = gyro.angle()
    target_heading = current_heading - 90  # Right turn = negative
    
    turn_to_heading(target_heading, tolerance=1.5)
    
    ev3.speaker.beep(frequency=1000, duration=150)
    wait(200)

def stage_3_follow_wall():
    """
    Stage 3: Follow the left wall using PID control
    Ultrasonic sensor faces LEFT to measure wall distance
    """
    ev3.screen.clear()
    ev3.screen.print("Stage 3: Following Wall")
    
    # Initialize controller state
    target_heading = gyro.angle()
    integral = 0.0
    last_error = 0.0
    filtered_distance = 0.0
    
    # Initialize odometry
    last_left_angle = left_motor.angle()
    last_right_angle = right_motor.angle()
    distance_traveled = 0.0
    
    # Initialize timers
    loop_timer = StopWatch()
    total_timer = StopWatch()
    loop_timer.reset()
    total_timer.reset()
    
    # Main wall-following loop
    while distance_traveled < WALL_FOLLOW_DISTANCE:
        # Safety: check timeout
        if total_timer.time() > TIMEOUT_MS:
            ev3.screen.clear()
            ev3.screen.print("TIMEOUT!")
            break
        
        # Read and filter wall distance from LEFT ultrasonic
        filtered_distance = smooth_ultrasonic(filtered_distance)
        wall_distance = filtered_distance
        
        # Calculate time delta for derivatives
        dt = max(loop_timer.time(), 1) / 1000.0
        loop_timer.reset()
        
        # ========== PID Control for Lateral Distance ==========
        # Error: positive when too far from wall, negative when too close
        error = TARGET_WALL_DISTANCE - wall_distance
        
        # Integral with anti-windup
        integral += error * dt
        integral = clamp(integral, -500, 500)
        
        # Derivative
        derivative = (error - last_error) / dt
        
        # PID output (positive = steer left toward wall)
        pid_output = KP * error + KI * integral + KD * derivative
        steering_pid = pid_output / 100.0  # Scale to reasonable range
        
        # ========== Adaptive Heading for Curves ==========
        # Slowly adjust target heading based on sustained error
        # If consistently too far, turn slightly left; if too close, turn right
        target_heading += K_ADAPTIVE * error
        
        # ========== Heading Hold Using Gyro ==========
        current_heading = gyro.angle()
        heading_error = normalize_angle(target_heading - current_heading)
        heading_correction = 0.012 * heading_error
        
        # ========== Combine Steering Commands ==========
        total_steering = steering_pid + heading_correction
        total_steering = clamp(total_steering, -STEER_LIMIT, STEER_LIMIT)
        
        # Apply to motors (positive steering = turn left)
        base_speed = CRUISE_SPEED
        left_speed = base_speed * (1 + total_steering)
        right_speed = base_speed * (1 - total_steering)
        set_motor_speeds(left_speed, right_speed)
        
        # ========== Odometry Update ==========
        current_left = left_motor.angle()
        current_right = right_motor.angle()
        
        # Calculate wheel displacement
        delta_left = (current_left - last_left_angle) / 360.0 * (math.pi * WHEEL_DIAMETER_MM)
        delta_right = (current_right - last_right_angle) / 360.0 * (math.pi * WHEEL_DIAMETER_MM)
        
        # Average of both wheels for distance
        distance_traveled += abs((delta_left + delta_right) / 2.0)
        
        last_left_angle = current_left
        last_right_angle = current_right
        last_error = error
        
        # ========== Safety: Handle Collision ==========
        if touch_left.pressed() or touch_right.pressed():
            stop_motors()
            ev3.speaker.beep(frequency=600, duration=100)
            
            # Back up
            set_motor_speeds(-120, -120)
            wait(400)
            stop_motors()
            
            # Turn slightly away from wall
            target_heading = gyro.angle() - 8
            turn_to_heading(target_heading)
            
            # Reset PID state
            integral = 0
            last_error = 0
            wait(100)
        
        # ========== Handle Lost Wall ==========
        # If distance suddenly too large, gently turn toward wall
        if wall_distance > 650:
            # Continue forward while turning slightly left to search
            target_heading += 4.0
            drive_straight_with_heading(CRUISE_SPEED * 0.9, target_heading)
            wait(250)
            # Reset integral to avoid overshoot when wall found
            integral = 0
        
        wait(10)
    
    # ========== Task Complete ==========
    stop_motors()
    ev3.speaker.beep(frequency=1200, duration=400)
    wait(100)
    ev3.speaker.beep(frequency=1400, duration=400)
    
    ev3.screen.clear()
    ev3.screen.print("=== TASK COMPLETE ===")
    ev3.screen.print("")
    ev3.screen.print("Distance: %.0f mm" % distance_traveled)
    ev3.screen.print("Target: %d mm" % WALL_FOLLOW_DISTANCE)
    ev3.screen.print("Time: %.1f s" % (total_timer.time() / 1000.0))

# ============================== MAIN ====================================

def main():
    """Main program execution"""
    try:
        stage_1_approach_wall()
        stage_2_turn_right()
        stage_3_follow_wall()
        
    except Exception as e:
        # Emergency stop and error reporting
        stop_motors()
        ev3.screen.clear()
        ev3.screen.print("ERROR OCCURRED!")
        ev3.screen.print(str(e))
        ev3.speaker.beep(frequency=400, duration=1000)

if __name__ == "__main__":
    main()