#!/usr/bin/env pybricks-micropython

# Authors: Lianrui Geng && Xinyi Guo
# Course:  UNC COMP 581
# Lab:     Lab 03
# Date:    [To be filled]

"""
EV3 MicroPython program for Lab 3: Boundary Tracing and Return to Start.

This program implements Lab 3: Boundary Tracing and Return to Start.

Task Sequence:
1. Start at starting point (2.0 m, 0.5 m)
2. Drive straight forward until collision with obstacle (bumper pressed)
3. Beep to indicate obstacle found, record hit point pose (odometry + heading)
4. Back away from obstacle
5. Turn right 90 degrees using gyro
6. Left-side wall following with PID control using ultrasonic sensor, keeping measuring point within 30 cm of obstacle boundary
7. Continue tracing until back near hit point
8. Turn away from obstacle and return to starting point (2.0 m, 0.5 m)

Based on prelab design with two-wheel differential drive:
- Two touch sensors (bumpers) for collision detection during initial approach
- Single ultrasonic sensor positioned on the left side, facing directly to the left (perpendicular to forward direction) for wall following only
- Gyro sensor for heading maintenance and accurate 90-degree turns
"""

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait, StopWatch

# ==============================================================================
# region: --- CONFIGURATION ---
# ==============================================================================

# -- Port Assignments
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
GYRO_PORT = Port.S2
TOUCH_LEFT_PORT = Port.S1     # Left bumper touch sensor
TOUCH_RIGHT_PORT = Port.S3   # Right bumper touch sensor
ULTRASONIC_LEFT_PORT = Port.S4   # Single ultrasonic sensor on left side, facing directly to the left

# -- Robot Physical Parameters
WHEEL_DIAMETER_MM = 56.0
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM
AXLE_TRACK_MM = 125.0  # Distance between wheels

# -- Movement Speeds
BASE_SPEED = 180  # Base motor speed in degrees per second
TURN_SPEED = 80   # Turning speed in degrees per second

# -- Objective Parameters
BACKUP_DISTANCE_MM = 200  # Distance to back away from obstacle after collision (20 cm)
TARGET_WALL_DISTANCE_MM = 200  # 20 cm target distance from obstacle boundary during wall following
MAX_WALL_DISTANCE_MM = 300  # Maximum allowed distance (30 cm requirement - measuring point must stay within this)

# -- Starting Point (from lab requirements)
START_POINT_X_MM = 2000.0  # 2.0 m
START_POINT_Y_MM = 500.0   # 0.5 m

# -- PID Parameters for Wall Following
PID_KP = 1.4  # Proportional gain
PID_KI = 0.01  # Integral gain
PID_KD = 1.5  # Derivative gain

# -- Hit Point Detection
HIT_POINT_TOLERANCE_MM = 100  # How close we need to be to consider "back at hit point"

# -- Odometry Update Parameters
ODOMETRY_UPDATE_INTERVAL_MS = 20  # Update pose every 20ms during movement

# endregion

# ==============================================================================
# region: --- INITIALIZATION ---
# ==============================================================================

ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
gyro = GyroSensor(GYRO_PORT)
touch_left = TouchSensor(TOUCH_LEFT_PORT)   # Left bumper touch sensor
touch_right = TouchSensor(TOUCH_RIGHT_PORT) # Right bumper touch sensor
ultrasonic_left = UltrasonicSensor(ULTRASONIC_LEFT_PORT)  # Single ultrasonic sensor on left side, facing directly to the left

# Reset gyro sensor
gyro.reset_angle(0)
wait(100)

# -- Global Pose State (for continuous tracking)
# Robot's current pose in the world coordinate system
robot_pose = {
    'x': 0.0,           # X position in mm (forward = positive)
    'y': 0.0,           # Y position in mm (left = positive)
    'heading': 0.0,     # Heading in degrees (0 = facing forward, positive = counterclockwise)
    'left_encoder': 0,   # Last left motor encoder reading
    'right_encoder': 0  # Last right motor encoder reading
}

# Reset motor encoders at start
left_motor.reset_angle(0)
right_motor.reset_angle(0)

# endregion

# ==============================================================================
# region: --- HELPER FUNCTIONS ---
# ==============================================================================

def update_odometry():
    """
    Update robot's pose using wheel odometry (differential drive kinematics).
    
    This function should be called regularly (every 20-50ms) during movement
    to maintain accurate position tracking. It uses the difference in encoder
    readings to calculate the robot's movement and update its x, y position.
    
    Odometry Calculation:
    - Calculate left and right wheel displacements
    - Compute forward and rotational movement
    - Update x, y coordinates based on current heading
    - Update heading from gyro sensor
    """
    global robot_pose
    
    # Get current encoder readings
    left_angle = left_motor.angle()
    right_angle = right_motor.angle()
    
    # Calculate change in encoder readings (in degrees)
    left_delta = left_angle - robot_pose['left_encoder']
    right_delta = right_angle - robot_pose['right_encoder']
    
    # Convert degrees to distance (mm)
    left_distance = (left_delta / 360.0) * WHEEL_CIRCUMFERENCE_MM
    right_distance = (right_delta / 360.0) * WHEEL_CIRCUMFERENCE_MM
    
    # Update stored encoder values
    robot_pose['left_encoder'] = left_angle
    robot_pose['right_encoder'] = right_angle
    
    # Calculate forward and rotational movement
    # Average forward distance
    forward_distance = (left_distance + right_distance) / 2.0
    
    # Rotational movement (difference between wheels)
    rotation_distance = right_distance - left_distance
    rotation_angle_rad = rotation_distance / AXLE_TRACK_MM  # Convert to radians
    
    # Get current heading from gyro (more reliable than calculated rotation)
    current_heading = gyro.angle()
    heading_rad = math.radians(current_heading)
    
    # Update pose
    # Move in the direction of current heading
    robot_pose['x'] += forward_distance * math.cos(heading_rad)
    robot_pose['y'] += forward_distance * math.sin(heading_rad)
    robot_pose['heading'] = current_heading
    
    return robot_pose


def get_current_pose():
    """Get the current robot pose (x, y, heading)."""
    update_odometry()  # Ensure pose is up to date
    return {
        'x': robot_pose['x'],
        'y': robot_pose['y'],
        'heading': robot_pose['heading']
    }


def reset_odometry():
    """
    Reset the odometry system to starting point (2.0 m, 0.5 m).
    
    Note: The robot physically starts at the starting point, so we initialize
    the odometry system to reflect this starting position.
    """
    global robot_pose
    robot_pose = {
        'x': START_POINT_X_MM,  # 2.0 m
        'y': START_POINT_Y_MM,  # 0.5 m
        'heading': 0.0,  # Facing forward (positive x direction)
        'left_encoder': 0,
        'right_encoder': 0
    }
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    gyro.reset_angle(0)
    wait(100)
    print("Odometry initialized at starting point (" + str(START_POINT_X_MM/1000.0) + 
          " m, " + str(START_POINT_Y_MM/1000.0) + " m)")


def distance_to_point(point):
    """
    Calculate Euclidean distance from current position to a target point.
    
    Args:
        point: Dictionary with 'x' and 'y' keys in mm
    
    Returns:
        Distance in mm
    """
    current = get_current_pose()
    dx = current['x'] - point['x']
    dy = current['y'] - point['y']
    return math.sqrt(dx*dx + dy*dy)


def wait_for_center_button_press():
    """Waits for the center button to be pressed and released."""
    ev3.screen.clear()
    ev3.screen.print("Press CENTER")
    ev3.screen.print("to start")
    
    # Wait for button to be released first (if already pressed)
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    
    # Wait for button press
    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)
    
    # Wait for button release
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    
    ev3.speaker.beep()


def drive_until_collision_controlled(speed=BASE_SPEED):
    """
    Drive forward until collision is detected by touch sensors.
    
    Uses gyro feedback to maintain straight path while approaching the obstacle.
    Stops immediately when either touch sensor detects contact.
    Updates odometry during movement.
    
    Args:
        speed: Forward driving speed in degrees per second
    """
    initial_gyro = gyro.angle()
    GYRO_KP = 2.5
    
    # Main loop: drive forward with gyro correction until collision
    while True:
        # Update odometry during movement
        update_odometry()
        
        # Check both touch sensors
        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            return
        
        # Calculate gyro correction to maintain straight path
        gyro_error = gyro.angle() - initial_gyro
        correction = GYRO_KP * gyro_error
        correction = max(-30, min(30, correction))  # Limit correction
        
        # Apply correction: if drifting right (positive error), speed up left motor
        left_speed = speed - correction
        right_speed = speed + correction
        
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        wait(ODOMETRY_UPDATE_INTERVAL_MS)


def turn_in_place_pid(angle_degrees, speed=TURN_SPEED):
    """
    Turn in place by specified angle using gyro feedback with PID control.
    
    Args:
        angle_degrees: Angle to turn (positive = counterclockwise, negative = clockwise)
        speed: Maximum turning speed in degrees per second
    """
    initial_gyro = gyro.angle()
    target_gyro = initial_gyro + angle_degrees
    
    # PID constants for turning
    KP = 5.0
    KI = 0.08
    KD = 3.0
    
    integral = 0
    last_error = 0
    stopwatch = StopWatch()
    last_time = 0
    stable_count = 0
    
    print("Turning " + str(angle_degrees) + " degrees...")
    
    while True:
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt == 0 or dt < 0.01:
            dt = 0.02
        last_time = current_time
        
        current_gyro = gyro.angle()
        error = target_gyro - current_gyro
        
        # Normalize error to -180 to 180 range
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        # Check if stable at target
        if abs(error) < 0.5:
            stable_count += 1
            if stable_count > 5:
                break
        else:
            stable_count = 0
        
        # Safety timeout
        if stopwatch.time() > 5000:
            print("Turn timeout!")
            break
        
        # PID calculation
        p = KP * error
        integral += error * dt
        integral = max(-10, min(10, integral))
        i = KI * integral
        derivative = (error - last_error) / dt
        d = KD * derivative
        last_error = error
        
        turn_speed = p + i + d
        turn_speed = max(-speed * 0.6, min(speed * 0.6, turn_speed))
        
        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)
        
        wait(20)
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(200)
    print("Turn complete!")


def turn_right_90deg_using_gyro(speed=TURN_SPEED):
    """Turn right 90 degrees using gyro feedback."""
    turn_in_place_pid(-90, speed)


def record_hit_point():
    """
    Record the current pose (position and heading) as the hit point.
    
    This function captures the robot's exact position and orientation when
    it detects the obstacle. It uses the continuous odometry system to get
    accurate x, y coordinates, not just encoder values.
    
    Returns:
        Dictionary with hit point pose: {'x': float, 'y': float, 'heading': float}
    """
    # Update odometry one final time before recording
    update_odometry()
    
    # Record actual coordinates from odometry system
    hit_point = {
        'x': robot_pose['x'],
        'y': robot_pose['y'],
        'heading': robot_pose['heading'],
        # Also store encoder values for reference/debugging
        'left_encoder': robot_pose['left_encoder'],
        'right_encoder': robot_pose['right_encoder']
    }
    
    print("="*50)
    print("HIT POINT RECORDED:")
    print("  Position: (" + str(int(hit_point['x'])) + ", " + str(int(hit_point['y'])) + ") mm")
    print("  Heading: " + str(int(hit_point['heading'])) + " deg")
    print("  Left encoder: " + str(hit_point['left_encoder']) + " deg")
    print("  Right encoder: " + str(hit_point['right_encoder']) + " deg")
    print("="*50)
    
    return hit_point


def back_near_hit_point(hit_point, tolerance_mm=HIT_POINT_TOLERANCE_MM):
    """
    Check if robot is back near the hit point using actual coordinates.
    
    This function uses the continuous odometry system to calculate the
    Euclidean distance to the hit point, which is more accurate than
    just using encoder differences (especially after a full loop).
    
    Args:
        hit_point: Dictionary with 'x', 'y', 'heading' keys
        tolerance_mm: Maximum distance to consider "near" (default 100mm)
    
    Returns:
        True if robot is within tolerance of hit point
    """
    # Update odometry to get current position
    current_pose = get_current_pose()
    
    # Calculate Euclidean distance to hit point
    distance = distance_to_point(hit_point)
    
    # Also check heading difference (for debugging)
    heading_delta = abs(current_pose['heading'] - hit_point['heading'])
    
    # Debug output (every 50 iterations to avoid spam)
    if not hasattr(back_near_hit_point, 'call_count'):
        back_near_hit_point.call_count = 0
    back_near_hit_point.call_count += 1
    
    if back_near_hit_point.call_count % 50 == 0:
        ev3.screen.clear()
        ev3.screen.draw_text(5, 5, "Dist to hit: " + str(int(distance)) + "mm")
        ev3.screen.draw_text(5, 25, "Pos: (" + str(int(current_pose['x'])) + 
                            ", " + str(int(current_pose['y'])) + ")")
        ev3.screen.draw_text(5, 45, "Hit: (" + str(int(hit_point['x'])) + 
                            ", " + str(int(hit_point['y'])) + ")")
        print("Distance to hit point: " + str(int(distance)) + " mm")
    
    # Consider "near" if within tolerance distance
    # Note: We don't strictly require heading match because robot might
    # be at same position but different orientation after full loop
    is_near = distance < tolerance_mm
    
    if is_near:
        print("="*50)
        print("BACK NEAR HIT POINT!")
        print("  Current: (" + str(int(current_pose['x'])) + ", " + 
              str(int(current_pose['y'])) + ") mm")
        print("  Hit point: (" + str(int(hit_point['x'])) + ", " + 
              str(int(hit_point['y'])) + ") mm")
        print("  Distance: " + str(int(distance)) + " mm")
        print("="*50)
    
    return is_near


def wall_following_pid(target_distance_mm=TARGET_WALL_DISTANCE_MM, speed=BASE_SPEED):
    """
    Left-side wall following using PID control.
    Maintains target distance from wall using left ultrasonic sensor.
    """
    print("Starting wall following...")
    print("Target distance: " + str(target_distance_mm) + " mm")
    
    # PID variables
    integral = 0
    last_error = 0
    last_distance = target_distance_mm
    ALPHA = 0.35  # Smoothing factor
    
    # Initialize
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    iteration = 0
    
    while True:
        iteration += 1
        
        # Read and filter distance
        try:
            current_distance = ultrasonic_left.distance()
            if current_distance <= 0:
                current_distance = last_distance
            else:
                # Exponential moving average filter
                current_distance = ALPHA * current_distance + (1 - ALPHA) * last_distance
        except:
            current_distance = last_distance
        
        last_distance = current_distance
        
        # Calculate error (target - actual)
        error = target_distance_mm - current_distance
        
        # PID calculation
        p = PID_KP * error
        
        integral += error
        integral = max(-30, min(30, integral))  # Anti-windup
        i = PID_KI * integral
        
        derivative = error - last_error
        d = PID_KD * derivative
        last_error = error
        
        correction = p + i + d
        correction = max(-100, min(100, correction))  # Limit correction
        
        # Apply correction to motor speeds
        # Positive correction = too close, turn right (left motor faster)
        # Negative correction = too far, turn left (right motor faster)
        left_speed = speed + correction
        right_speed = speed - correction
        
        # Enforce speed limits
        min_speed = 40
        max_speed = speed * 1.6
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))
        
        # Command motors
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        # Display status
        if iteration % 10 == 0:
            ev3.screen.clear()
            ev3.screen.draw_text(5, 5, "Dist: " + str(int(current_distance)))
            ev3.screen.draw_text(5, 25, "Err: " + str(int(error)))
            ev3.screen.draw_text(5, 45, "Corr: " + str(int(correction)))
        
        wait(10)


def turn_away_from_wall(speed=TURN_SPEED):
    """Turn away from wall (turn right) to prepare for return."""
    print("Turning away from wall...")
    turn_right_90deg_using_gyro(speed)
    wait(200)


def navigate_back_to_start_using_odometry(hit_point):
    """
    Navigate back to start position using odometry and gyro.
    
    This function uses the continuous odometry system to navigate back
    to the starting point (2.0 m, 0.5 m) as specified in lab requirements.
    It calculates the required turn angle and distance, then executes 
    the movement with gyro correction.
    
    Args:
        hit_point: Dictionary with hit point coordinates (not used for return-to-start,
                   but kept for consistency)
    """
    print("="*50)
    print("NAVIGATING BACK TO START")
    print("="*50)
    
    # Get current position
    current_pose = get_current_pose()
    start_point = {'x': START_POINT_X_MM, 'y': START_POINT_Y_MM}  # Start at (2.0m, 0.5m)
    
    # Calculate distance and angle to start
    distance_to_start = distance_to_point(start_point)
    dx = start_point['x'] - current_pose['x']
    dy = start_point['y'] - current_pose['y']
    target_heading = math.degrees(math.atan2(dy, dx))
    
    print("Current position: (" + str(int(current_pose['x'])) + ", " + 
          str(int(current_pose['y'])) + ") mm")
    print("Distance to start: " + str(int(distance_to_start)) + " mm")
    print("Target heading: " + str(int(target_heading)) + " deg")
    
    # Turn to face start position
    heading_error = target_heading - current_pose['heading']
    # Normalize to -180 to 180 range
    while heading_error > 180:
        heading_error -= 360
    while heading_error < -180:
        heading_error += 360
    
    if abs(heading_error) > 5:  # Only turn if significant error
        print("Turning " + str(int(heading_error)) + " degrees toward start...")
        turn_in_place_pid(heading_error, speed=TURN_SPEED)
        wait(200)
    
    # Drive straight to start
    print("Driving " + str(int(distance_to_start)) + " mm to start...")
    drive_straight_pid(distance_to_start, speed=BASE_SPEED)
    
    # Final position check
    final_pose = get_current_pose()
    final_distance = distance_to_point(start_point)
    print("Final position: (" + str(int(final_pose['x'])) + ", " + 
          str(int(final_pose['y'])) + ") mm")
    print("Final distance to start: " + str(int(final_distance)) + " mm")
    
    if final_distance > 50:
        print("WARNING: Did not reach start accurately!")
    else:
        print("Successfully returned to start!")


def drive_straight_pid(distance_mm, speed=BASE_SPEED):
    """Drive straight for a specific distance using gyro PID control."""
    wheel_circumference = math.pi * WHEEL_DIAMETER_MM
    target_rotation = (abs(distance_mm) / wheel_circumference) * 360
    
    GYRO_KP = 3.0
    GYRO_KI = 0.01
    GYRO_KD = 1.5
    
    direction = 1 if distance_mm > 0 else -1
    
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_gyro = gyro.angle()
    
    gyro_integral = 0
    gyro_last_error = 0
    stopwatch = StopWatch()
    last_time = 0
    
    while True:
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt == 0:
            dt = 0.05
        last_time = current_time
        
        avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        if avg_rotation >= target_rotation:
            break
        
        gyro_error = gyro.angle() - initial_gyro
        
        gyro_p = GYRO_KP * gyro_error
        gyro_integral += gyro_error * dt
        gyro_integral = max(-30, min(30, gyro_integral))
        gyro_i = GYRO_KI * gyro_integral
        gyro_derivative = (gyro_error - gyro_last_error) / dt
        gyro_d = GYRO_KD * gyro_derivative
        gyro_last_error = gyro_error
        
        correction = gyro_p + gyro_i + gyro_d
        correction = max(-50, min(50, correction))
        
        if direction > 0:
            left_speed = speed - correction
            right_speed = speed + correction
        else:
            left_speed = speed + correction
            right_speed = speed - correction
        
        left_speed = left_speed * direction
        right_speed = right_speed * direction
        
        max_abs_speed = speed * 1.2
        left_speed = max(-max_abs_speed, min(max_abs_speed, left_speed))
        right_speed = max(-max_abs_speed, min(max_abs_speed, right_speed))
        
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        wait(20)
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)

# endregion

# ==============================================================================
# region: --- MAIN PROGRAM ---
# ==============================================================================

def main():
    """Executes the complete Lab 3 task sequence."""
    ev3.speaker.beep()
    
    print("="*50)
    print("LAB 3: Boundary Tracing")
    print("="*50)
    
    # Initialize odometry system to starting point (2.0 m, 0.5 m)
    reset_odometry()  # This function prints the initialization message
    
    # Wait for button press
    wait_for_center_button_press()
    
    # ========== Phase 1: Drive Forward to Obstacle ==========
    print("")
    print("Phase 1: Driving forward to obstacle...")
    ev3.screen.clear()
    ev3.screen.print("Phase 1:")
    ev3.screen.print("Approach")
    
    # Drive forward until collision with obstacle (bumper pressed)
    drive_until_collision_controlled(BASE_SPEED)
    
    # Final odometry update after collision
    update_odometry()
    
    ev3.speaker.beep()
    print("Obstacle collision detected!")
    wait(200)
    
    # Back away from obstacle (20 cm back to hit point location)
    print("Backing away from obstacle to hit point...")
    drive_straight_pid(-BACKUP_DISTANCE_MM, speed=BASE_SPEED)
    
    # Final odometry update after backing up
    update_odometry()
    
    # Record hit point with accurate coordinates (at hit point, 20cm from obstacle)
    hit_point = record_hit_point()
    
    # Verify hit point recording
    verify_distance = distance_to_point(hit_point)
    print("Verification: Distance from recorded hit point: " + str(int(verify_distance)) + " mm")
    if verify_distance > 10:
        print("WARNING: Hit point verification failed! Distance should be ~0 mm")
    
    wait(500)
    
    # ========== Phase 2: Turn Right and Begin Tracing ==========
    print("")
    print("Phase 2: Turning right 90 degrees...")
    ev3.screen.clear()
    ev3.screen.print("Phase 2:")
    ev3.screen.print("Turn right")
    
    turn_right_90deg_using_gyro()
    gyro.reset_angle(0)  # Reset for wall following phase
    wait(300)
    
    # ========== Phase 3: Wall Following Loop ==========
    print("")
    print("Phase 3: Wall following...")
    ev3.screen.clear()
    ev3.screen.print("Phase 3:")
    ev3.screen.print("Wall follow")
    
    # Wall following until back near hit point
    # Initialize PID variables
    pid_integral = 0
    pid_last_error = 0
    last_distance = TARGET_WALL_DISTANCE_MM
    ALPHA = 0.35  # Smoothing factor for distance readings
    
    iteration = 0
    
    while True:
        iteration += 1
        
        # Check if back near hit point
        if back_near_hit_point(hit_point):
            print("Back near hit point!")
            break
        
        # Update odometry during wall following
        update_odometry()
        
        # Read and filter ultrasonic distance
        try:
            current_distance = ultrasonic_left.distance()
            if current_distance <= 0:
                current_distance = last_distance
            else:
                # Exponential moving average filter to smooth noisy readings
                current_distance = ALPHA * current_distance + (1 - ALPHA) * last_distance
        except:
            current_distance = last_distance
        
        last_distance = current_distance
        
        # Calculate error (target - actual)
        error = TARGET_WALL_DISTANCE_MM - current_distance
        
        # Full PID calculation
        p = PID_KP * error
        
        pid_integral += error
        pid_integral = max(-30, min(30, pid_integral))  # Anti-windup
        i = PID_KI * pid_integral
        
        derivative = error - pid_last_error
        d = PID_KD * derivative
        pid_last_error = error
        
        correction = p + i + d
        correction = max(-100, min(100, correction))  # Limit correction
        
        # Apply correction to motor speeds
        # Positive correction = too close, turn right (left motor faster)
        # Negative correction = too far, turn left (right motor faster)
        left_motor_speed = BASE_SPEED + correction
        right_motor_speed = BASE_SPEED - correction
        
        # Limit speeds
        min_speed = 40
        max_speed = BASE_SPEED * 1.6
        left_motor_speed = max(min_speed, min(max_speed, left_motor_speed))
        right_motor_speed = max(min_speed, min(max_speed, right_motor_speed))
        
        left_motor.run(left_motor_speed)
        right_motor.run(right_motor_speed)
        
        # Display status occasionally
        if iteration % 50 == 0:
            ev3.screen.clear()
            ev3.screen.draw_text(5, 5, "Dist: " + str(int(current_distance)))
            ev3.screen.draw_text(5, 25, "Err: " + str(int(error)))
        
        wait(ODOMETRY_UPDATE_INTERVAL_MS)
    
    # Stop after wall following
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(500)
    
    # ========== Phase 4: Return to Start ==========
    print("")
    print("Phase 4: Returning to start...")
    ev3.screen.clear()
    ev3.screen.print("Phase 4:")
    ev3.screen.print("Return")
    
    turn_away_from_wall()
    navigate_back_to_start_using_odometry(hit_point)
    
    # Final stop
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    
    # Completion
    print("")
    print("="*50)
    print("LAB 3 COMPLETE!")
    print("="*50)
    
    ev3.speaker.beep(frequency=800, duration=200)
    wait(200)
    ev3.speaker.beep(frequency=1000, duration=200)
    
    ev3.screen.clear()
    ev3.screen.print("Complete!")

if __name__ == "__main__":
    main()

# endregion

