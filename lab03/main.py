#!/usr/bin/env pybricks-micropython
# Team Members: Lianrui Geng && Xinyi Guo
# Lab 02  WALL 
#
# This program controls an EV3 robot to:
# 1. Drive forward until it detects a wall (using touch sensors)
# 2. Back up and turn 90 degrees to the right
# 3. Follow the wall at a constant distance using ultrasonic sensor feedback
#
# The robot uses PID control algorithms for:
# - Straight-line driving with gyro correction
# - Precise turning to target angles
# - Wall-following distance maintenance

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait, StopWatch

# ============================ CONFIGURATION =============================

# Hardware Ports
# Define the physical ports where each motor and sensor is connected
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_LEFT_PORT = Port.S1     # Left bumper touch sensor
TOUCH_RIGHT_PORT = Port.S3    # Right bumper touch sensor
GYRO_PORT = Port.S2           # Gyroscope for angle measurement
ULTRA_PORT = Port.S4          # Ultrasonic sensor facing LEFT side
# test

# Robot Geometry
# Physical dimensions of the robot needed for distance calculations
WHEEL_DIAMETER_MM = 56.0      # Diameter of drive wheels in millimeters
AXLE_TRACK_MM = 125.0         # Distance between left and right wheels

# Movement Parameters
# Base speed settings for different movement types
DRIVE_SPEED = 180             # Motor speed in degrees per second for forward motion
TURN_SPEED = 80               # Motor speed in degrees per second for turning

# ============================ INITIALIZATION =============================

# Initialize EV3 Brick
# Create the main EV3 brick object for display and speaker control
ev3 = EV3Brick()

# Initialize motors
# Create motor objects for the left and right drive motors
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

# Initialize sensors
# Create sensor objects for collision detection, angle tracking, and distance measurement
touch_left = TouchSensor(TOUCH_LEFT_PORT)
touch_right = TouchSensor(TOUCH_RIGHT_PORT)
gyro = GyroSensor(GYRO_PORT)
ultrasonic = UltrasonicSensor(ULTRA_PORT)

# Reset gyro sensor
# Set the gyro to zero degrees as the initial reference angle
gyro.reset_angle(0)
wait(10)

# ============================ HELPER FUNCTIONS =============================

def drive_straight_pid(distance_mm, speed=DRIVE_SPEED):
    """
    Drive straight for a specific distance using gyro PID control.
    
    This function uses a PID controller to maintain a straight path by monitoring
    the gyro sensor and making real-time corrections to motor speeds. The robot
    will drive the specified distance while actively correcting for any drift.
    
    Args:
        distance_mm: Target distance to travel in millimeters (positive=forward, negative=backward)
        speed: Base motor speed in degrees per second
    
    PID Control:
        - Proportional (P): Corrects based on current angle error
        - Integral (I): Corrects accumulated error over time
        - Derivative (D): Dampens oscillations by considering rate of change
    """
    # Calculate how many degrees the motors need to rotate based on wheel circumference
    wheel_circumference = math.pi * WHEEL_DIAMETER_MM
    target_rotation = (abs(distance_mm) / wheel_circumference) * 360
    
    # PID gain constants - tuned for straight-line performance
    GYRO_KP = 3.0      # Proportional gain - strength of immediate correction
    GYRO_KI = 0.01     # Integral gain - correction for accumulated drift
    GYRO_KD = 1.5      # Derivative gain - dampening to prevent oscillation
    
    # Determine direction: 1 for forward, -1 for backward
    direction = 1 if distance_mm > 0 else -1
    
    # Reset motor encoders to track distance traveled
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_gyro = gyro.angle()
    
    # Initialize PID variables
    gyro_integral = 0       # Accumulated error for integral term
    gyro_last_error = 0     # Previous error for derivative calculation
    
    # Timer for calculating time intervals (dt) between iterations
    stopwatch = StopWatch()
    last_time = 0
    
    print("Driving straight: " + str(distance_mm) + " mm")
    
    # Main control loop - runs until target distance is reached
    while True:
        # Calculate time since last iteration (in seconds)
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt == 0:
            dt = 0.05  # Prevent division by zero
        last_time = current_time
        
        # Check if we've traveled the target distance
        avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        
        if avg_rotation >= target_rotation:
            break
        
        # Calculate gyro error (deviation from straight line)
        gyro_error = gyro.angle() - initial_gyro
        
        # PID calculations
        # P term: immediate response proportional to error
        gyro_p = GYRO_KP * gyro_error
        
        # I term: accumulate error over time, with anti-windup limits
        gyro_integral += gyro_error * dt
        gyro_integral = max(-30, min(30, gyro_integral))
        gyro_i = GYRO_KI * gyro_integral
        
        # D term: rate of change to smooth corrections
        gyro_derivative = (gyro_error - gyro_last_error) / dt
        gyro_d = GYRO_KD * gyro_derivative
        gyro_last_error = gyro_error
        
        # Combine PID terms for total correction
        correction = gyro_p + gyro_i + gyro_d
        correction = max(-50, min(50, correction))  # Limit correction magnitude
        
        # Apply correction to motor speeds
        # Positive gyro error means robot turned right, so speed up left motor
        if direction > 0:
            left_speed = speed - correction
            right_speed = speed + correction
        else:
            left_speed = speed + correction
            right_speed = speed - correction
        
        # Apply direction multiplier
        left_speed = left_speed * direction
        right_speed = right_speed * direction
        
        # Safety limits to prevent excessive speed
        max_abs_speed = speed * 1.2
        left_speed = max(-max_abs_speed, min(max_abs_speed, left_speed))
        right_speed = max(-max_abs_speed, min(max_abs_speed, right_speed))
        
        # Command motors with calculated speeds
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        wait(20)  # 20ms delay between iterations
    
    # Stop motors with braking for precise positioning
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    
    print("Drive complete.")


def turn_in_place_pid(angle_degrees, speed=TURN_SPEED):
    """
    Turn in place by specified angle using gyro feedback.
    
    This function performs a precise turn using a two-stage approach:
    1. Coarse turn: Quickly get close to target angle
    2. Fine turn: Use PID control for precise final positioning
    
    Args:
        angle_degrees: Angle to turn in degrees (positive=counterclockwise, negative=clockwise)
        speed: Maximum turning speed in degrees per second
    
    Two-stage approach prevents overshoot while maintaining speed.
    """
    # PID constants for coarse phase (faster, less precise)
    COARSE_KP = 2.5
    
    # PID constants for fine tuning phase (slower, more precise)
    FINE_KP = 5.0      # Higher proportional gain for tighter control
    FINE_KI = 0.08     # Integral to eliminate steady-state error
    FINE_KD = 3.0      # Derivative to prevent oscillation
    
    # Record starting angle and calculate target
    initial_gyro = gyro.angle()
    target_gyro = initial_gyro + angle_degrees
    
    # Initialize PID variables for fine tuning
    integral = 0
    last_error = 0
    
    # Timer and stability counter
    stopwatch = StopWatch()
    last_time = 0
    stable_count = 0  # Counts iterations where robot is stable at target
    
    print("Turning from " + str(initial_gyro) + " to " + str(target_gyro) + " deg")
    
    # ========== Phase 1: Coarse turn ==========
    # Quickly get within 5 degrees of target
    while True:
        current_gyro = gyro.angle()
        error = target_gyro - current_gyro
        
        # Exit when close enough to target
        if abs(error) < 5:
            break
        
        # Safety timeout to prevent infinite loop
        if stopwatch.time() > 5000:
            print("Coarse turn timeout!")
            break
        
        # Simple proportional control for coarse turn
        turn_speed = max(-speed, min(speed, COARSE_KP * error))
        
        # Opposite motor directions for in-place turning
        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)
        
        wait(10)
    
    # Brief stop between phases
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    
    stopwatch.reset()
    
    # ========== Phase 2: Fine turn ==========
    # Use full PID control for precise positioning
    while True:
        # Calculate time interval for derivative and integral terms
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt == 0 or dt < 0.01:
            dt = 0.02  # Minimum dt to prevent calculation issues
        last_time = current_time
        
        current_gyro = gyro.angle()
        error = target_gyro - current_gyro
        
        # Check if we're stable at the target angle
        if abs(error) < 0.5:
            stable_count += 1
            if stable_count > 5:  # Must be stable for multiple iterations
                print("Target reached!")
                break
        else:
            stable_count = 0  # Reset if we deviate
        
        # Safety timeout for fine tuning
        if stopwatch.time() > 3000:
            print("Fine turn timeout")
            break
        
        # Full PID calculation
        p = FINE_KP * error
        
        # Integral with anti-windup
        integral += error * dt
        integral = max(-10, min(10, integral))
        i = FINE_KI * integral
        
        # Derivative for smooth approach
        derivative = (error - last_error) / dt
        d = FINE_KD * derivative
        last_error = error
        
        # Combine PID terms
        turn_speed = p + i + d
        turn_speed = max(-speed * 0.6, min(speed * 0.6, turn_speed))
        
        # Apply to motors (opposite directions for turning)
        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)
        
        wait(20)
    
    # Final stop with braking
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(200)
    
    print("Turn complete!")


def drive_until_collision_controlled(speed=DRIVE_SPEED):
    """
    Drive forward until collision is detected by touch sensors.
    
    Uses gyro feedback to maintain straight path while approaching the wall.
    Stops immediately when either touch sensor detects contact.
    
    Args:
        speed: Forward driving speed in degrees per second
    """
    print("Driving forward until collision...")
    
    wait(10)
    # Reset encoders and gyro for this movement
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_gyro = gyro.angle()
    wait(10)
    
    # Simple proportional control for gyro correction
    GYRO_KP = 2.5
    
    # Main loop: drive forward with gyro correction until collision
    while True:
        # Check both touch sensors
        left_pressed = touch_left.pressed()
        right_pressed = touch_right.pressed()
        
        # Stop immediately on any collision
        if left_pressed or right_pressed:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("Collision detected!")
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
        
        wait(10)


def follow_wall_diagnostic(target_distance_mm=200, wall_length_mm=2400, speed=DRIVE_SPEED):
    """
    Diagnostic version of wall following - outputs detailed information for debugging.
    
    This function follows a wall at a constant distance using the ultrasonic sensor.
    The robot adjusts its steering to maintain the target distance from the wall.
    
    Args:
        target_distance_mm: Desired distance from wall in millimeters
        wall_length_mm: Distance to travel along the wall in millimeters
        speed: Base forward speed in degrees per second
    
    Algorithm:
        1. Continuously measure distance to wall with ultrasonic sensor
        2. Calculate error (actual distance - target distance)
        3. Apply correction to steering (differential drive)
           - Too close: turn away from wall (left motor faster)
           - Too far: turn toward wall (right motor faster)
        4. Display diagnostic information on screen and terminal
    
    Key Features:
        - Detailed console output for each iteration
        - Screen display showing current status
        - Special handling for "far away" situations
        - Gyro-based drift prevention (currently disabled)
    """
    print("="*50)
    print("DIAGNOSTIC WALL FOLLOWING")
    print("="*50)
    print("Target: " + str(target_distance_mm) + "mm")
    print("Length: " + str(wall_length_mm) + "mm")
    
    # ========== Key Parameters ==========
    TARGET_DISTANCE = target_distance_mm
    
    # Correction gain: how aggressively to steer based on distance error
    # Higher value = more aggressive steering
    CORRECTION_GAIN = 1.4
    MAX_CORRECTION = 100  # Maximum steering correction to prevent excessive turning
    
    # Gyro assist: helps maintain parallel orientation to wall
    # Currently disabled (set to 0.0) to test distance-only control
    GYRO_ASSIST = 0.0
    
    # Option to reverse correction sign if steering direction is backwards
    REVERSE_CORRECTION = False  # If direction is wrong, change to True
    
    # Special parameter: number of iterations to suppress correction when far away
    # Helps prevent oscillation when robot suddenly moves far from wall
    K_FAR = 30

    # Smoothing factor for distance measurements (low-pass filter)
    # ALPHA=1.0 means no smoothing, ALPHA=0.0 means maximum smoothing
    ALPHA = 0.35

    
    print("CORRECTION_GAIN: " + str(CORRECTION_GAIN))
    print("MAX_CORRECTION: " + str(MAX_CORRECTION))
    print("GYRO_ASSIST: " + str(GYRO_ASSIST))
    print("REVERSE_CORRECTION: " + str(REVERSE_CORRECTION))
    print("="*50)   
    
    # ========== Initialization ==========
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    # Store initial gyro angle as reference for "parallel to wall"
    parallel_gyro_reference = gyro.angle()
    last_distance = TARGET_DISTANCE  # Initialize filter
    
    iteration = 0         # Loop counter for diagnostics
    continue_far = 0      # Counter for "far away" state

    # ========== Main Wall-Following Loop ==========
    while True:
        iteration += 1
        
        # ========== Step 1: Read Ultrasonic Distance ==========
        try:
            current_distance = ultrasonic.distance()
        except:
            # If sensor fails, use last good reading
            current_distance = last_distance
        
        # Filter invalid readings
        if current_distance <= 0:
            current_distance = last_distance
        else:
            # Apply exponential moving average filter to smooth noisy readings
            # This reduces jitter from ultrasonic sensor
            current_distance = ALPHA*current_distance + (1-ALPHA)*last_distance

        # ========== Step 2: Calculate Distance Error ==========
        # Positive error = too far from wall
        # Negative error = too close to wall
        distance_error = current_distance - TARGET_DISTANCE
        
        # ========== Step 3: Calculate Distance-Based Correction ==========
        distance_correction = distance_error * CORRECTION_GAIN
        
        # Special handling when robot is far from wall
        # Temporarily reduce correction to avoid sudden jerky movements
        if distance_error > 15 and continue_far < K_FAR:
            continue_far += 1
            distance_correction = -10  # Gentle correction toward wall
        elif distance_error < 15:
            continue_far = 0  # Reset counter when back in normal range
    
        # Limit correction to prevent excessive steering
        distance_correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, distance_correction))
        
        # ========== Step 4: Calculate Gyro-Based Correction (Optional) ==========
        # This helps maintain parallel orientation to the wall
        current_gyro = gyro.angle()
        gyro_deviation = current_gyro - parallel_gyro_reference
        gyro_correction = gyro_deviation * GYRO_ASSIST  # Currently disabled (GYRO_ASSIST=0.0)
        
        # ========== Step 5: Combine Corrections ==========
        total_correction = distance_correction + gyro_correction
        
        # Safety override: if gyro indicates severe misalignment, force gentle correction
        if gyro_deviation>60:
            total_correction=10    # Force gentle right turn
        elif gyro_deviation<-60:
            total_correction=-10   # Force gentle left turn
        
        # Apply correction reversal if needed (for debugging)
        if REVERSE_CORRECTION:
            total_correction = -total_correction
        
        # ========== Step 6: Apply Correction to Motor Speeds ==========
        # Positive correction: turn left (toward wall)
        #   - Decrease left motor speed, increase right motor speed
        # Negative correction: turn right (away from wall)
        #   - Increase left motor speed, decrease right motor speed
        left_speed = speed - total_correction
        right_speed = speed + total_correction
        
        # Enforce speed limits
        min_speed = 40            # Minimum to prevent stalling
        max_speed = speed * 1.6   # Allow significant speed differential for tight turns
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))
        
        # Command motors
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        # ========== Step 7: Diagnostic Output ==========
        # Print detailed information for each iteration
        print("="*50)
        print("Iter: " + str(iteration))
        print("Distance: " + str(int(current_distance)) + "mm")
        print("Error: " + str(int(distance_error)) + "mm")
        print("Correction: " + str(int(total_correction)))
        print("Left Speed: " + str(int(left_speed)))
        print("Right Speed: " + str(int(right_speed)))
        
        # Show what the robot SHOULD be doing based on distance error
        if distance_error < -20:
            print(">>> TOO CLOSE - Should turn RIGHT (away)")
            print(">>> Expected: Left FASTER, Right SLOWER")
        elif distance_error > 20:
            print(">>> TOO FAR - Should turn LEFT (toward)")
            print(">>> Expected: Left SLOWER, Right FASTER")
        else:
            print(">>> GOOD DISTANCE")
        
        # Show what the robot IS actually doing based on motor speeds
        if left_speed > right_speed + 20:
            print(">>> ACTUAL: Turning RIGHT")
        elif right_speed > left_speed + 20:
            print(">>> ACTUAL: Turning LEFT")
        else:
            print(">>> ACTUAL: Going STRAIGHT")
        
        print("="*50)
        
        # ========== Step 8: Update Screen Display ==========
        ev3.screen.clear()
        ev3.screen.draw_text(5, 5, "D:" + str(current_distance))
        # ev3.screen.draw_text(5, 20, "Err:" + str(int(distance_error)))
        ev3.screen.draw_text(5, 20, "Corr:" + str(int(total_correction)))
        ev3.screen.draw_text(5, 65, "pos: " + str(continue_far))
        # ev3.screen.draw_text(5, 50, "L:" + str(int(left_speed)))
        # ev3.screen.draw_text(5, 65, "R:" + str(int(right_speed)))
        ev3.screen.draw_text(5, 50, "X: "+ str(ultrasonic.distance()))
        
        # Display status message
        if current_distance < TARGET_DISTANCE - 20:
            ev3.screen.draw_text(5, 80, "TOO CLOSE >>")
        elif current_distance > TARGET_DISTANCE + 20:
            ev3.screen.draw_text(5, 80, "<< TOO FAR")
        else:
            ev3.screen.draw_text(5, 80, "GOOD")
        
        # ========== Step 9: Check Completion ==========
        # Calculate distance traveled using motor encoders
        avg_motor_angle = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        wheel_circumference = math.pi * WHEEL_DIAMETER_MM
        distance_traveled = (avg_motor_angle / 360) * wheel_circumference
        last_distance = current_distance
        
        # Exit when we've traveled the full wall length
        if distance_traveled >= wall_length_mm:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("COMPLETE!")
            break
        
        wait(10)  # 10ms loop delay
    
    # Final stop
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)


# ============================ MAIN PROGRAM =============================

def main():
    """
    Main program that executes the complete wall-following task.
    
    Task sequence:
    1. Wait for user to press center button to start
    2. Drive forward until collision with wall
    3. Back up 30cm from wall
    4. Turn 90 degrees to the right
    5. Follow the wall for 2.4 meters using diagnostic wall-following
    6. Celebrate success with beeps
    
    Includes comprehensive error handling and progress reporting.
    """
    try:
        # ========== Startup Sequence ==========
        ev3.speaker.beep()
        print("="*50)
        print("DIAGNOSTIC VERSION")
        print("Detailed Speed Output")
        print("="*50)
        print("")
        print("Press CENTER to start...")
        
        # Wait for user to press center button
        while True:
            if Button.CENTER in ev3.buttons.pressed():
                break
            wait(10)
        
        # Confirmation beep and countdown
        ev3.speaker.beep()
        wait(3000)  # 3-second delay before starting
        
        # ============== Objective 1: Detect Wall ==============
        print("")
        print("="*50)
        print("OBJECTIVE 1: Detect Wall")
        print("="*50)
        
        # Drive forward until bumpers hit the wall
        drive_until_collision_controlled(speed=DRIVE_SPEED)
        
        ev3.speaker.beep()
        wait(500)
        
        # Back away from wall to prepare for turning
        print("Backing up 30cm...")
        drive_straight_pid(-200, speed=DRIVE_SPEED)  # Negative = backward
        wait(500)
        
        # ============== Objective 2: Turn Right 90° ==============
        print("")
        print("="*50)
        print("OBJECTIVE 2: Turn Right 90°")
        print("="*50)
        
        # Execute precise 90-degree turn using gyro feedback
        print("Turning...")
        turn_in_place_pid(90, speed=TURN_SPEED)
        wait(500)
        
        # Reset gyro to establish new "forward" direction
        print("Resetting gyro...")
        gyro.reset_angle(0)
        wait(300)
        
        ev3.speaker.beep()
        print("Turn complete!")
        wait(500)
        
        # ============== Objective 3: Diagnostic Wall Following ==============
        print("")
        print("="*50)
        print("OBJECTIVE 3: Diagnostic Wall Following")
        print("="*50)
        print("Watch terminal for detailed output!")
        print("Each iteration shows:")
        print("- Current distance")
        print("- Distance error")
        print("- Correction value")
        print("- Left/Right speeds")
        print("- Expected vs Actual turning direction")
        print("="*50)
        
        wait(10)
        # Execute wall following for 2.4 meters at 30cm target distance
        follow_wall_diagnostic(
            target_distance_mm=300,   # Stay 30cm from wall
            wall_length_mm=2400,      # Follow for 2.4 meters
            speed=DRIVE_SPEED
        )
        wait(10)
        
        # ============== SUCCESS! ==============
        print("")
        print("="*50)
        print("SUCCESS!")
        print("="*50)
        
        # Play ascending victory beeps
        for i in range(4):
            ev3.speaker.beep(frequency=800 + i*200, duration=100)
            wait(150)
        
    except Exception as e:
        # ========== Error Handling ==========
        print("")
        print("="*50)
        print("ERROR!")
        print("="*50)
        print("Error: " + str(e))
        
        # Emergency stop
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        
        # Play error beeps
        ev3.speaker.beep(frequency=400, duration=300)
        wait(200)
        ev3.speaker.beep(frequency=400, duration=300)


# ============================ RUN PROGRAM =============================

# Program entry point - execute main function when script is run
if __name__ == "__main__":
    main()