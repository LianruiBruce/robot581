#!/usr/bin/env pybricks-micropython
# Team Members: Lianrui Geng && Xinyi Guo
# Lab 03  BOUNDARY TRACING AND RETURN TO START
#
# This program controls an EV3 robot to:
# 1. Start at starting point (2.0 m, 0.5 m)
# 2. Drive forward until it detects an obstacle (using touch sensors)
# 3. Record hit point (position and heading)
# 4. Back up and turn 90 degrees to the right
# 5. Follow the obstacle boundary clockwise until back near hit point
# 6. Turn away from wall and return to starting point using odometry
#
# The robot uses PID control algorithms for:
# - Straight-line driving with gyro correction
# - Precise turning to target angles
# - Wall-following distance maintenance
# - Odometry-based navigation back to start

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait, StopWatch

# ============================ CONFIGURATION =============================

# Hardware Ports
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_LEFT_PORT = Port.S1     # Left bumper touch sensor
TOUCH_RIGHT_PORT = Port.S3    # Right bumper touch sensor
GYRO_PORT = Port.S2           # Gyroscope for angle measurement
ULTRA_PORT = Port.S4          # Ultrasonic sensor facing LEFT side

# Robot Geometry
WHEEL_DIAMETER_MM = 56.0      # Diameter of drive wheels in millimeters
AXLE_TRACK_MM = 125.0         # Distance between left and right wheels

# Movement Parameters
DRIVE_SPEED = 180             # Motor speed in degrees per second for forward motion
TURN_SPEED = 80               # Motor speed in degrees per second for turning

# Lab 3 Specific Parameters
BACKUP_DISTANCE_MM = 200      # Distance to back away from obstacle (20 cm)
TARGET_WALL_DISTANCE_MM = 200 # Target distance from wall during following (20 cm)
MAX_WALL_DISTANCE_MM = 300    # Maximum allowed distance (30 cm requirement)
HIT_POINT_TOLERANCE_MM = 100  # How close to be considered "back at hit point"

# Starting Point (from lab requirements)
START_POINT_X_MM = 2000.0     # 2.0 m
START_POINT_Y_MM = 500.0      # 0.5 m

# Wall Following PID Parameters
WALL_KP = 1.4                 # Proportional gain for wall following
WALL_KI = 0.01                # Integral gain
WALL_KD = 1.5                 # Derivative gain

# ============================ INITIALIZATION =============================

ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
touch_left = TouchSensor(TOUCH_LEFT_PORT)
touch_right = TouchSensor(TOUCH_RIGHT_PORT)
gyro = GyroSensor(GYRO_PORT)
ultrasonic = UltrasonicSensor(ULTRA_PORT)

# Reset gyro sensor
gyro.reset_angle(0)
wait(10)

# Odometry state - track robot position
robot_x = START_POINT_X_MM    # X position in mm (starts at 2.0m)
robot_y = START_POINT_Y_MM    # Y position in mm (starts at 0.5m)
robot_heading = 0.0           # Heading in degrees (0 = positive X direction)
last_left_angle = 0           # Last left motor encoder reading
last_right_angle = 0           # Last right motor encoder reading

# Reset motor encoders
left_motor.reset_angle(0)
right_motor.reset_angle(0)

# ============================ HELPER FUNCTIONS =============================

def update_odometry():
    """
    Update robot's position using wheel odometry (differential drive kinematics).
    Should be called regularly during movement to maintain accurate position tracking.
    """
    global robot_x, robot_y, robot_heading, last_left_angle, last_right_angle
    
    # Get current encoder readings
    left_angle = left_motor.angle()
    right_angle = right_motor.angle()
    
    # Calculate change in encoder readings (in degrees)
    left_delta = left_angle - last_left_angle
    right_delta = right_angle - last_right_angle
    
    # Convert degrees to distance (mm)
    wheel_circumference = math.pi * WHEEL_DIAMETER_MM
    left_distance = (left_delta / 360.0) * wheel_circumference
    right_distance = (right_delta / 360.0) * wheel_circumference
    
    # Update stored encoder values
    last_left_angle = left_angle
    last_right_angle = right_angle
    
    # Calculate forward and rotational movement
    forward_distance = (left_distance + right_distance) / 2.0
    rotation_distance = right_distance - left_distance
    rotation_angle_rad = rotation_distance / AXLE_TRACK_MM
    
    # Get current heading from gyro (more reliable)
    robot_heading = gyro.angle()
    heading_rad = math.radians(robot_heading)
    
    # Update position based on current heading
    robot_x += forward_distance * math.cos(heading_rad)
    robot_y += forward_distance * math.sin(heading_rad)
    
    return (robot_x, robot_y, robot_heading)


def distance_to_point(x, y):
    """Calculate Euclidean distance from current position to target point."""
    update_odometry()
    dx = robot_x - x
    dy = robot_y - y
    return math.sqrt(dx*dx + dy*dy)


def drive_straight_pid(distance_mm, speed=DRIVE_SPEED):
    """
    Drive straight for a specific distance using gyro PID control.
    
    This function uses a PID controller to maintain a straight path by monitoring
    the gyro sensor and making real-time corrections to motor speeds.
    
    Args:
        distance_mm: Target distance to travel in millimeters (positive=forward, negative=backward)
        speed: Base motor speed in degrees per second
    """
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
    
    print("Driving straight: " + str(distance_mm) + " mm")
    
    while True:
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt == 0:
            dt = 0.05
        last_time = current_time
        
        # Update odometry during movement
        update_odometry()
        
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
    
    print("Drive complete.")


def turn_in_place_pid(angle_degrees, speed=TURN_SPEED):
    """
    Turn in place by specified angle using gyro feedback.
    
    Uses two-stage approach: coarse turn followed by fine PID control.
    
    Args:
        angle_degrees: Angle to turn in degrees (positive=counterclockwise, negative=clockwise)
        speed: Maximum turning speed in degrees per second
    """
    COARSE_KP = 2.5
    FINE_KP = 5.0
    FINE_KI = 0.08
    FINE_KD = 3.0
    
    initial_gyro = gyro.angle()
    target_gyro = initial_gyro + angle_degrees
    
    integral = 0
    last_error = 0
    stopwatch = StopWatch()
    last_time = 0
    stable_count = 0
    
    print("Turning from " + str(initial_gyro) + " to " + str(target_gyro) + " deg")
    
    # Phase 1: Coarse turn
    while True:
        current_gyro = gyro.angle()
        error = target_gyro - current_gyro
        
        if abs(error) < 5:
            break
        
        if stopwatch.time() > 5000:
            print("Coarse turn timeout!")
            break
        
        turn_speed = max(-speed, min(speed, COARSE_KP * error))
        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)
        wait(10)
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    
    stopwatch.reset()
    
    # Phase 2: Fine turn
    while True:
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt == 0 or dt < 0.01:
            dt = 0.02
        last_time = current_time
        
        current_gyro = gyro.angle()
        error = target_gyro - current_gyro
        
        if abs(error) < 0.5:
            stable_count += 1
            if stable_count > 5:
                print("Target reached!")
                break
        else:
            stable_count = 0
        
        if stopwatch.time() > 3000:
            print("Fine turn timeout")
            break
        
        p = FINE_KP * error
        integral += error * dt
        integral = max(-10, min(10, integral))
        i = FINE_KI * integral
        derivative = (error - last_error) / dt
        d = FINE_KD * derivative
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


def drive_until_collision_controlled(speed=DRIVE_SPEED):
    """
    Drive forward until collision is detected by touch sensors.
    
    Uses gyro feedback to maintain straight path while approaching the obstacle.
    Updates odometry during movement.
    
    Args:
        speed: Forward driving speed in degrees per second
    """
    print("Driving forward until collision...")
    
    wait(10)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_gyro = gyro.angle()
    wait(10)
    
    GYRO_KP = 2.5
    
    while True:
        # Update odometry during movement
        update_odometry()
        
        left_pressed = touch_left.pressed()
        right_pressed = touch_right.pressed()
        
        if left_pressed or right_pressed:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("Collision detected!")
            return
        
        gyro_error = gyro.angle() - initial_gyro
        correction = GYRO_KP * gyro_error
        correction = max(-30, min(30, correction))
        
        left_speed = speed - correction
        right_speed = speed + correction
        
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        wait(10)


def follow_wall_until_hit_point(hit_point_x, hit_point_y, target_distance_mm=TARGET_WALL_DISTANCE_MM, speed=DRIVE_SPEED):
    """
    Follow wall using PID control until robot returns near hit point.
    
    This function follows the obstacle boundary clockwise, maintaining target distance
    from the wall. It continues until the robot is within tolerance of the hit point.
    
    Args:
        hit_point_x: X coordinate of hit point in mm
        hit_point_y: Y coordinate of hit point in mm
        target_distance_mm: Desired distance from wall in millimeters
        speed: Base forward speed in degrees per second
    """
    print("="*50)
    print("WALL FOLLOWING UNTIL HIT POINT")
    print("="*50)
    print("Target distance: " + str(target_distance_mm) + "mm")
    print("Hit point: (" + str(int(hit_point_x)) + ", " + str(int(hit_point_y)) + ") mm")
    
    # PID variables
    integral = 0
    last_error = 0
    last_distance = target_distance_mm
    ALPHA = 0.35  # Smoothing factor
    
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    iteration = 0
    
    while True:
        iteration += 1
        
        # Update odometry
        update_odometry()
        
        # Check if back near hit point
        dist_to_hit = distance_to_point(hit_point_x, hit_point_y)
        if dist_to_hit < HIT_POINT_TOLERANCE_MM:
            print("Back near hit point! Distance: " + str(int(dist_to_hit)) + " mm")
            break
        
        # Read and filter distance
        try:
            current_distance = ultrasonic.distance()
            if current_distance <= 0:
                current_distance = last_distance
            else:
                current_distance = ALPHA * current_distance + (1 - ALPHA) * last_distance
        except:
            current_distance = last_distance
        
        last_distance = current_distance
        
        # Calculate error
        error = target_distance_mm - current_distance
        
        # PID calculation
        p = WALL_KP * error
        
        integral += error
        integral = max(-30, min(30, integral))
        i = WALL_KI * integral
        
        derivative = error - last_error
        d = WALL_KD * derivative
        last_error = error
        
        correction = p + i + d
        correction = max(-100, min(100, correction))
        
        # Apply correction
        # Positive correction = too close, turn right (left motor faster)
        # Negative correction = too far, turn left (right motor faster)
        left_speed = speed + correction
        right_speed = speed - correction
        
        # Limit speeds
        min_speed = 40
        max_speed = speed * 1.6
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))
        
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        # Display status occasionally
        if iteration % 50 == 0:
            ev3.screen.clear()
            ev3.screen.draw_text(5, 5, "Dist: " + str(int(current_distance)))
            ev3.screen.draw_text(5, 25, "To hit: " + str(int(dist_to_hit)))
            ev3.screen.draw_text(5, 45, "Pos: (" + str(int(robot_x)) + "," + str(int(robot_y)) + ")")
        
        wait(10)
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    print("Wall following complete!")


def navigate_back_to_start():
    """
    Navigate back to start position (2.0 m, 0.5 m) using odometry.
    
    Calculates the required turn angle and distance, then executes the movement.
    """
    print("="*50)
    print("NAVIGATING BACK TO START")
    print("="*50)
    
    # Update odometry to get current position
    update_odometry()
    
    # Calculate distance and angle to start
    dx = START_POINT_X_MM - robot_x
    dy = START_POINT_Y_MM - robot_y
    distance_to_start = math.sqrt(dx*dx + dy*dy)
    target_heading = math.degrees(math.atan2(dy, dx))
    
    print("Current position: (" + str(int(robot_x)) + ", " + str(int(robot_y)) + ") mm")
    print("Distance to start: " + str(int(distance_to_start)) + " mm")
    print("Target heading: " + str(int(target_heading)) + " deg")
    
    # Turn to face start position
    heading_error = target_heading - robot_heading
    # Normalize to -180 to 180 range
    while heading_error > 180:
        heading_error -= 360
    while heading_error < -180:
        heading_error += 360
    
    if abs(heading_error) > 5:
        print("Turning " + str(int(heading_error)) + " degrees toward start...")
        turn_in_place_pid(heading_error, speed=TURN_SPEED)
        wait(200)
    
    # Drive straight to start
    print("Driving " + str(int(distance_to_start)) + " mm to start...")
    drive_straight_pid(distance_to_start, speed=DRIVE_SPEED)
    
    # Final position check
    update_odometry()
    final_distance = distance_to_point(START_POINT_X_MM, START_POINT_Y_MM)
    print("Final position: (" + str(int(robot_x)) + ", " + str(int(robot_y)) + ") mm")
    print("Final distance to start: " + str(int(final_distance)) + " mm")
    
    if final_distance > 50:
        print("WARNING: Did not reach start accurately!")
    else:
        print("Successfully returned to start!")


# ============================ MAIN PROGRAM =============================

def main():
    """
    Main program that executes the complete Lab 3 task sequence.
    
    Task sequence:
    1. Wait for user to press center button to start
    2. Drive forward until collision with obstacle
    3. Record hit point (position and heading)
    4. Back up from obstacle
    5. Turn 90 degrees to the right
    6. Follow obstacle boundary until back near hit point
    7. Turn away from wall
    8. Return to starting point using odometry
    """
    try:
        # Startup
        ev3.speaker.beep()
        print("="*50)
        print("LAB 3: Boundary Tracing and Return to Start")
        print("="*50)
        print("Press CENTER to start...")
        
        # Wait for button press
        while True:
            if Button.CENTER in ev3.buttons.pressed():
                break
            wait(10)
        
        ev3.speaker.beep()
        wait(1000)
        
        # ========== Phase 1: Drive Forward to Obstacle ==========
        print("")
        print("="*50)
        print("PHASE 1: Drive Forward to Obstacle")
        print("="*50)
        
        drive_until_collision_controlled(speed=DRIVE_SPEED)
        ev3.speaker.beep()
        wait(500)
        
        # ========== Phase 2: Record Hit Point and Back Up ==========
        print("")
        print("="*50)
        print("PHASE 2: Record Hit Point and Back Up")
        print("="*50)
        
        # Final odometry update at collision point
        update_odometry()
        
        # Record hit point (at collision, before backing up)
        hit_point_x = robot_x
        hit_point_y = robot_y
        hit_point_heading = robot_heading
        
        print("Hit point recorded:")
        print("  Position: (" + str(int(hit_point_x)) + ", " + str(int(hit_point_y)) + ") mm")
        print("  Heading: " + str(int(hit_point_heading)) + " deg")
        
        # Back away from obstacle
        print("Backing up " + str(BACKUP_DISTANCE_MM) + " mm...")
        drive_straight_pid(-BACKUP_DISTANCE_MM, speed=DRIVE_SPEED)
        wait(500)
        
        # ========== Phase 3: Turn Right 90° ==========
        print("")
        print("="*50)
        print("PHASE 3: Turn Right 90°")
        print("="*50)
        
        print("Turning...")
        turn_in_place_pid(-90, speed=TURN_SPEED)  # Negative = clockwise (right)
        wait(500)
        
        # Reset gyro to establish new "forward" direction
        print("Resetting gyro...")
        gyro.reset_angle(0)
        wait(300)
        
        ev3.speaker.beep()
        print("Turn complete!")
        wait(500)
        
        # ========== Phase 4: Wall Following Until Back Near Hit Point ==========
        print("")
        print("="*50)
        print("PHASE 4: Wall Following Until Back Near Hit Point")
        print("="*50)
        
        follow_wall_until_hit_point(hit_point_x, hit_point_y, 
                                   target_distance_mm=TARGET_WALL_DISTANCE_MM, 
                                   speed=DRIVE_SPEED)
        wait(500)
        
        # ========== Phase 5: Turn Away and Return to Start ==========
        print("")
        print("="*50)
        print("PHASE 5: Turn Away and Return to Start")
        print("="*50)
        
        print("Turning away from wall...")
        turn_in_place_pid(-90, speed=TURN_SPEED)  # Turn right (away from wall)
        wait(500)
        
        navigate_back_to_start()
        
        # ========== SUCCESS! ==========
        print("")
        print("="*50)
        print("LAB 3 COMPLETE!")
        print("="*50)
        
        # Play victory beeps
        for i in range(4):
            ev3.speaker.beep(frequency=800 + i*200, duration=100)
            wait(150)
        
        ev3.screen.clear()
        ev3.screen.print("Complete!")
        
    except Exception as e:
        print("")
        print("="*50)
        print("ERROR!")
        print("="*50)
        print("Error: " + str(e))
        
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        
        ev3.speaker.beep(frequency=400, duration=300)
        wait(200)
        ev3.speaker.beep(frequency=400, duration=300)


# ============================ RUN PROGRAM =============================

if __name__ == "__main__":
    main()
