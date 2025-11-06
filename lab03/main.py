#!/usr/bin/env pybricks-micropython
# Team Members: Lianrui Geng && Xinyi Guo
# Lab 03  BOUNDARY TRACING AND RETURN TO START
#
# This program implements Lab 3: Boundary Tracing and Return to Start.
# 
# Task Sequence:
# 1. Start at starting point (2.0 m, 0.5 m)
# 2. Drive straight forward until obstacle is detected (within 30cm or contact)
# 3. Beep to indicate obstacle found
# 4. Record hit point (position and heading) - 20cm from obstacle front wall
# 5. Back away from obstacle
# 6. Turn right 90 degrees using gyro
# 7. Left-side wall following with PID control, keeping measuring point within 30 cm
# 8. Continue tracing until back near hit point
# 9. Turn away from obstacle and return to starting point using odometry

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
TOUCH_RIGHT_PORT = Port.S3   # Right bumper touch sensor
GYRO_PORT = Port.S2           # Gyroscope for angle measurement
ULTRA_PORT = Port.S4         # Ultrasonic sensor facing LEFT side

# Robot Geometry
WHEEL_DIAMETER_MM = 56.0      # Diameter of drive wheels in millimeters
AXLE_TRACK_MM = 125.0         # Distance between left and right wheels
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM

# Movement Parameters
DRIVE_SPEED = 180             # Motor speed in degrees per second for forward motion
TURN_SPEED = 80               # Motor speed in degrees per second for turning

# Lab 3 Specific Parameters
BACKUP_DISTANCE_MM = 200      # Distance to back away from obstacle (20 cm)
TARGET_WALL_DISTANCE_MM = 200 # Target distance from wall during following (20 cm)
MAX_WALL_DISTANCE_MM = 300    # Maximum allowed distance (30 cm requirement)
HIT_POINT_TOLERANCE_MM = 100  # How close to be considered "back at hit point"
OBSTACLE_DETECTION_DISTANCE_MM = 300  # Distance to detect obstacle (30 cm)

# Starting Point (from lab requirements)
# NOTE: The starting point is the hit point, which is 20 cm away from the obstacle's front wall.
START_POINT_X_MM = 2000.0     # 2.0 m
START_POINT_Y_MM = 500.0      # 0.5 m

# Wall Following PID Parameters (reduced to prevent overreaction)
WALL_KP = 0.8                 # Reduced proportional gain to prevent overreaction
WALL_KI = 0.005               # Reduced integral gain
WALL_KD = 0.8                 # Reduced derivative gain

# Dead Reckoning Parameters
STEP_DISTANCE_MM = 80         # Distance to move in each step (dead reckoning)
STEP_CHECK_INTERVAL = 10      # Check sensors every N steps

# Straight Drive PID Parameters
GYRO_KP = 3.0                 # Gyro correction PID gains
GYRO_KI = 0.01
GYRO_KD = 1.5

# Turn Control Parameters
COARSE_KP = 2.5               # Coarse turn proportional gain
FINE_KP = 5.0                 # Fine turn PID gains
FINE_KI = 0.08
FINE_KD = 3.0

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
robot_x = START_POINT_X_MM    # X position in mm
robot_y = START_POINT_Y_MM    # Y position in mm
robot_heading = 0.0           # Heading in degrees (0 = positive X direction)
last_left_angle = 0            # Last left motor encoder reading
last_right_angle = 0           # Last right motor encoder reading

# Reset motor encoders
left_motor.reset_angle(0)
right_motor.reset_angle(0)

# ============================ HELPER FUNCTIONS =============================

# 这个方法是将角度归一化到-180到180度之间, 也就是角度归一化的函数
def normalize_angle(angle_deg):
    """
    Normalize angle to -180 to 180 degree range.
    This ensures we always take the shortest path when turning.
    """
    angle_deg = angle_deg % 360  # First normalize to 0-360
    if angle_deg > 180:
        angle_deg -= 360
    return angle_deg

# 更新机器人位置,这个是计算机器人位置的核心函数
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
    # 这个是计算左右轮的差值
    left_delta = left_angle - last_left_angle
    right_delta = right_angle - last_right_angle
    
    # Convert degrees to distance (mm) 
    left_distance = (left_delta / 360.0) * WHEEL_CIRCUMFERENCE_MM
    right_distance = (right_delta / 360.0) * WHEEL_CIRCUMFERENCE_MM
    
    # Update stored encoder values
    # 更新左右轮的角速度
    last_left_angle = left_angle
    last_right_angle = right_angle
    
    # Calculate forward movement (average of both wheels)
    # 计算前进距离
    forward_distance = (left_distance + right_distance) / 2.0
    
    # Get current heading from gyro (more reliable than calculated rotation)
    # 获取当前航向角
    robot_heading = gyro.angle()
    # 将航向角转换为弧度
    heading_rad = math.radians(robot_heading)
    
    # Update position based on current heading
    # 更新机器人位置,根据当前航向角和前进距离,更新机器人位置(不太准,在目前来看)
    robot_x += forward_distance * math.cos(heading_rad)
    robot_y += forward_distance * math.sin(heading_rad)
    
    return (robot_x, robot_y, robot_heading)


# 计算机器人当前位置到目标位置的距离
def distance_to_point(x, y):
    """Calculate Euclidean distance from current position to target point."""
    dx = robot_x - x
    dy = robot_y - y
    return math.sqrt(dx*dx + dy*dy)


def drive_straight_pid(distance_mm, speed=DRIVE_SPEED):
    """
    Drive straight for a specific distance using gyro PID control.
    
    Args:
        distance_mm: Target distance (positive=forward, negative=backward)
        speed: Base motor speed in degrees per second
    """
    target_rotation = (abs(distance_mm) / WHEEL_CIRCUMFERENCE_MM) * 360
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
        
        # Check if target distance reached
        avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        if avg_rotation >= target_rotation:
            break
        
        # Calculate gyro error
        gyro_error = gyro.angle() - initial_gyro
        
        # PID calculation
        gyro_p = GYRO_KP * gyro_error
        gyro_integral += gyro_error * dt
        gyro_integral = max(-30, min(30, gyro_integral))
        gyro_i = GYRO_KI * gyro_integral
        gyro_derivative = (gyro_error - gyro_last_error) / dt
        gyro_d = GYRO_KD * gyro_derivative
        gyro_last_error = gyro_error
        
        correction = gyro_p + gyro_i + gyro_d
        correction = max(-50, min(50, correction))
        
        # Apply correction
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
    wait(10)
    
    # Final odometry update
    update_odometry()

# 这个方法是根据陀螺仪角度来控制机器人转向, 也就是转向的控制依据
def turn_in_place_simple(angle_degrees, speed=TURN_SPEED):
    """
    Simple turn-in-place using gyro feedback. No advanced convergence checks.
    Turns robot by the specified angle (positive=clockwise/right, negative=counterclockwise/left).
    """
    initial_gyro = gyro.angle()
    target_gyro = initial_gyro + angle_degrees

    # Normalize target so the robot takes the shortest path
    def normalize_angle_simple(deg):
        while deg > 180:
            deg -= 360
        while deg < -180:
            deg += 360
        return deg

    Kp = 2.5
    Ki = 0.02
    Kd = 0.5

    integral = 0
    last_error = 0

    while True:
        current_gyro = gyro.angle()
        error = normalize_angle_simple(target_gyro - current_gyro)
        if abs(error) < 2:  # close enough (degrees)
            break

        # Simple PID
        p = Kp * error
        integral += error * 0.02  # dt=20ms ~=0.02s
        integral = max(-10, min(10, integral))
        i = Ki * integral
        d = Kd * (error - last_error) / 0.02
        last_error = error
        turn = p + i + d
        turn = max(-speed, min(speed, turn))

        left_motor.run(turn)
        right_motor.run(-turn)
        wait(20)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)

# def turn_in_place_pid(angle_degrees, speed=TURN_SPEED):
    """
    Turn in place by specified angle using gyro feedback.
    
    This function handles ANY angle, including angles >180°.
    Uses improved angle normalization and progress tracking to prevent spinning in place.
    
    IMPORTANT: In this system, positive angle = clockwise/right turn.
    
    Args:
        angle_degrees: Angle to turn (positive=clockwise/right, negative=counterclockwise/left)
        speed: Maximum turning speed in degrees per second
    """
    initial_gyro = gyro.angle()
    
    # Calculate target angle FIRST (before normalization)
    target_gyro_raw = initial_gyro + angle_degrees
    
    # 计算误差
    error_raw = target_gyro_raw - initial_gyro
    error_normalized = normalize_angle(error_raw)
    # 计算目标角度
    target_gyro = initial_gyro + error_normalized
    
    
    # 这几行代码的含义如下：
    # 1. integral = 0
    #    初始化PID控制器中的积分项为0，用于累计误差从而消除稳态偏差。
    # 2. last_error = 0
    #    记录上一次循环中的误差值，为微分项计算提供依据。
    # 3. last_time = 0
    #    上一次PID更新的时间戳，可用于按实际时间步长积分（如果后续用到）。
    # 4. stable_count = 0
    #    用来统计机器人稳定在目标附近的次数，常用于判断是否保持在目标角度一段时间。
    # 5. no_progress_count = 0
    #    记录陀螺仪读数长时间没有变化的次数，用于检测机器人是否卡住或打滑。
    # 6. last_gyro = initial_gyro
    #    保存上一次读取的陀螺仪角度值，用于后续“无进展”判断。
    # 7. consecutive_same_error = 0
    #    统计连续误差几乎没有变化的次数，用于检测机器人是否在原地空转或误差未收敛。
    # 8. last_error_value = None
    #    保存上一次误差的具体数值，辅助判断误差变化趋势。
    integral = 0
    last_error = 0
    last_time = 0
    stable_count = 0
    no_progress_count = 0
    last_gyro = initial_gyro
    consecutive_same_error = 0
    last_error_value = None
    
    # ========== Phase 1: Coarse turn ==========
    # Quickly get within 5 degrees of target
    while True:
        current_gyro = gyro.angle()
        
        # 这里计算的是误差
        error = target_gyro - current_gyro
        error = normalize_angle(error)
        
        # Check if we're making progress
        if last_error_value is not None:
            if abs(error - last_error_value) < 0.5:
                consecutive_same_error += 1
            else:
                consecutive_same_error = 0
            last_error_value = error
        
        if abs(error) < 5:
            print("Coarse turn complete, error=" + str(int(error)) + "°")
            break
        
        # Check for stuck/spinning (no progress OR error not changing)
        if abs(current_gyro - last_gyro) < 1.0:
            no_progress_count += 1
            if no_progress_count > 30:  # Stuck for 0.3 seconds
                print("WARNING: Turn appears stuck (no gyro change), forcing completion")
                break
        else:
            no_progress_count = 0
            last_gyro = current_gyro
        
        # Check if error is not changing (spinning in place)
        if consecutive_same_error > 100:  # Same error for 1 second
            print("WARNING: Error not changing (spinning), forcing completion")
            break
        

        
        # Proportional control with aggressive gain for large errors
        if abs(error) > 45:
            kp = COARSE_KP * 2.0  # Much more aggressive for large errors
        elif abs(error) > 20:
            kp = COARSE_KP * 1.5
        else:
            kp = COARSE_KP
        
        turn_speed = kp * error
        turn_speed = max(-speed, min(speed, turn_speed))
        
        # Ensure minimum speed for errors > 2 degrees
        if abs(turn_speed) < 15 and abs(error) > 2:
            turn_speed = 15 if error > 0 else -15
        
        # Apply to motors (opposite directions for in-place turning)
        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)
        
        wait(10)
    
    # Brief stop between phases
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(150)

    no_progress_count = 0
    consecutive_same_error = 0
    last_gyro = gyro.angle()
    last_error_value = None
    
    # ========== Phase 2: Fine turn ==========
    # Use full PID control for precise positioning
    while True:
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt == 0 or dt < 0.01:
            dt = 0.02
        last_time = current_time
        
        current_gyro = gyro.angle()
        error = target_gyro - current_gyro
        
        # Normalize error to -180 to 180 range
        error = normalize_angle(error)
        
        # Check progress
        if last_error_value is not None:
            if abs(error - last_error_value) < 0.2:
                consecutive_same_error += 1
            else:
                consecutive_same_error = 0
            last_error_value = error
        
        # Check if stable at target
        if abs(error) < 0.5:
            stable_count += 1
            if stable_count > 5:
                print("Fine turn complete! Final heading: " + str(int(current_gyro)) + "°")
                break
        else:
            stable_count = 0
        
        # Check for stuck
        if abs(current_gyro - last_gyro) < 0.3:
            no_progress_count += 1
            if no_progress_count > 20:
                print("Fine turn appears stuck, accepting current position")
                break
        else:
            no_progress_count = 0
            last_gyro = current_gyro
        
        # Check if error not changing
        if consecutive_same_error > 50:
            print("Fine turn error not changing, accepting position")
            break
        
        # Safety timeout
        if stopwatch.time() > 4000:
            print("Fine turn timeout, error: " + str(int(error)) + "°")
            break
        
        # Full PID calculation
        p = FINE_KP * error
        integral += error * dt
        integral = max(-10, min(10, integral))
        i = FINE_KI * integral
        derivative = (error - last_error) / dt
        d = FINE_KD * derivative
        last_error = error
        
        turn_speed = p + i + d
        turn_speed = max(-speed * 0.6, min(speed * 0.6, turn_speed))
        
        # Apply to motors
        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)
        
        wait(20)
    
    # Final stop
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(200)
    
    # Update odometry after turn
    update_odometry()
    final_error = normalize_angle(target_gyro - robot_heading)
    print("Turn complete! Heading: " + str(int(robot_heading)) + "°, error: " + str(int(final_error)) + "°")

def drive_until_obstacle_detected(speed=DRIVE_SPEED):
    """
    Drive forward until obstacle is detected via TOUCH SENSORS ONLY.
    
    CRITICAL: Ultrasonic sensor is on the LEFT SIDE, so it cannot detect
    obstacles in front during forward motion. We ONLY use touch sensors here.
    
    Beeps when obstacle is found.
    
    Returns:
        True if obstacle detected, False if timeout
    """
    
    wait(10)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_gyro = gyro.angle()
    wait(10)
    
    GYRO_CORRECTION_KP = 2.5

    while True:  
        # Update odometry during movement
        update_odometry()
        
        # CRITICAL: ONLY check touch sensors - ultrasonic is on left side, not front!
        # Touch sensors are the reliable way to detect collision with front wall
        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            ev3.speaker.beep()
            update_odometry()
            return
        
        # Gyro correction to maintain straight path
        gyro_error = gyro.angle() - initial_gyro
        correction = GYRO_CORRECTION_KP * gyro_error
        # 这里“correction = max(-20, min(20, correction))”的含义是限制矫正转向的最大幅度，和“后退20cm”无关
        correction = max(-30, min(30, correction))
        
        left_speed = speed - correction
        right_speed = speed + correction
        
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        wait(10)

# 这个方法是根据机器人与墙的相对位置来判断应该采取哪种恢复策略, 也就是恢复策略的判断依据
def handle_collision_recovery_intelligent():
    """
    Intelligent collision recovery based on robot's position relative to wall.
    
    CRITICAL CONTEXT:
    - Robot is ALWAYS on the RIGHT side of the wall (left sensor faces wall)
    - Robot moves CLOCKWISE around the wall (right turns)
    - Ultrasonic sensor is on LEFT side, measuring distance to wall
    
    Collision Logic:
    - Left sensor only: Wall curves inward (concave corner) or too close to wall
                       → Back up, turn RIGHT to move away from wall
    - Right sensor only: Wall curves outward (convex corner) or obstacle ahead
                        → Back up, turn RIGHT to follow wall around corner
    - Both sensors: Direct frontal collision (dead end or sharp corner)
                   → Back up, turn RIGHT to continue following wall
    - Default: Always turn RIGHT (most common case)
    
    Returns:
        True if collision was handled, False otherwise
    """
    
    # Stop immediately
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(20)
    
    # Check which sensors are pressed
    left_pressed = touch_left.pressed()
    right_pressed = touch_right.pressed()
    
    # Determine collision type and adjust accordingly
    # REMEMBER: Robot is on RIGHT side of wall, moving clockwise
    if left_pressed and right_pressed:
        # Both sensors: Direct frontal collision (dead end or 90° corner)
        # Most likely a sharp corner - need aggressive right turn
        # 如果两个都触发，说明是直角，需要后退150mm，然后右转60度
        drive_straight_pid(-150, speed=DRIVE_SPEED * 0.7)
        wait(20)
       # turn_in_place_pid(60, speed=TURN_SPEED)  # Right turn to follow wall
       # 这里使用的是简单的转向方法，而不是PID控制, 试用一下看效果
        turn_in_place_simple(60, speed=TURN_SPEED)
        
    elif left_pressed and not right_pressed:
        # Left sensor only: Wall curves inward or robot too close to wall
        # Wall is on the LEFT, so we need to turn RIGHT to move away
        print("Left sensor: Wall curves inward or too close")
        print("  → Backing up 120mm, then turning RIGHT 35°...")
        drive_straight_pid(-120, speed=DRIVE_SPEED * 0.7)
        wait(200)
        # turn_in_place_pid(35, speed=TURN_SPEED)  # Right turn away from wall
        # 这里使用的是简单的转向方法，而不是PID控制, 试用一下看效果
        turn_in_place_simple(35, speed=TURN_SPEED)
        
        
    elif right_pressed and not left_pressed:
        # Right sensor only: Wall curves outward (convex corner) or obstacle ahead
        # This means wall ahead turns right, need to follow it
        print("Right sensor: Wall curves outward (convex corner)")
        print("  → Backing up 100mm, then turning RIGHT 45° to follow...")
        drive_straight_pid(-100, speed=DRIVE_SPEED * 0.7)
        wait(200)
        # turn_in_place_pid(45, speed=TURN_SPEED)  # Right turn to follow wall around corner
        # 这里使用的是简单的转向方法，而不是PID控制, 试用一下看效果
        turn_in_place_simple(45, speed=TURN_SPEED)
        
    else:
        # No sensors pressed (shouldn't happen, but handle gracefully)
        # Default: Turn RIGHT (most common recovery direction)
        print("No sensors detected, default recovery...")
        print("  → Backing up 100mm, then turning RIGHT 30°...")
        drive_straight_pid(-100, speed=DRIVE_SPEED * 0.7)
        wait(200)
        # turn_in_place_pid(30, speed=TURN_SPEED)  # Right turn (default)
        # 这里使用的是简单的转向方法，而不是PID控制, 试用一下看效果
        turn_in_place_simple(30, speed=TURN_SPEED)
    
    wait(10)
    print("Recovery complete!")
    return True


def check_pose_intelligent():
    """
    Intelligent pose checking using multiple methods:
    1. Check ultrasonic distance (handle infinite/out of range)
    2. Check collision sensors
    3. Move forward/backward to probe wall position
    4. Analyze robot orientation
    
    Returns:
        Tuple (needs_adjustment, adjustment_type, distance)
        adjustment_type: 'too_close', 'too_far', 'corner_detected', 'collision', None
    """
    update_odometry()
    
    # Check collision sensors first
    left_pressed = touch_left.pressed()
    right_pressed = touch_right.pressed()
    
    if left_pressed or right_pressed:
        return (True, 'collision', None)
    
    # Read ultrasonic distance multiple times to filter noise
    distances = []
    for i in range(3):
        try:
            dist = ultrasonic.distance()
            if dist > 0 and dist <= 8000:
                distances.append(dist)
        except:
            pass
        wait(10)
    
    if len(distances) == 0:
        # No valid readings - might be at corner or sensor issue
        print("No valid distance readings - possible corner detected")
        return (True, 'corner_detected', None)
    
    avg_distance = sum(distances) / len(distances)
    
    # Check for infinite/out of range (corner or wall end)
    if avg_distance > 5000 or avg_distance < 0:
        print("Distance out of range (" + str(int(avg_distance)) + "mm) - corner detected")
        return (True, 'corner_detected', None)
    
    # Normal distance check
    if avg_distance < TARGET_WALL_DISTANCE_MM - 50:  # Too close (< 15cm)
        return (True, 'too_close', avg_distance)
    elif avg_distance > MAX_WALL_DISTANCE_MM:  # Too far (> 30cm)
        return (True, 'too_far', avg_distance)
    
    # Probe forward to check if wall is ahead (for corner detection)
    # Small forward movement to check distance change
    print("Probing forward 30mm to check wall...")
    initial_distance = avg_distance
    initial_gyro = gyro.angle()
    
    # Move forward a small amount
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    probe_distance = 30
    target_rotation = (probe_distance / WHEEL_CIRCUMFERENCE_MM) * 360
    
    while True:
        # Check collision during probe
        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("Collision during probe!")
            drive_straight_pid(-probe_distance, speed=DRIVE_SPEED * 0.6)
            return (True, 'collision', None)
        
        avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        if avg_rotation >= target_rotation:
            break
        
        # Gyro correction
        gyro_error = gyro.angle() - initial_gyro
        correction = 2.0 * gyro_error
        left_motor.run(DRIVE_SPEED * 0.6 - correction)
        right_motor.run(DRIVE_SPEED * 0.6 + correction)
        wait(10)
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    
    # Check distance after probe
    try:
        probe_distance_reading = ultrasonic.distance()
        if probe_distance_reading > 0 and probe_distance_reading <= 8000:
            distance_change = abs(probe_distance_reading - initial_distance)
            
            # If distance changed significantly, we might be at a corner
            if distance_change > 100:
                print("Significant distance change (" + str(int(distance_change)) + "mm) - possible corner")
                # Back up to original position
                drive_straight_pid(-probe_distance, speed=DRIVE_SPEED * 0.6)
                return (True, 'corner_detected', None)
    except:
        pass
    
    # Back up to original position
    drive_straight_pid(-probe_distance, speed=DRIVE_SPEED * 0.6)
    wait(100)
    
    return (False, None, avg_distance)


def check_pose_and_adjust():
    """
    Check robot pose using dead reckoning and adjust if needed.
    Returns True if adjustment was made, False otherwise.
    """
    update_odometry()
    
    # Check both touch sensors
    left_pressed = touch_left.pressed()
    right_pressed = touch_right.pressed()
    
    if left_pressed or right_pressed:
        print("Collision detected during pose check!")
        return True
    
    # Read ultrasonic distance
    try:
        distance = ultrasonic.distance()
        if distance <= 0 or distance > 8000:
            return False
    except:
        return False
    
    # If too close to wall, we need to adjust
    if distance < TARGET_WALL_DISTANCE_MM - 50:  # 15cm or less
        print("Too close to wall (" + str(int(distance)) + "mm), adjusting...")
        return True
    
    return False


def follow_wall_until_hit_point(hit_point_x, hit_point_y, target_distance_mm=TARGET_WALL_DISTANCE_MM, speed=DRIVE_SPEED):
    """
    Follow wall using Dead Reckoning approach: move in small steps, check pose frequently.
    
    Uses step-by-step movement with collision detection and pose checking after each step.
    Reduces overreaction by using lower PID gains and step-based movement.
    
    Args:
        hit_point_x: X coordinate of hit point in mm
        hit_point_y: Y coordinate of hit point in mm
        target_distance_mm: Desired distance from wall in millimeters
        speed: Base forward speed in degrees per second
    """
    print("="*50)
    print("WALL FOLLOWING UNTIL HIT POINT (Dead Reckoning)")
    print("="*50)
    print("Target distance: " + str(target_distance_mm) + " mm")
    print("Step distance: " + str(STEP_DISTANCE_MM) + " mm")
    print("Hit point: (" + str(int(hit_point_x)) + ", " + str(int(hit_point_y)) + ") mm")
    
    # Initialize PID variables (reduced gains)
    integral = 0
    last_error = 0
    last_distance = target_distance_mm
    ALPHA = 0.5  # Increased smoothing to reduce noise
    
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    step_count = 0
    min_distance_seen = float('inf')
    initial_distance_to_hit = None
    max_distance_from_hit = 0
    
    while True:
        step_count += 1
        
        # ========== Step 1: Move forward one step (Dead Reckoning) ==========
        print("Step " + str(step_count) + ": Moving " + str(STEP_DISTANCE_MM) + "mm...")
        
        # Calculate target rotation for this step
        target_rotation = (STEP_DISTANCE_MM / WHEEL_CIRCUMFERENCE_MM) * 360
        
        # Read initial distance and calculate correction
        try:
            raw_distance = ultrasonic.distance()
            if raw_distance <= 0 or raw_distance > 8000:
                current_distance = last_distance
            else:
                current_distance = ALPHA * raw_distance + (1 - ALPHA) * last_distance
        except:
            current_distance = last_distance
        
        last_distance = current_distance
        
        # Calculate error and PID correction
        error = target_distance_mm - current_distance
        
        # PID calculation with reduced gains
        p = WALL_KP * error
        integral += error * 0.1  # Scale down integral accumulation
        integral = max(-20, min(20, integral))  # Reduced anti-windup
        i = WALL_KI * integral
        derivative = (error - last_error) / 1.0
        d = WALL_KD * derivative
        last_error = error
        
        correction = p + i + d
        # Limit correction more strictly to prevent overreaction
        correction = max(-60, min(60, correction))
        
        # Apply correction to motor speeds
        # Positive error = too close, turn RIGHT (left motor faster)
        # Negative error = too far, turn LEFT (right motor faster)
        left_speed = speed + correction
        right_speed = speed - correction
        
        # Limit speeds
        min_speed = 50
        max_speed = speed * 1.4  # Reduced max speed
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))
        
        # Move forward for this step
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        initial_gyro = gyro.angle()
        
        while True:
            # Check for collision during movement with intelligent detection
            left_pressed = touch_left.pressed()
            right_pressed = touch_right.pressed()
            
            if left_pressed or right_pressed:
                print("Collision during step movement!")
                left_motor.stop(Stop.BRAKE)
                right_motor.stop(Stop.BRAKE)
                wait(100)
                
                # Use intelligent recovery based on which sensor hit
                handle_collision_recovery_intelligent()
                integral = 0
                last_error = 0
                break
            
            # Gyro correction for straight movement
            gyro_error = gyro.angle() - initial_gyro
            gyro_correction = 2.0 * gyro_error
            gyro_correction = max(-20, min(20, gyro_correction))
            
            left_step_speed = left_speed - gyro_correction
            right_step_speed = right_speed + gyro_correction
            
            left_motor.run(left_step_speed)
            right_motor.run(right_step_speed)
            
            # Check if step distance reached
            avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
            if avg_rotation >= target_rotation:
                left_motor.stop(Stop.BRAKE)
                right_motor.stop(Stop.BRAKE)
                break
            
            wait(10)
        
        # Brief pause after step
        wait(50)
        
        # ========== Step 2: Intelligent pose check and adjust (Dead Reckoning) ==========
        needs_adjust, adjustment_type, check_distance = check_pose_intelligent()
        
        if needs_adjust:
            if adjustment_type == 'collision':
                # Handle collision with intelligent recovery
                handle_collision_recovery_intelligent()
                integral = 0
                last_error = 0
                continue
                
            elif adjustment_type == 'too_close':
                # Too close to wall (wall on LEFT, robot on RIGHT)
                # Need to turn RIGHT to move away from wall
                print("Too close (" + str(int(check_distance)) + "mm), backing up and turning RIGHT...")
                drive_straight_pid(-80, speed=DRIVE_SPEED * 0.6)
                wait(200)
                # Turn RIGHT away from wall (wall is on left)
                turn_in_place_pid(20, speed=TURN_SPEED * 0.7)
                wait(200)
                integral = 0
                last_error = 0
                
            elif adjustment_type == 'too_far':
                # Too far from wall (wall on LEFT, robot on RIGHT)
                # Need to turn LEFT slightly to get closer to wall
                # But be careful - we're moving clockwise, so small left turn to close gap
                print("Too far (" + str(int(check_distance)) + "mm), turning LEFT slightly to approach wall...")
                turn_in_place_pid(-10, speed=TURN_SPEED * 0.7)  # Small left turn toward wall
                wait(200)
                # Move forward a bit to get closer
                drive_straight_pid(50, speed=DRIVE_SPEED * 0.6)
                wait(200)
                
            elif adjustment_type == 'corner_detected':
                # At a corner or wall end - wall likely turns right (90° corner)
                # Need aggressive RIGHT turn to follow wall around corner
                print("Corner detected (wall likely turns right), turning RIGHT 60°...")
                # Turn RIGHT (clockwise) to follow wall around corner
                turn_in_place_pid(60, speed=TURN_SPEED * 0.8)
                wait(200)
                # Move forward a bit to clear corner and re-establish wall following
                drive_straight_pid(80, speed=DRIVE_SPEED * 0.6)
                wait(200)
                integral = 0
                last_error = 0
        
        # Check if back near hit point
        dist_to_hit = distance_to_point(hit_point_x, hit_point_y)
        
        if initial_distance_to_hit is None:
            initial_distance_to_hit = dist_to_hit
            print("Initial distance to hit point: " + str(int(dist_to_hit)) + " mm")
        
        # Track distances
        if dist_to_hit < min_distance_seen:
            min_distance_seen = dist_to_hit
        if dist_to_hit > max_distance_from_hit:
            max_distance_from_hit = dist_to_hit
        
        # Check if we've returned to hit point
        if dist_to_hit < HIT_POINT_TOLERANCE_MM:
            if step_count > 20 and max_distance_from_hit > initial_distance_to_hit + 200:
                print("Back near hit point! Distance: " + str(int(dist_to_hit)) + " mm")
                print("Steps taken: " + str(step_count))
                print("Minimum distance: " + str(int(min_distance_seen)) + " mm")
                print("Maximum distance: " + str(int(max_distance_from_hit)) + " mm")
                break
        
        # Display status
        if step_count % 5 == 0:
            ev3.screen.clear()
            ev3.screen.draw_text(5, 5, "Step: " + str(step_count))
            if check_distance is not None:
                ev3.screen.draw_text(5, 25, "Dist: " + str(int(check_distance)))
            else:
                ev3.screen.draw_text(5, 25, "Dist: N/A")
            ev3.screen.draw_text(5, 45, "To hit: " + str(int(dist_to_hit)))
            
            # Display sensor status
            left_status = "L" if touch_left.pressed() else " "
            right_status = "R" if touch_right.pressed() else " "
            ev3.screen.draw_text(5, 65, "Sensors: " + left_status + right_status)
        
        wait(100)  # Brief pause between steps
    
    # Stop motors
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    update_odometry()
    print("Wall following complete! Total steps: " + str(step_count))


def navigate_back_to_start():
    """
    Navigate back to start position (2.0 m, 0.5 m) using odometry.
    """
    print("="*50)
    print("NAVIGATING BACK TO START")
    print("="*50)
    
    # Update odometry
    update_odometry()
    
    # Calculate distance and angle to start
    dx = START_POINT_X_MM - robot_x
    dy = START_POINT_Y_MM - robot_y
    distance_to_start = math.sqrt(dx*dx + dy*dy)
    target_heading = math.degrees(math.atan2(dy, dx))
    
    print("Current position: (" + str(int(robot_x)) + ", " + str(int(robot_y)) + ") mm")
    print("Current heading: " + str(int(robot_heading)) + "°")
    print("Distance to start: " + str(int(distance_to_start)) + " mm")
    print("Target heading: " + str(int(target_heading)) + "°")
    
    # Calculate heading error and normalize
    heading_error = target_heading - robot_heading
    heading_error = normalize_angle(heading_error)
    
    # Turn to face start position
    if abs(heading_error) > 5:
        print("Turning " + str(int(heading_error)) + "° toward start...")
        turn_in_place_pid(heading_error, speed=TURN_SPEED)
        wait(200)
    else:
        print("Already facing start direction")
    
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
    """
    try:
        # ========== Startup ==========
        ev3.speaker.beep()
        print("="*50)
        print("LAB 3: Boundary Tracing and Return to Start")
        print("="*50)
        print("Starting position: (" + str(int(robot_x)) + ", " + str(int(robot_y)) + ") mm")
        print("Press CENTER to start...")
        
        # Wait for button press
        while True:
            if Button.CENTER in ev3.buttons.pressed():
                break
            wait(10)
        
        ev3.speaker.beep()
        wait(1000)
        
        # ========== Phase 1: Drive Forward Until Obstacle Detected ==========
        print("")
        print("="*50)
        print("PHASE 1: Drive Forward Until Obstacle Detected")
        print("="*50)
        
        if not drive_until_obstacle_detected(speed=DRIVE_SPEED):
            print("ERROR: Failed to detect obstacle!")
            return
        
        wait(500)
        
        # ========== Phase 2: Record Hit Point and Back Up ==========
        print("")
        print("="*50)
        print("PHASE 2: Record Hit Point and Back Up")
        print("="*50)
        
        # Record hit point at detection point
        # According to spec: "The point directly in front of your robot and 20 cm away 
        # from the obstacle's front wall will be known as the hit point."
        # We record at detection, then back up 20cm to the actual hit point location
        hit_point_x = robot_x
        hit_point_y = robot_y
        hit_point_heading = robot_heading
        
        print("Hit point recorded at detection:")
        print("  Position: (" + str(int(hit_point_x)) + ", " + str(int(hit_point_y)) + ") mm")
        print("  Heading: " + str(int(hit_point_heading)) + "°")
        
        # Back away from obstacle to reach actual hit point (20cm from obstacle front wall)
        print("Backing up " + str(BACKUP_DISTANCE_MM) + " mm to hit point location...")
        drive_straight_pid(-BACKUP_DISTANCE_MM, speed=DRIVE_SPEED)
        wait(500)
        
        # Update hit point to actual location (after backing up)
        update_odometry()
        hit_point_x = robot_x
        hit_point_y = robot_y
        hit_point_heading = robot_heading
        print("Hit point updated to actual location:")
        print("  Position: (" + str(int(hit_point_x)) + ", " + str(int(hit_point_y)) + ") mm")
        print("  Heading: " + str(int(hit_point_heading)) + "°")
        
        # ========== Phase 3: Turn Right 90° ==========
        print("")
        print("="*50)
        print("PHASE 3: Turn Right 90°")
        print("="*50)
        
        print("Turning right 90 degrees...")
        turn_in_place_pid(90, speed=TURN_SPEED)  # Positive = clockwise (right)
        wait(500)
        
        # Reset gyro to establish new "forward" direction (parallel to wall)
        print("Resetting gyro to 0...")
        gyro.reset_angle(0)
        wait(300)
        update_odometry()
        
        ev3.speaker.beep()
        print("Turn complete! New heading: " + str(int(robot_heading)) + "°")
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
        
        print("Turning away from wall (right 90 degrees)...")
        turn_in_place_pid(90, speed=TURN_SPEED)  # Turn right (away from wall)
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
        # ========== Error Handling ==========
        print("")
        print("="*50)
        print("ERROR!")
        print("="*50)
        print("Error: " + str(e))
        import traceback
        traceback.print_exc()
        
        # Emergency stop
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        
        # Play error beeps
        ev3.speaker.beep(frequency=400, duration=300)
        wait(200)
        ev3.speaker.beep(frequency=400, duration=300)


# ============================ RUN PROGRAM =============================

if __name__ == "__main__":
    main()
