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
MAX_WALL_DISTANCE_MM = 280    # Maximum allowed distance (28 cm requirement)
HIT_POINT_TOLERANCE_MM = 100  # How close to be considered "back at hit point"
OBSTACLE_DETECTION_DISTANCE_MM = 300  # Distance to detect obstacle (30 cm)
CORNER_DISTANCE_TOLERANCE_MM = 80     # 将最后一次正常距离视为拐角的容差

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
last_valid_wall_distance = TARGET_WALL_DISTANCE_MM  # 记录最近一次可靠的墙距读数

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
        base_left_speed = speed - correction
        base_right_speed = speed + correction
        left_speed = base_left_speed * direction
        right_speed = base_right_speed * direction
        
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

## NOTE: An alternative PID-based turn_in_place implementation used for experimentation
## was previously left here at module scope with the function header commented out.
## It has been removed to prevent executing control code at import time.

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
            return True
        
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
    简化的沿墙碰撞处理：后退，然后向右旋转90度。

    用于沿墙过程中任何触碰传感器触发的情形，统一采取相同行为，
    以确保稳定继续逆时针贴左墙绕行。

    Returns:
        True after performing the recovery motion.
    """
    # 停车
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(50)

    # 统一后退距离（可按需要微调）
    BACKUP_MM = 150
    drive_straight_pid(-BACKUP_MM, speed=DRIVE_SPEED * 0.7)
    wait(150)

    # 向右旋转90度（顺时针）
    turn_in_place_simple(90, speed=TURN_SPEED)
    wait(120)

    print("Collision recovery: backed up and turned RIGHT 90°")
    return True

# 这个方法是根据机器人与墙的相对位置来判断应该采取哪种恢复策略, 也就是恢复策略的判断依据
def check_pose_intelligent():
    """
    智能姿态检测，采用多种方法综合判断：
    1. 检查超声波测距（处理无穷大/超出范围的情况）
    2. 检查碰撞传感器（触碰开关）
    3. 前进/后退小距离探测墙体位置
    4. 分析机器人朝向与角度信息

    返回值:
        (是否需要调整, 调整类型, 距离值)
        调整类型包括: 'too_close'(距离太近), 'too_far'(距离太远), 'corner_detected'(检测到转角), 'collision'(发生碰撞), None(不需要调整)
    """
    global last_valid_wall_distance
    update_odometry()  # 中文：先更新里程计，确保机器人位置信息是最新的
    
    # 中文：第一步，先检测碰撞传感器（左右两个按钮），优先级最高
    left_pressed = touch_left.pressed()
    right_pressed = touch_right.pressed()
    
    if left_pressed or right_pressed:
        # 中文：只要有一个碰撞传感器被触发，说明发生碰撞，立即返回需要调整类型为“collision”
        return (True, 'collision', None)
    
    # 中文：第二步，读取多次超声波测距，取平均值，过滤噪音
    distances = []
    for i in range(7):
        try:
            dist = ultrasonic.distance()
            if dist > 0 and dist <= 2000:
                distances.append(dist)   # 中文：只收集有效（大于0，小于2000mm）的数据
        except:
            # 中文：如果测距异常（如传感器抖动），跳过本次
            pass
        wait(10)  # 中文：每次采集间间隔10ms
    
    if len(distances) == 0:
        # 中文：连续多次都无法读取有效距离，极有可能在拐角或者传感器异常
        return (True, 'corner_detected', last_valid_wall_distance)
    
    avg_distance = sum(distances) / len(distances)  # 中文：有效测距的均值
    last_valid_wall_distance = avg_distance
    
    # 中文：第三步，检查距离超范围（比如无穷大或者小于0），属于拐角或墙体尽头
    if avg_distance > 2000 or avg_distance < 0:
        print("Distance out of range (" + str(int(avg_distance)) + "mm) - corner detected")
        return (True, 'corner_detected', last_valid_wall_distance)
    
    # 中文：第四步，判断是否太近（距离小于目标距离-50mm）、太远（大于最大设定距离）
    if avg_distance < TARGET_WALL_DISTANCE_MM - 50:  # 中文：距离目标墙小于15厘米，太近
        return (True, 'too_close', avg_distance)
    elif avg_distance > MAX_WALL_DISTANCE_MM:        # 中文：距离目标墙大于28厘米，太远
        return (True, 'too_far', avg_distance)
    
    # 中文：第五步，主动前探——机器人向前探测30mm，再判断距离变化，用于辅助检测拐角
    initial_distance = avg_distance    # 中文：记录初始距离，后面用于计算变化
    initial_gyro = gyro.angle()        # 中文：记录初始角度，便于直行修正
    
    # 中文：复位电机编码器，准备前行
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    probe_distance = 50
    target_rotation = (probe_distance / WHEEL_CIRCUMFERENCE_MM) * 360  # 中文：把前行距离转换成编码器角度
    
    while True:
        # 中文：前探过程中随时检测是否发生碰撞
        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            # 中文：前探时如果撞上障碍，立刻停止、后退
            drive_straight_pid(-probe_distance, speed=DRIVE_SPEED * 0.6)
            return (True, 'collision', None)
        
        avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        # 中文：已经前进到指定距离，跳出循环
        if avg_rotation >= target_rotation:
            break
        
        # 中文：用陀螺仪做直行校正，避免探测歪斜
        gyro_error = gyro.angle() - initial_gyro
        correction = 2.0 * gyro_error
        left_motor.run(DRIVE_SPEED * 0.6 - correction)
        right_motor.run(DRIVE_SPEED * 0.6 + correction)
        wait(10)
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)  # 中文：短暂停止，等待惯性消失
    
    # 中文：第六步，前探后再次测距，分析距离变化幅度，如果变化剧烈说明前面是拐角
    try:
        probe_distance_reading = ultrasonic.distance()
        if probe_distance_reading > 0 and probe_distance_reading <= 2000:
            distance_change = abs(probe_distance_reading - initial_distance)
            
            # 中文：如果距离突然变化超过100mm，判定为拐角。先退回原位，再报告“corner_detected”
            if distance_change > 100:
                # 中文：退回到原位
                drive_straight_pid(-probe_distance, speed=DRIVE_SPEED * 0.6)
                last_valid_wall_distance = initial_distance
                return (True, 'corner_detected', last_valid_wall_distance)
    except:
        # 中文：如果探测超声波异常，忽略
        pass
    
    # 中文：最后一步，不论前探测出什么，都要退回原位置，保证机器人实际位置不变
    drive_straight_pid(-probe_distance, speed=DRIVE_SPEED * 0.6)
    wait(100)
    
    # 中文：最终判断为无需调整，返回False和当前平均距离
    last_valid_wall_distance = avg_distance
    return (False, None, avg_distance)

# 这个方法是根据机器人与墙的相对位置来判断应该采取哪种恢复策略, 也就是恢复策略的判断依据
def check_pose_and_adjust():
    """
    快速姿态检查，仅返回是否需要立即恢复的标志，不直接执行动作。
    返回:
        (needs_adjust, reason, distance_mm)
        reason ∈ {'collision', 'too_close', None}
    """
    global last_valid_wall_distance
    update_odometry()
    
    left_pressed = touch_left.pressed()
    right_pressed = touch_right.pressed()
    
    if left_pressed or right_pressed:
        return (True, 'collision', None)
    
    try:
        distance = ultrasonic.distance()
        if distance <= 0 or distance > 2000:
            return (False, None, None)
    except:
        return (False, None, None)
    
    last_valid_wall_distance = distance
    if distance < TARGET_WALL_DISTANCE_MM - 50:
        return (True, 'too_close', distance)
    
    return (False, None, distance)


def apply_wall_adjustment(adjustment_type, measured_distance=None):
    """
    根据姿态检查结果执行相应的恢复/调整动作。
    返回True表示确实执行了调整。
    """
    if adjustment_type == 'collision':
        print("Applying collision recovery...")
        handle_collision_recovery_intelligent()
        return True
    
    if adjustment_type == 'too_close':
        if measured_distance is not None:
            print("Adjusting: too close to wall (" + str(int(measured_distance)) + "mm)")
        else:
            print("Adjusting: too close to wall")
        drive_straight_pid(-80, speed=DRIVE_SPEED * 0.6)
        wait(200)
        turn_in_place_simple(25, speed=TURN_SPEED * 0.6)
        wait(150)
        return True
    
    if adjustment_type == 'too_far':
        if measured_distance is not None:
            print("Adjusting: too far from wall (" + str(int(measured_distance)) + "mm)")
        else:
            print("Adjusting: too far from wall")
        turn_in_place_simple(-20, speed=TURN_SPEED * 0.7)
        wait(200)
        drive_straight_pid(50, speed=DRIVE_SPEED * 0.6)
        wait(200)
        return True
    
    global last_valid_wall_distance
    if adjustment_type == 'corner_detected':
        print("Adjusting: corner detected, handling turn...")
        left_pressed = touch_left.pressed()
        right_pressed = touch_right.pressed()
        effective_distance = measured_distance if measured_distance is not None else last_valid_wall_distance
        last_valid_wall_distance = effective_distance
        if left_pressed or right_pressed:
            drive_straight_pid(-100, speed=DRIVE_SPEED * 0.7)
            wait(200)
            turn_in_place_simple(60, speed=TURN_SPEED * 0.8)
            wait(200)
            drive_straight_pid(80, speed=DRIVE_SPEED * 0.6)
        else:
            if abs(effective_distance - TARGET_WALL_DISTANCE_MM) <= CORNER_DISTANCE_TOLERANCE_MM:
                print("Corner bypass using steady distance " + str(int(effective_distance)) + "mm")
                drive_straight_pid(80, speed=DRIVE_SPEED * 0.7)
                wait(200)
                turn_in_place_simple(-85, speed=TURN_SPEED * 0.8)
                wait(200)
                drive_straight_pid(60, speed=DRIVE_SPEED * 0.6)
                wait(200)
                return True
            drive_straight_pid(120, speed=DRIVE_SPEED * 0.7)
            wait(200)
            for i in range(3):
                try:
                    distance_left = ultrasonic.distance()
                except:
                    distance_left = None
                
                if distance_left is not None and distance_left < TARGET_WALL_DISTANCE_MM + 40:
                    turn_in_place_simple(-80, speed=TURN_SPEED * 0.7)
                    wait(200)
                    break
                wait(100)
            else:
                turn_in_place_simple(-50, speed=TURN_SPEED * 0.7)
                wait(200)
        return True
    
    return False


def assess_and_correct_pose(run_deep_check=True):
    """
    组合快速检测与智能检测；可根据 run_deep_check 决定是否执行深度探测。
    返回:
        (did_adjust, adjustment_reason, measured_distance)
    """
    quick_needs, quick_reason, quick_distance = check_pose_and_adjust()
    if quick_needs:
        apply_wall_adjustment(quick_reason, quick_distance)
        return (True, quick_reason, quick_distance)
    
    if not run_deep_check:
        return (False, None, quick_distance)
    
    needs_adjust, adjustment_type, check_distance = check_pose_intelligent()
    if needs_adjust:
        apply_wall_adjustment(adjustment_type, check_distance)
        return (True, adjustment_type, check_distance)
    
    return (False, None, check_distance)


def prepare_wall_following(max_attempts=3):
    """
    在开始沿墙前，预先进行若干次姿态检查，确保距离墙体稳定。
    """
    attempts = 0
    while attempts < max_attempts:
        adjusted, reason, _ = assess_and_correct_pose(run_deep_check=(attempts == 0))
        if not adjusted:
            print("Wall follow prep: pose looks good.")
            return
        attempts += 1
        print("Wall follow prep: adjustment (" + str(reason) + ") applied, rechecking...")
        wait(200)
    print("Wall follow prep: reached max adjustments, proceeding with caution.")

# 沿着墙走，直到接近hit point, 核心的算法部分

def follow_wall_until_hit_point(hit_point_x, hit_point_y, target_distance_mm=TARGET_WALL_DISTANCE_MM, speed=DRIVE_SPEED):
    """
    沿墙前进直到回到hit point，用走一步检查一步的“死算”方式，防止累积误差
    """

    global last_valid_wall_distance
    # 初始化PID相关变量（所有增益都较弱，防止过度反应）
    integral = 0
    last_error = 0
    last_distance = target_distance_mm
    ALPHA = 0.5  # 滤波参数，用于距离平滑

    # 重置左右轮编码器
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    step_count = 0  # 步数计数
    min_distance_seen = float('inf')  # 跟踪离hit点最近的距离
    initial_distance_to_hit = None    # 首次记录距离hit点的位置
    max_distance_from_hit = 0         # 记录离hit点最远的距离

    while True:
        step_count += 1  # 步号+1

        # ========== 步骤1：走一步 ==========
        # 计算这一步需要转多少度（将距离转换为电机角度）
        target_rotation = (STEP_DISTANCE_MM / WHEEL_CIRCUMFERENCE_MM) * 360

        # 读取超声波测距，做平滑处理（防止噪声带来的大跳变）
        try:
            raw_distance = ultrasonic.distance()
            if raw_distance <= 0 or raw_distance > 2000:
                # 距离异常就用上一次的
                current_distance = last_distance
            else:
                current_distance = ALPHA * raw_distance + (1 - ALPHA) * last_distance
                last_valid_wall_distance = current_distance
        except:
            # 读取异常也用上次的
            current_distance = last_distance

        last_distance = current_distance  # 存档本次测距

        # 计算距离误差，准备PID
        error = target_distance_mm - current_distance

        # PID控制部分（参数都取较小，防止突兀修正）
        p = WALL_KP * error
        integral += error * 0.1  # 积分项带缩放，防积分爆炸
        integral = max(-20, min(20, integral))  # 限制积分项范围
        i = WALL_KI * integral
        derivative = (error - last_error) / 1.0
        d = WALL_KD * derivative
        last_error = error

        correction = p + i + d
        # 再限制修正量，放缓反应速度
        correction = max(-60, min(60, correction))

        # 按照PID调整之后的左右轮速度
        # 如果太靠近墙壁（error>0，correction>0），左电机快，机器人右转，远离墙
        # 如果太远，则右轮快，机器人左转，靠近墙
        left_speed = speed + correction
        right_speed = speed - correction

        # 限制实际速度（安全+防止速度过大形变）
        min_speed = 50
        max_speed = speed * 1.4
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))

        # 每走一步都清零电机转角，记录起始陀螺仪角度
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        initial_gyro = gyro.angle()

        while True:
            # 检查碰撞（触碰传感器是否按下）
            left_pressed = touch_left.pressed()
            right_pressed = touch_right.pressed()

            if left_pressed or right_pressed:
                left_motor.stop(Stop.BRAKE)
                right_motor.stop(Stop.BRAKE)
                wait(100)

                # 智能碰撞恢复（根据哪个传感器撞到来调整）
                handle_collision_recovery_intelligent()
                integral = 0
                last_error = 0
                break

            # 用陀螺仪纠偏直线（防止偏航）
            gyro_error = gyro.angle() - initial_gyro
            gyro_correction = 2.0 * gyro_error  # 校准比例
            gyro_correction = max(-20, min(20, gyro_correction))  # 避免修正过大

            left_step_speed = left_speed - gyro_correction
            right_step_speed = right_speed + gyro_correction

            # 运行电机前进一步
            left_motor.run(left_step_speed)
            right_motor.run(right_step_speed)

            # 判断走的距离是否达到本步目标
            avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
            if avg_rotation >= target_rotation:
                left_motor.stop(Stop.BRAKE)
                right_motor.stop(Stop.BRAKE)
                break

            wait(10)  # 循环检测响应快一些

        # 步进完成后短暂停顿
        wait(50)

        # ========== 步骤2：姿态检查和必要调整 ==========
        run_deep_check = (step_count == 1) or (step_count % STEP_CHECK_INTERVAL == 0)
        adjusted, adjustment_reason, check_distance = assess_and_correct_pose(run_deep_check=run_deep_check)
        if adjusted:
            integral = 0
            last_error = 0
            last_distance = target_distance_mm
            wait(150)
            continue

        if check_distance is None:
            check_distance = current_distance

        # ========== 步骤3：判断是否回到hit_point ==========
        dist_to_hit = distance_to_point(hit_point_x, hit_point_y)

        # 初次记录离hit点的距离，用于判断是否真的绕了一圈
        if initial_distance_to_hit is None:
            initial_distance_to_hit = dist_to_hit
            print("初始距离hit点：" + str(int(dist_to_hit)) + " mm")

        # 记录离hit点的最近和最远处
        if dist_to_hit < min_distance_seen:
            min_distance_seen = dist_to_hit
        if dist_to_hit > max_distance_from_hit:
            max_distance_from_hit = dist_to_hit

        # 如果距离足够近（且已绕墙远走过一段），就判定为走完一圈
        if dist_to_hit < HIT_POINT_TOLERANCE_MM:
            if step_count > 20 and max_distance_from_hit > initial_distance_to_hit + 200:
                print("已回到hit点！距离为：" + str(int(dist_to_hit)) + " mm")
                print("总步数：" + str(step_count))
                print("最小离hit点距离：" + str(int(min_distance_seen)) + " mm")
                print("最大离hit点距离：" + str(int(max_distance_from_hit)) + " mm")
                break

        # ========== 步骤4：每隔5步更新界面，显示进展和传感器状态 ==========
        if step_count % 5 == 0:
            ev3.screen.clear()
            ev3.screen.draw_text(5, 5, "Step: " + str(step_count))
            if check_distance is not None:
                ev3.screen.draw_text(5, 25, "Dist: " + str(int(check_distance)))
            else:
                ev3.screen.draw_text(5, 25, "Dist: N/A")
            ev3.screen.draw_text(5, 45, "To hit: " + str(int(dist_to_hit)))

            # 显示触碰传感器状态
            left_status = "L" if touch_left.pressed() else " "
            right_status = "R" if touch_right.pressed() else " "
            ev3.screen.draw_text(5, 65, "Sensors: " + left_status + right_status)

        wait(100)  # 步与步之间短暂延时

    # 停止电机，并更新里程计，最终统计
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    update_odometry()
    print("已经完成沿墙一圈，总步数：" + str(step_count))


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
        turn_in_place_simple(heading_error, speed=TURN_SPEED)
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

        
        # Wait for button press
        while True:
            if Button.CENTER in ev3.buttons.pressed():
                break
            wait(10)
        
        ev3.speaker.beep()
        wait(1000)
        
        # ========== Phase 1: Drive Forward Until Obstacle Detected ==========

        
        if not drive_until_obstacle_detected(speed=DRIVE_SPEED):
            print("ERROR: Failed to detect obstacle!")
            return
        
        wait(500)
        
        # ========== Phase 2: Record Hit Point and Back Up ==========

        
        # Record hit point at detection point
        # According to spec: "The point directly in front of your robot and 20 cm away 
        # from the obstacle's front wall will be known as the hit point."
        # We record at detection, then back up 20cm to the actual hit point location
        hit_point_x = robot_x
        hit_point_y = robot_y
        hit_point_heading = robot_heading
        

        
        # Back away from obstacle to reach actual hit point (20cm from obstacle front wall)
        drive_straight_pid(-BACKUP_DISTANCE_MM, speed=DRIVE_SPEED)
        wait(500)
        
        # Update hit point to actual location (after backing up)
        update_odometry()
        hit_point_x = robot_x
        hit_point_y = robot_y
        hit_point_heading = robot_heading

        
        # ========== Phase 3: Turn Right 90° ==========

        
        print("Turning right 90 degrees...")
        #turn_in_place_pid(90, speed=TURN_SPEED)  # Positive = clockwise (right)
        turn_in_place_simple(90, speed=TURN_SPEED)  # Positive = clockwise (right)
        wait(500)
        
        # Reset gyro to establish new "forward" direction (parallel to wall)
        print("Resetting gyro to 0...")
        gyro.reset_angle(0)
        wait(300)
        update_odometry()
        prepare_wall_following()
        
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

        #turn_in_place_pid(90, speed=TURN_SPEED)  # Turn right (away from wall)
        turn_in_place_simple(90, speed=TURN_SPEED)  # Turn right (away from wall)
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
