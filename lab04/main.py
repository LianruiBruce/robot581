#!/usr/bin/env pybricks-micropython
# Team Members: Lianrui Geng && Xinyi Guo
# Lab 03  BOUNDARY TRACING AND RETURN TO START + Bug2 Algorithm
#
# This program implements:
#   - Lab 3: Boundary Tracing & Return to Start (original logic, kept intact)
#   - Bug2 Algorithm from (0.5m, 0) to (2.5m, 2.5m) using the same sensors,
#     odometry, and left-side wall following style.
#
# Bug2 Task:
#   Start at (0.5 m, 0.0 m)  -> (500 mm, 0 mm)
#   Goal at (2.5 m, 2.5 m)   -> (2500 mm, 2500 mm)
#
# Bug2 Strategy:
#   1. Move along M-line (straight line from start to goal).
#   2. If no obstacle => reach goal and stop.
#   3. If obstacle hit:
#       - Record hit point.
#       - Use left-wall following (same style as Lab3) to trace boundary.
#       - When robot returns to M-line at a point closer to goal than before,
#         leave the obstacle and continue along M-line.
#       - If robot comes back to the same hit point: goal unreachable.

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
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM

# Movement Parameters
DRIVE_SPEED = 180             # Motor speed in degrees per second for forward motion
TURN_SPEED = 150               # Motor speed in degrees per second for turning

# Lab 3 Specific Parameters
BACKUP_DISTANCE_MM = 100      # Distance to back away from obstacle (~18 cm)
TARGET_WALL_DISTANCE_MM = 100 # Target distance from wall during following (15 cm)
MAX_WALL_DISTANCE_MM = 180   # Maximum allowed distance (~23 cm)
HIT_POINT_TOLERANCE_MM = 100  # How close to be considered "back at hit point"
OBSTACLE_DETECTION_DISTANCE_MM = 350  # Distance to detect obstacle (35 cm)
CORNER_DISTANCE_TOLERANCE_MM = 80     # 将最后一次正常距离视为拐角的容差
FAKE_WALL_DISTANCE_MM = 330           # 用于拐角绕行的假设墙距（25 cm）
FAKE_WALL_DISTANCE_MAX_MM = 360       # 用于拐角绕行的假设墙距最大值（40 cm）
LEFT_CORNER_GAP      = 450         # mm: d_s must exceed target by this much
LEFT_CORNER_DE_DOT   = 500.0          # mm/s: d_s must be increasing at least this fast
K_FAR = 10
K_CORNER = 1
MAX_D_STEP = 40

# Hit Point Definition (from lab requirements)
HIT_POINT_X_MM = 2000.0     # 2.0 m
HIT_POINT_Y_MM = 500.0      # 0.5 m

# Wall Following PID Parameters
WALL_KP = 1.1
WALL_KI = 0.002
WALL_KD = 1.2


# Straight Drive PID Parameters
GYRO_KP = 2.0                 # Gyro correction PID gains
GYRO_KI = 0
GYRO_KD = 1

# Turn Control Parameters
COARSE_KP = 2.5               # Coarse turn proportional gain
FINE_KP = 5.0                 # Fine turn PID gains
FINE_KI = 0.08
FINE_KD = 3.0

# ---------------------- Bug2-specific parameters ------------------------
GOAL_TOLERANCE_MM = 200.0       # Within 10cm of goal => success
M_LINE_THRESHOLD_MM = 100      # Within 8cm of M-line => considered "on M-line"

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
robot_x = 0.0                 # X position in mm
robot_y = 0.0                 # Y position in mm
robot_heading = 90.0           # Heading in degrees (0 = +X direction)
gyro_offset = 0.0             # Offset to handle gyro resets while maintaining global heading
last_left_angle = 0           # Last left motor encoder reading
last_right_angle = 0          # Last right motor encoder reading
last_valid_wall_distance = TARGET_WALL_DISTANCE_MM  # 记录最近一次可靠的墙距读数

# Starting position (星星位置)
start_point_x = 0.0
start_point_y = 0.0

# Reset motor encoders and sync with odometry
left_motor.reset_angle(0)
right_motor.reset_angle(0)
last_left_angle = 0
last_right_angle = 0

# Bug2 wall-follow PID state
bug2_wall_integral = 0.0
bug2_wall_last_error = 0.0
bug2_wall_last_distance = TARGET_WALL_DISTANCE_MM

# ============================ HELPER FUNCTIONS =============================

def reset_and_sync_encoders():
    """
    重置电机编码器并同步里程计变量。
    """
    global last_left_angle, last_right_angle
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    last_left_angle = 0
    last_right_angle = 0


def normalize_angle(angle_deg):
    """
    Normalize angle to -180 to 180 degree range.
    """
    angle_deg = angle_deg % 360
    if angle_deg > 180:
        angle_deg -= 360
    return angle_deg


def update_odometry():
    """
    Update robot's position using wheel odometry (differential drive kinematics).
    Should be called regularly during movement to maintain accurate position tracking.
    """
    global robot_x, robot_y, robot_heading, last_left_angle, last_right_angle, gyro_offset
    
    left_angle = left_motor.angle()
    right_angle = right_motor.angle()
    
    left_delta = left_angle - last_left_angle
    right_delta = right_angle - last_right_angle
    
    left_distance = (left_delta / 360.0) * WHEEL_CIRCUMFERENCE_MM
    right_distance = (right_delta / 360.0) * WHEEL_CIRCUMFERENCE_MM
    
    last_left_angle = left_angle
    last_right_angle = right_angle
    
    forward_distance = (left_distance + right_distance) / 2.0
    
    # 全局航向角：注意这里使用 -gyro.angle() + offset
    robot_heading = -gyro.angle() + gyro_offset
    print("robot_heading: ", robot_heading)
    heading_rad = math.radians(robot_heading)
    
    robot_x += forward_distance * math.cos(heading_rad)
    robot_y += forward_distance * math.sin(heading_rad)
    
    return (robot_x, robot_y, robot_heading)


def distance_to_point(x, y):
    """Calculate Euclidean distance from current position to target point."""
    dx = robot_x - x
    dy = robot_y - y
    return math.sqrt(dx*dx + dy*dy)


def drive_straight_pid(distance_mm, speed=DRIVE_SPEED):
    """
    Drive straight for a specific distance using gyro PID control.
    Positive distance: forward; negative: backward.
    """
    target_rotation = (abs(distance_mm) / WHEEL_CIRCUMFERENCE_MM) * 360
    direction = 1 if distance_mm > 0 else -1
    
    update_odometry()
    reset_and_sync_encoders()
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
        
        if direction == 1:
            left_speed = speed - correction
            right_speed = speed + correction
        else:
            left_speed = -(speed + correction)
            right_speed = -(speed - correction)
        
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        wait(20)
        
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(10)
    update_odometry()


def turn_in_place_simple(angle_degrees, speed=TURN_SPEED):
    """
    Simple turn-in-place using gyro feedback.
    Positive angle_degrees = clockwise(right), negative = counterclockwise(left).
    """
    initial_gyro = gyro.angle()
    target_gyro = initial_gyro + angle_degrees

    def normalize_angle_simple(deg):
        while deg > 180:
            deg -= 360
        while deg < -180:
            deg += 360
        return deg

    Kp = 4
    Ki = 0.02
    Kd = 0.5

    integral = 0
    last_error = 0

    while True:
        current_gyro = gyro.angle()
        error = normalize_angle_simple(target_gyro - current_gyro)
        if abs(error) < 2:
            break

        p = Kp * error
        integral += error * 0.02
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
    update_odometry()

def handle_collision_recovery_intelligent():
    """
    简化的沿墙碰撞处理：后退，然后向右旋转70度。
    用于沿墙过程中任何触碰传感器触发的情形。
    """
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(50)

    BACKUP_MM = 100
    drive_straight_pid(-BACKUP_MM, speed=DRIVE_SPEED * 0.7)
    wait(150)

    turn_in_place_simple(70, speed=TURN_SPEED)
    update_odometry()
    wait(120)

    print("Collision recovery: backed up and turned RIGHT 70°")
    return True


# ======================= Bug2 工具函数（新增） =======================

def point_line_distance(px, py, x1, y1, x2, y2):
    """
    计算点 (px, py) 到线段 (x1, y1)-(x2, y2) 的最短距离。
    之前版本把 M-line 当作无限长直线；现在将投影约束在 [start, goal] 之间。
    """
    A = px - x1
    B = py - y1
    C = x2 - x1
    D = y2 - y1

    len_sq = C * C + D * D
    if len_sq == 0:
        return math.sqrt(A * A + B * B)

    param = max(0.0, min(1.0, (A * C + B * D) / len_sq))
    xx = x1 + param * C
    yy = y1 + param * D

    dx = px - xx
    dy = py - yy
    return math.sqrt(dx * dx + dy * dy)


def on_m_line_and_closer(robot_x, robot_y,
                         start_x, start_y,
                         goal_x, goal_y,
                         last_min_dist_to_goal,
                         m_line_threshold=M_LINE_THRESHOLD_MM):
    """
    判断：
      1. 当前点是否在 M-line 附近（距离直线 < m_line_threshold）。
      2. 当前点是否比之前任何 M-line 上的点更接近目标。

    返回:
        (should_leave_obstacle, updated_min_dist_to_goal)
    """
    dist_line = point_line_distance(robot_x, robot_y,
                                    start_x, start_y,
                                    goal_x, goal_y)
    print("dist to line: ", dist_line)

    if dist_line > m_line_threshold:
        return False, last_min_dist_to_goal

    dx = goal_x - robot_x
    dy = goal_y - robot_y
    dist_goal = math.sqrt(dx*dx + dy*dy)

    # 只有当更接近目标时才更新
    if dist_goal + 5 < last_min_dist_to_goal:
        return True, dist_goal
    
    return False, last_min_dist_to_goal


def drive_towards_goal(goal_x, goal_y, m_line_heading_deg, speed=DRIVE_SPEED):
    """
    沿 M-line 方向向目标前进：
    - 使用陀螺仪保持朝向 m_line_heading_deg。
    - 如果到达目标区域 => 返回 "goal"
    - 如果碰到障碍物（触碰）=> 返回 "obstacle"
    """
    update_odometry()

    dx = goal_x - robot_x
    dy = goal_y - robot_y
    dist_goal = math.sqrt(dx*dx + dy*dy)
    if dist_goal <= GOAL_TOLERANCE_MM:
        return "goal"

    print("robot_heading: ", robot_heading)
    print("m_line_heading: ", m_line_heading_deg)
    # 转向到 M-line 方向
    heading_error = normalize_angle(robot_heading - m_line_heading_deg)
    if abs(heading_error) > 3:
        turn_in_place_simple(heading_error, speed=TURN_SPEED)
        wait(100)
        update_odometry()

    initial_gyro = gyro.angle()
    GYRO_CORRECTION_KP = 5
    sw = StopWatch()
    sw.reset()
    max_run_time_ms = 180000  # fail-safe，防止无限跑

    while True:
        update_odometry()

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist_goal = math.sqrt(dx*dx + dy*dy)
        if dist_goal <= GOAL_TOLERANCE_MM:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("Bug2: reached goal while driving along M-line.")
            return "goal"

        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            ev3.speaker.beep()
            print("Bug2: obstacle hit while driving along M-line.")
            return "obstacle"

        if sw.time() > max_run_time_ms:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("Bug2: timeout while driving towards goal.")
            return "timeout"
        print("gyro_angle:", gyro.angle())
        gyro_angle_n = normalize_angle(gyro.angle())
        gyro_error = gyro_angle_n - ( - m_line_heading_deg)
        correction = GYRO_CORRECTION_KP * gyro_error
        correction = max(-40, min(40, correction))

        print("correction: ", correction)
        left_motor.run(speed * 1.1 - correction)
        right_motor.run(speed + correction)
        wait(500)

def follow_wall_until_hit_point(goal_x, goal_y, hit_x, hit_y,
                                target_distance_mm=TARGET_WALL_DISTANCE_MM, speed=DRIVE_SPEED,
                                ):
    
    global last_min_goal_dist

    # 初始化PID相关变量（所有增益都较弱，防止过度反应）
    integral = 0
    last_error = 0
    last_distance = target_distance_mm
    ALPHA = 0.5  # 滤波参数，用于距离平滑

    # 清算残余位移，重置并同步编码器
    update_odometry()
    reset_and_sync_encoders()

    step_count = 0  # 步数计数
    continue_far = 0  # 连续过远计数器
    corner_trigger_count = 0
    initial_gyro_angle = gyro.angle()  # 记录初始陀螺仪角度
    
    print("Enter wall following")

    while True:
        step_count += 1  # 步号+1

        # ========== 步骤1：走一步 ==========
        # 计算这一步需要转多少度（将距离转换为电机角度）
        #target_rotation = (STEP_DISTANCE_MM / WHEEL_CIRCUMFERENCE_MM) * 360

        # 读取超声波测距，做平滑处理（防止噪声带来的大跳变）
        try:
            raw_distance = ultrasonic.distance()
            #print("raw_distance: ", raw_distance)
            if raw_distance <= 0:
                # 距离异常就用上一次的
                current_distance = last_distance
                print("raw_distance<=0")
            elif abs(raw_distance - last_distance) > MAX_D_STEP:
                current_distance = last_distance + MAX_D_STEP * (1 if raw_distance > last_distance else -1)
                print("big difference")
                print("current distance: ", current_distance)
            else:
                current_distance = ALPHA * raw_distance + (1 - ALPHA) * last_distance
                last_valid_wall_distance = current_distance
                #print("normal")
        except:
            # 读取异常也用上次的
            current_distance = last_distance

        #print("current_distance:", current_distance)
        error = current_distance - target_distance_mm
        derivative = (current_distance - last_distance) / 0.02

        last_distance = current_distance  # 存档本次测距

        if (current_distance > FAKE_WALL_DISTANCE_MM and current_distance < FAKE_WALL_DISTANCE_MAX_MM) and continue_far < K_FAR:
            continue_far += 1
            current_distance = target_distance_mm - 7  # Gentle correction toward wall
        elif current_distance< FAKE_WALL_DISTANCE_MM or current_distance >= FAKE_WALL_DISTANCE_MAX_MM:
            continue_far = 0  # Reset counter when back in normal range:

        # --- 拐角判断 ---
        if error > LEFT_CORNER_GAP and corner_trigger_count < K_CORNER:
            corner_trigger_count += 1
            current_dance = target_distance_mm
            print("corner_trigger_count:", corner_trigger_count )
        elif error < LEFT_CORNER_GAP:
            corner_trigger_count = 0
        
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
            continue
        #if error > LEFT_CORNER_GAP and derivative > LEFT_CORNER_DE_DOT: 
        if corner_trigger_count >= K_CORNER:
            print("Enter left turn")
            reset_and_sync_encoders()
            curr = gyro.angle() - initial_gyro_angle
            turn_in_place_simple(-40)
            surge_deg = (160 / WHEEL_CIRCUMFERENCE_MM) * 360
            while True:
                avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
                if avg_rotation >= surge_deg:
                    break
                left_motor.run(240)
                right_motor.run(240)
                update_odometry()
                if touch_left.pressed() or touch_right.pressed():
                    left_motor.stop(Stop.BRAKE)
                    right_motor.stop(Stop.BRAKE)
                    update_odometry()
                    wait(100)

                    # 调用你的智能碰撞恢复逻辑
                    handle_collision_recovery_intelligent()
                    break
                wait(10)
            
            corner_trigger_count = 0
            wait(20)
            continue
        
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
        correction = max(-45, min(45, correction))

        # 按照PID调整之后的左右轮速度
        # 如果太靠近墙壁（error>0，correction>0），左电机快，机器人右转，远离墙
        # 如果太远，则右轮快，机器人左转，靠近墙
        left_speed = speed + correction
        right_speed = speed - correction

        # 限制实际速度（安全+防止速度过大形变）
        min_speed = 50
        max_speed = speed * 1.2
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))
        
        # # 检查碰撞（触碰传感器是否按下）
        # left_pressed = touch_left.pressed()
        # right_pressed = touch_right.pressed()

        # if left_pressed or right_pressed:
        #     left_motor.stop(Stop.BRAKE)
        #     right_motor.stop(Stop.BRAKE)
        #     wait(100)

        #     # 智能碰撞恢复（根据哪个传感器撞到来调整）
        #     handle_collision_recovery_intelligent()
        #     integral = 0
        #     last_error = 0
        #     continue

        left_motor.run(left_speed)
        right_motor.run(right_speed)
        # ★ 每步内做里程计积分，捕捉弧线位移
        update_odometry()

        dxg = goal_x - robot_x
        dyg = goal_y - robot_y
        dist_goal = math.sqrt(dxg*dxg + dyg*dyg)
        if dist_goal <= GOAL_TOLERANCE_MM:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            return "goal"
        leave, last_min_goal_dist = on_m_line_and_closer(
                        robot_x, robot_y,
                        start_point_x, start_point_y,
                        goal_x, goal_y,
                        last_min_goal_dist,
                        m_line_threshold=M_LINE_THRESHOLD_MM
                    )
        if leave:
            return "leave"
        
        dist_back_to_hit = math.sqrt((robot_x - hit_x)**2 + (robot_y - hit_y)**2)
        if step_count > 50 and dist_back_to_hit < HIT_POINT_TOLERANCE_MM:
            return "hit_point"
        wait(200)  # 循环检测响应快一些

        if step_count %50 ==0:
            print("robot_x: ", robot_x)
            print("robot_y: ", robot_y)


# ======================= Bug2 主程序（新增） =======================

def main_bug2():
    """
    Bug2 Algorithm 主程序：
    从 (0.5m, 0) 出发，最终停在 (2.5m, 2.5m)。
    使用：
      - M-line 直线前进
      - 左侧贴墙绕障碍物
      - 与原代码一致的里程计更新方式
    """
    global robot_x, robot_y, robot_heading
    global start_point_x, start_point_y
    global gyro_offset
    global bug2_wall_integral, bug2_wall_last_error, bug2_wall_last_distance

    try:
        # 启动提示 & 等待按键
        ev3.speaker.beep()
        while True:
            if Button.CENTER in ev3.buttons.pressed():
                break
            wait(10)
        ev3.speaker.beep()
        wait(500)

        # ------- Bug2 起点与目标 -------
        start_point_x = 500.0   # 0.5 m
        start_point_y = 0.0     # 0.0 m
        robot_x = start_point_x
        robot_y = start_point_y
        robot_heading = 90

        goal_x = 2500.0         # 2.5 m
        goal_y = 2500.0         # 2.5 m

        reset_and_sync_encoders()
        gyro_offset = 0.0
        gyro.reset_angle(-90)
        wait(50)

        print("="*50)
        print("Bug2 Start at (0.5 m, 0.0 m)")
        print("Goal at (2.5 m, 2.5 m)")
        print("Start: ({:.0f}, {:.0f}) mm".format(start_point_x, start_point_y))
        print("="*50)

        global m_line_heading_deg
        # M-line 方向角（整个过程保持不变）
        m_line_heading_deg = math.degrees(
            math.atan2(goal_y - start_point_y, goal_x - start_point_x)
        )
        print("m_line_heading: ", m_line_heading_deg)
       
        global last_min_goal_dist
        # Bug2 中：记录在 M-line 上距目标的“最小距离”
        dx0 = goal_x - start_point_x
        dy0 = goal_y - start_point_y
        last_min_goal_dist = math.sqrt(dx0*dx0 + dy0*dy0)

        while True:
            # 1) 沿 M-line 向目标前进
            print("robot_heading1: ", robot_heading)
            result = drive_towards_goal(goal_x, goal_y, m_line_heading_deg, speed=DRIVE_SPEED)
            
            if result == "goal":
                print("Bug2: Successfully reached goal!")
                for i in range(3):
                    ev3.speaker.beep(frequency=1000 + i*200, duration=150)
                    wait(150)
                print("Final pose: ({:.0f}, {:.0f}) mm, heading {:.1f}°".format(
                    robot_x, robot_y, robot_heading
                ))
                return

            if result == "timeout":
                print("Bug2: timeout while trying to reach goal along M-line. Stopping.")
                ev3.speaker.beep(frequency=400, duration=500)
                return

            # 2) 遇到障碍物 => 记录 hit point，开始绕行
            if result == "obstacle":
                update_odometry()
                hit_x = robot_x
                hit_y = robot_y
                hit_to_goal = math.sqrt((goal_x - hit_x)**2 + (goal_y - hit_y)**2)

                print("Bug2: Obstacle encountered, hit point = ({:.0f}, {:.0f}) mm".format(
                    hit_x, hit_y
                ))

                # 初始时，hit point 上的距离肯定比起点更靠近目标，更新一下面板
                if hit_to_goal < last_min_goal_dist:
                    last_min_goal_dist = hit_to_goal

                # 使用你的智能碰撞恢复逻辑，让墙在左边
                handle_collision_recovery_intelligent()
                update_odometry()

                # 重置贴墙 PID 状态
                bug2_wall_integral = 0.0
                bug2_wall_last_error = 0.0
                bug2_wall_last_distance = TARGET_WALL_DISTANCE_MM

                step_counter = 0

                # 3) 沿墙绕障碍物
                obstacle_result = follow_wall_until_hit_point(
                    goal_x, goal_y,
                    hit_x, hit_y,
                    target_distance_mm=TARGET_WALL_DISTANCE_MM,
                    speed=DRIVE_SPEED)

                    # 绕行过程中也检查是否刚好到达目标
                if obstacle_result == "goal":
                    print("Bug2: reached goal while wall-following!")
                    for i in range(3):
                        ev3.speaker.beep(frequency=1000 + i*200, duration=150)
                        wait(150)
                    return

                    # 检查是否回到了 M-line 且比之前更接近目标 => 可以离开障碍物
                if obstacle_result == "leave":
                    print("Bug2: back on M-line and closer to goal. Leave obstacle.")
                    update_odometry()
                    # 面向 M-line 方向
                    heading_error = normalize_angle(m_line_heading_deg - robot_heading)
                    if abs(heading_error) > 3:
                        turn_in_place_simple(heading_error, speed=TURN_SPEED)
                        wait(100)
                        update_odometry()
                    continue  # 跳出绕墙循环，回到外层 while，继续朝目标直行

                    # 检查是否绕了一整圈又回到同一个 hit point
                if obstacle_result == "hit_point":
                    print("Bug2: returned to the same hit point -> goal is unreachable.")
                    ev3.speaker.beep(frequency=400, duration=600)
                    return

    except Exception as e:
        # Emergency stop
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        ev3.speaker.beep(frequency=400, duration=300)
        wait(200)
        ev3.speaker.beep(frequency=400, duration=300)
        print("Exception in main_bug2:", e)

# ============================ RUN PROGRAM =============================

if __name__ == "__main__":
    # 原来的 Lab3 main() 仍然保留在文件中，但这里我们运行 Bug2 主程序
    main_bug2()
