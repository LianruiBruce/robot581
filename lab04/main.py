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
DRIVE_SPEED = 250             # Motor speed in degrees per second for forward motion
TURN_SPEED = 150               # Motor speed in degrees per second for turning

# Lab 3 Specific Parameters
BACKUP_DISTANCE_MM = 130      # Distance to back away from obstacle (~18 cm)
TARGET_WALL_DISTANCE_MM = 130 # Target distance from wall during following (15 cm)
MAX_WALL_DISTANCE_MM = 180   # Maximum allowed distance (~23 cm)
HIT_POINT_TOLERANCE_MM = 100  # How close to be considered "back at hit point"
OBSTACLE_DETECTION_DISTANCE_MM = 350  # Distance to detect obstacle (35 cm)
CORNER_DISTANCE_TOLERANCE_MM = 80     # 将最后一次正常距离视为拐角的容差
FAKE_WALL_DISTANCE_MM = 330           # 用于拐角绕行的假设墙距（25 cm）
FAKE_WALL_DISTANCE_MAX_MM = 360       # 用于拐角绕行的假设墙距最大值（40 cm）
LEFT_CORNER_GAP      = 400         # mm: d_s must exceed target by this much
LEFT_CORNER_DE_DOT   = 450.0          # mm/s: d_s must be increasing at least this fast
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
GOAL_TOLERANCE_MM = 100.0       # Within 10cm of goal => success
M_LINE_THRESHOLD_MM = 50      # Within 8cm of M-line => considered "on M-line"

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
    global robot_x, robot_y, robot_heading, last_left_angle, last_right_angle, gyro_offset
    # ⭐ 应该包含 gyro_offset
    
    left_angle = left_motor.angle()
    right_angle = right_motor.angle()
    
    left_delta = left_angle - last_left_angle
    right_delta = right_angle - last_right_angle
    
    left_distance = (left_delta / 360.0) * WHEEL_CIRCUMFERENCE_MM
    right_distance = (right_delta / 360.0) * WHEEL_CIRCUMFERENCE_MM
    
    last_left_angle = left_angle
    last_right_angle = right_angle
    
    # 使用轮差计算角度变化
    wheel_angle_change = math.degrees(
        (right_distance - left_distance) / AXLE_TRACK_MM
    )
    
    # 融合轮式里程计和陀螺仪
    gyro_heading = -gyro.angle() + gyro_offset  # ⭐ 这里用到了 gyro_offset
    wheel_based_heading = robot_heading + wheel_angle_change
    
    # 70%陀螺仪 + 30%轮差
    robot_heading = 0.5 * gyro_heading + 0.5 * wheel_based_heading
    
    # 位置更新使用平均航向
    avg_heading = robot_heading - wheel_angle_change / 2
    heading_rad = math.radians(avg_heading)
    
    forward_distance = (left_distance + right_distance) / 2.0
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
        update_odometry()
        wait(20)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    update_odometry()
    
    
def handle_collision_recovery_intelligent():
    """
    改进版智能碰撞恢复：
    1) 停止 → 稳定
    2) 后退固定距离
    3) 向右原地转 70°
    4) 前冲一小段（恢复到可测墙的稳定几何区域）
    """
    # (0) 停下来
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(50)
    update_odometry()

    # ========== (1) 后退 ==========
    BACKUP_MM = 100
    drive_straight_pid(-BACKUP_MM, speed=DRIVE_SPEED * 0.6)
    wait(80)
    update_odometry()

    # ========== (2) 向右转 70° ==========
    turn_in_place_simple(90, speed=TURN_SPEED)
    wait(80)
    update_odometry()

    # ========== (3) 微小前冲 ========== 
    SURGE_MM = 90     # 建议 80–120mm，EV3 最稳区间
    surge_deg = (SURGE_MM / WHEEL_CIRCUMFERENCE_MM) * 360

    reset_and_sync_encoders()

    while True:
        avg_rot = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        if avg_rot >= surge_deg:
            break
        
        # 往前跑一点
        left_motor.run(220)
        right_motor.run(220)
        wait(10)
        update_odometry()

        # 过程中如果再次撞到东西 → 再用小角度右转恢复
        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            wait(80)
            update_odometry()

            # 小角度右转防止再次贴墙
            turn_in_place_simple(30, speed=TURN_SPEED * 0.8)
            wait(80)
            update_odometry()
            break

    # 完成后停止
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    update_odometry()

    print("Collision recovery: back, rotate 70°, surge forward")
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
    改进的M-line跟踪，使用横向偏移修正
    """
    update_odometry()

    dx = goal_x - robot_x
    dy = goal_y - robot_y
    dist_goal = math.sqrt(dx*dx + dy*dy)
    if dist_goal <= GOAL_TOLERANCE_MM:
        return "goal"

    # ⭐ 计算横向偏移（Cross-Track Error）
    cross_track_error = point_line_distance(
        robot_x, robot_y,
        start_point_x, start_point_y,
        goal_x, goal_y
    )
    
    # 判断偏移方向（在M-line左侧还是右侧）
    # 使用叉积判断
    dx_line = goal_x - start_point_x
    dy_line = goal_y - start_point_y
    dx_robot = robot_x - start_point_x
    dy_robot = robot_y - start_point_y
    cross_product = dx_line * dy_robot - dy_line * dx_robot
    
    if cross_product > 0:
        cross_track_error = -cross_track_error  # 在左侧，需要右转

    desired_heading = m_line_heading_deg + math.degrees(
            math.atan2(cross_track_error, 500)  # 500mm是预瞄距离
        )
    heading_error = normalize_angle(desired_heading - robot_heading)
    if abs(heading_error) > 5:
        turn_in_place_simple(-heading_error, speed=TURN_SPEED)
        wait(50)
        update_odometry()

        

    sw = StopWatch()
    sw.reset()
    max_run_time_ms = 180000

    # 横向偏移PID参数
    CROSS_TRACK_KP = 0.3  # 横向偏移比例增益
    HEADING_KP = 2.0      # 航向误差比例增益

    while True:
        update_odometry()

        # 检查是否到达
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist_goal = math.sqrt(dx*dx + dy*dy)
        if dist_goal <= GOAL_TOLERANCE_MM:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            update_odometry()
            return "goal"

        # 检查碰撞
        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            update_odometry()
            ev3.speaker.beep()
            return "obstacle"

        if sw.time() > max_run_time_ms:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            return "timeout"
        
        # ⭐ 重新计算横向偏移
        cross_track_error = point_line_distance(
            robot_x, robot_y,
            start_point_x, start_point_y,
            goal_x, goal_y
        )
        
        # 判断偏移方向
        dx_line = goal_x - start_point_x
        dy_line = goal_y - start_point_y
        dx_robot = robot_x - start_point_x
        dy_robot = robot_y - start_point_y
        cross_product = dx_line * dy_robot - dy_line * dx_robot
        
        if cross_product > 0:
            cross_track_error = -cross_track_error
        
        # ⭐ 同时修正朝向误差和横向偏移
        heading_error = normalize_angle(m_line_heading_deg - robot_heading)
        
        # 组合修正量
        correction = (HEADING_KP * heading_error + 
                     CROSS_TRACK_KP * cross_track_error)
        correction = max(-40, min(40, correction))        
        left_motor.run(speed - correction)
        right_motor.run(speed + correction)
        wait(50)

def follow_wall_until_hit_point(goal_x, goal_y, hit_x, hit_y,
                                target_distance_mm=TARGET_WALL_DISTANCE_MM, speed=DRIVE_SPEED):
    """w9
    左侧贴墙绕障碍物：
      - 贴墙走，保持与墙的距离 ~ target_distance_mm
      - 检查是否到达 Bug2 目标
      - 检查是否回到 M-line 且更接近 goal => 返回 "leave"
      - 检查是否绕一圈回到 hit point => 返回 "hit_point"

    相比旧版：
      - 超声波读数增加滤波与限跳变
      - 拐角检测单独分支（避免误判造成猛撞）
      - PID 反应稍微温和一点，减少过度转向
    """
    global last_min_goal_dist

    # ---- PID 状态 ----
    integral = 0.0
    last_error = 0.0

    # 初始距离用当前测距（如果异常就退回 target）
    raw = ultrasonic.distance()
    if raw is None or raw <= 0:
        last_distance = target_distance_mm
    else:
        last_distance = raw

    ALPHA = 0.4  # 距离 EMA 滤波（比 0.5 稍微平稳一点）

    # 编码器重新对齐
    update_odometry()
    reset_and_sync_encoders()

    step_count = 0
    corner_trigger_count = 0

    print("Enter wall following (Bug2)")

    while True:
        step_count += 1

        # ===================== 1) 超声波读取 + 预处理 =====================
        raw_d = ultrasonic.distance()
        if raw_d is None or raw_d <= 0:
            # 读不到墙，就先用上一次
            raw_d = last_distance

        # 原始斜率，用于拐角检测
        deriv_raw = raw_d - last_distance

        # ---------- 真拐角检测 ----------
        # “距离突然大很多 + 斜率很大” => 说明前面的这段墙结束了，需要左转去找下一面墙
        if raw_d > (target_distance_mm + LEFT_CORNER_GAP) and deriv_raw > LEFT_CORNER_DE_DOT:
            corner_trigger_count += 1
            print("[Corner trigger {} / {}] raw_d={}, deriv={}".format(
                corner_trigger_count, K_CORNER, raw_d, deriv_raw
            ))

            if corner_trigger_count >= K_CORNER:
                print("[REAL LEFT CORNER CONFIRMED] -> turn left & surge")

                # 1) 先停止并重置编码器
                left_motor.stop(Stop.BRAKE)
                right_motor.stop(Stop.BRAKE)
                reset_and_sync_encoders()

                # 2) 左转一个固定角度（负值 = 左转；墙在左边，遇到拐角需要左转过去）
                turn_in_place_simple(-45, speed=TURN_SPEED)   # 60° 左转，视实际情况可调
                wait(30)
                reset_and_sync_encoders()
                update_odometry()

                # 3) 再向前冲一点，让机器人真正靠上新的那面墙
                surge_mm = 200.0
                surge_deg = (surge_mm / WHEEL_CIRCUMFERENCE_MM) * 360.0
                while True:
                    avg_rot = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
                    if avg_rot >= surge_deg:
                        break

                    left_motor.run(240)
                    right_motor.run(240)
                    update_odometry()

                    # 冲的过程中如果又撞到了前墙，交给你的智能碰撞恢复
                    if touch_left.pressed() or touch_right.pressed():
                        left_motor.stop(Stop.BRAKE)
                        right_motor.stop(Stop.BRAKE)
                        update_odometry()
                        wait(100)
                        handle_collision_recovery_intelligent()
                        reset_and_sync_encoders()
                        break

                    wait(10)

                # 拐角处理完：重置 corner 计数，距离回到目标附近
                corner_trigger_count = 0
                last_distance = target_distance_mm
                integral = 0.0
                last_error = 0.0
                wait(40)
                # 不做 PID 一步，直接进入下一轮 while
                continue
        else:
            # 条件不满足，重置 trigger 计数
            corner_trigger_count = 0

        # ---------- 假远距离保护 ----------
        # 有时车身姿态有点斜，瞬间读到一个很远的数，这里只轻微往墙内拉一点
        if raw_d > 350 and abs(deriv_raw) > 220:
            print("[FALSE FAR DISTANCE GUARD] raw_d={}, deriv={}".format(raw_d, deriv_raw))
            current_distance = max(50, last_distance - 8)
        else:
            # ---------- 正常情况：限跳变 + EMA 滤波 ----------
            if abs(raw_d - last_distance) > MAX_D_STEP:
                # 限制每次变化不超过 MAX_D_STEP
                if raw_d > last_distance:
                    raw_d = last_distance + MAX_D_STEP
                else:
                    raw_d = last_distance - MAX_D_STEP

            current_distance = ALPHA * raw_d + (1.0 - ALPHA) * last_distance

        last_distance = current_distance

        # ===================== 2) 碰撞检测（触碰传感器） =====================
        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            wait(80)

            handle_collision_recovery_intelligent()
            # 碰撞恢复之后，把 PID 状态清空
            integral = 0.0
            last_error = 0.0
            last_distance = target_distance_mm
            reset_and_sync_encoders()
            update_odometry()
            continue

        # ===================== 3) 墙距 PID 控制 =====================
        # 这里继续沿用你原本的定义：error = target - current
        error = target_distance_mm - current_distance

        # P
        p = WALL_KP * error

        # I（积分收紧一些）
        integral += error * 0.1
        integral = max(-20, min(20, integral))
        i = WALL_KI * integral

        # D（假设 loop ~0.1s）
        derivative = (error - last_error) / 1.0
        d = WALL_KD * derivative
        last_error = error

        correction = p + i + d
        correction = max(-45, min(45, correction))

        # too close (error>0) => 右转离墙：左轮快右轮慢
        left_speed = speed + correction
        right_speed = speed - correction

        # 限制速度
        min_speed = 60
        max_speed = speed * 1.2
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))

        left_motor.run(left_speed)
        right_motor.run(right_speed)

        # 里程计更新
        update_odometry()

        # ===================== 4) Bug2 退出条件 =====================
        # (1) 到达最终目标
        dxg = goal_x - robot_x
        dyg = goal_y - robot_y
        dist_goal = math.sqrt(dxg * dxg + dyg * dyg)
        if dist_goal <= GOAL_TOLERANCE_MM:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            update_odometry()
            return "goal"

        # (2) 回到 M-line 且比以往更接近目标 => 可以离开障碍物
        leave, last_min_goal_dist = on_m_line_and_closer(
            robot_x, robot_y,
            start_point_x, start_point_y,
            goal_x, goal_y,
            last_min_goal_dist,
            m_line_threshold=M_LINE_THRESHOLD_MM
        )
        if leave:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            update_odometry()
            print("[Bug2] leave obstacle on M-line")
            return "leave"

        # (3) 绕一圈又回到 hit point 附近 => 目标不可达
        dist_back_to_hit = math.sqrt((robot_x - hit_x) ** 2 + (robot_y - hit_y) ** 2)
        if step_count > 50 and dist_back_to_hit < HIT_POINT_TOLERANCE_MM:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            update_odometry()
            print("[Bug2] back to hit point -> unreachable")
            return "hit_point"

        # ===================== 5) 节奏控制 & 调试输出 =====================
        wait(50)  # 每步 ~50ms
        if step_count % 50 == 0:
            print("wall-follow pose: x={:.0f}, y={:.0f}, dist={:.1f}".format(
                robot_x, robot_y, current_distance
            ))



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
    # global goal_x, goal_y

    try:
        # 启动提示 & 等待按键
        ev3.speaker.beep()
        while True:
            if Button.CENTER in ev3.buttons.pressed():
                break
            wait(10)
        ev3.speaker.beep()
        wait(200)

        # ------- Bug2 起点与目标 -------
        start_point_x = 500.0   # 0.5 m
        start_point_y = 0.0     # 0.0 m
        robot_x = start_point_x
        robot_y = start_point_y
        robot_heading = 90

        goal_x = 2567.0         # 2.5 m
        goal_y = 2578.0         # 2.5 m

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
                    # heading_error = normalize_angle(m_line_heading_deg - robot_heading)
                    # if abs(heading_error) > 3:
                    #     turn_in_place_simple(heading_error, speed=TURN_SPEED)
                    #     wait(100)
                    #     update_odometry()
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