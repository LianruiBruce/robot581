#!/usr/bin/env pybricks-micropython
# Team Members: Lianrui Geng && Xinyi Guo
# Lab 03  BOUNDARY TRACING AND RETURN TO START
#
# 实验核心原则：
# 1. 每一步动作都短小且可预测，方便死算（dead reckoning）。
# 2. 每次动作前后都读取传感器并更新姿态，及时纠正误差。
# 3. 决策逻辑保持简洁：感知 → 判断 → 小步调整。

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait, StopWatch

# ---------------------------------------------------------------------------
# 硬件端口与几何参数
# ---------------------------------------------------------------------------
LEFT_MOTOR_PORT = Port.B              # 左驱动电机
RIGHT_MOTOR_PORT = Port.C             # 右驱动电机
TOUCH_LEFT_PORT = Port.S1             # 左触碰传感器（面向墙）
TOUCH_RIGHT_PORT = Port.S3            # 右触碰传感器
GYRO_PORT = Port.S2                   # 陀螺仪
ULTRA_PORT = Port.S4                  # 左侧超声波

WHEEL_DIAMETER_MM = 56.0              # 轮子直径
AXLE_TRACK_MM = 125.0                 # 轮距
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM

# ---------------------------------------------------------------------------
# 运动与控制参数
# ---------------------------------------------------------------------------
DRIVE_SPEED = 180                     # 前进速度（度/秒）
TURN_SPEED = 80                       # 转向速度（度/秒）

STEP_DISTANCE_MM = 50                 # 每步前进距离（减小步长更谨慎）
TARGET_WALL_DISTANCE_MM = 200         # 理想贴墙距离（20cm）
DISTANCE_TOLERANCE_MM = 30            # 距离容差（±30mm）
MAX_DISTANCE_VALID_MM = 2000          # 超声有效上限
HIT_POINT_TOLERANCE_MM = 150          # 判定回到 hit point 的距离容差

# 简化的调整角度
SMALL_TURN_ANGLE = 5                  # 小幅调整角度
MEDIUM_TURN_ANGLE = 10                # 中等调整角度
LARGE_TURN_ANGLE = 15                 # 大幅调整角度

GYRO_KP = 3.0                         # 直行 PID 比例
GYRO_KI = 0.01                        # 直行 PID 积分
GYRO_KD = 1.5                         # 直行 PID 微分

# ---------------------------------------------------------------------------
# 设备与里程计状态
# ---------------------------------------------------------------------------
ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
touch_left = TouchSensor(TOUCH_LEFT_PORT)
touch_right = TouchSensor(TOUCH_RIGHT_PORT)
gyro = GyroSensor(GYRO_PORT)
ultrasonic = UltrasonicSensor(ULTRA_PORT)

gyro.reset_angle(0)
left_motor.reset_angle(0)
right_motor.reset_angle(0)
wait(100)

robot_x = 2000.0
robot_y = 500.0
robot_heading = 0.0
last_left_angle = 0
last_right_angle = 0

# ---------------------------------------------------------------------------
# 基础工具函数
# ---------------------------------------------------------------------------
def normalize_angle(angle_deg):
    """中文：将角度转换到 [-180, 180] 范围，便于比较。"""
    angle_deg = angle_deg % 360
    if angle_deg > 180:
        angle_deg -= 360
    return angle_deg


def update_odometry():
    """中文：根据轮子编码器增量与陀螺仪更新里程计状态。"""
    global robot_x, robot_y, robot_heading, last_left_angle, last_right_angle
    
    current_left = left_motor.angle()
    current_right = right_motor.angle()

    delta_left = current_left - last_left_angle
    delta_right = current_right - last_right_angle

    last_left_angle = current_left
    last_right_angle = current_right

    distance_left = (delta_left / 360.0) * WHEEL_CIRCUMFERENCE_MM
    distance_right = (delta_right / 360.0) * WHEEL_CIRCUMFERENCE_MM
    distance_forward = (distance_left + distance_right) / 2.0

    robot_heading = gyro.angle()
    heading_rad = math.radians(robot_heading)
    
    robot_x += distance_forward * math.cos(heading_rad)
    robot_y += distance_forward * math.sin(heading_rad)

    return robot_x, robot_y, robot_heading


def distance_to_point(x_mm, y_mm):
    """中文：计算当前与目标点的欧氏距离。"""
    dx = robot_x - x_mm
    dy = robot_y - y_mm
    return math.sqrt(dx * dx + dy * dy)

# ---------------------------------------------------------------------------
# 基础运动原语：直行 + 原地转向
# ---------------------------------------------------------------------------
def drive_straight_pid(distance_mm, speed=DRIVE_SPEED, abort_on_touch=False):
    """中文：陀螺仪闭环直行；若启用 abort_on_touch 则触碰即停。"""
    if distance_mm == 0:
        return 'ok'

    target_rotation = (abs(distance_mm) / WHEEL_CIRCUMFERENCE_MM) * 360
    direction = 1 if distance_mm > 0 else -1
    
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_gyro = gyro.angle()
    
    gyro_integral = 0
    gyro_last_error = 0
    stopwatch = StopWatch()
    last_time = 0
    
    while True:
        if abort_on_touch and (touch_left.pressed() or touch_right.pressed()):
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            wait(60)
            update_odometry()
            return 'collision'

        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt <= 0:
            dt = 0.05
        last_time = current_time
        
        avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        if avg_rotation >= target_rotation:
            break
        
        gyro_error = gyro.angle() - initial_gyro
        gyro_integral += gyro_error * dt
        gyro_integral = max(-30, min(30, gyro_integral))
        gyro_derivative = (gyro_error - gyro_last_error) / dt
        gyro_last_error = gyro_error
        
        correction = (GYRO_KP * gyro_error +
                      GYRO_KI * gyro_integral +
                      GYRO_KD * gyro_derivative)
        correction = max(-45, min(45, correction))

        if direction > 0:
            left_speed = speed - correction
            right_speed = speed + correction
        else:
            left_speed = speed + correction
            right_speed = speed - correction
        
        left_speed *= direction
        right_speed *= direction

        max_speed = speed * 1.2
        left_speed = max(-max_speed, min(max_speed, left_speed))
        right_speed = max(-max_speed, min(max_speed, right_speed))
        
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        wait(20)
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(60)
    update_odometry()
    return 'ok'


def turn_in_place_simple(angle_degrees, speed=TURN_SPEED):
    """中文：原地转向指定角度，使用简单 PID 抑制振荡。"""
    initial_gyro = gyro.angle()
    target_gyro = initial_gyro + angle_degrees

    def normalize_angle_simple(deg):
        while deg > 180:
            deg -= 360
        while deg < -180:
            deg += 360
        return deg

    kp = 2.5
    ki = 0.02
    kd = 0.5
    integral = 0
    last_error = 0

    while True:
        current_gyro = gyro.angle()
        error = normalize_angle_simple(target_gyro - current_gyro)
        if abs(error) < 2:
            break

        integral += error * 0.02
        integral = max(-8, min(8, integral))
        derivative = (error - last_error) / 0.02
        last_error = error

        turn = kp * error + ki * integral + kd * derivative
        turn = max(-speed, min(speed, turn))

        left_motor.run(turn)
        right_motor.run(-turn)
        wait(20)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(80)
    update_odometry()

# ---------------------------------------------------------------------------
# 传感器读取
# ---------------------------------------------------------------------------
def measure_wall_distance():
    """中文：谨慎地测量超声波距离，多次采样取平均值确保准确性。"""
    readings = []
    for _ in range(5):  # 增加采样次数
        try:
            reading = ultrasonic.distance()
            if 0 < reading <= MAX_DISTANCE_VALID_MM:
                readings.append(reading)
        except:
            pass
        wait(15)  # 增加等待时间确保传感器稳定
    
    if len(readings) >= 3:
        # 如果有足够的读数，去掉最大和最小值取平均
        readings.sort()
        trimmed = readings[1:-1]
        return sum(trimmed) / len(trimmed)
    elif readings:
        # 读数较少时，直接取平均
        return sum(readings) / len(readings)
    return None

# ---------------------------------------------------------------------------
# 姿态修正动作（简化版）
# ---------------------------------------------------------------------------
def handle_collision():
    """中文：碰撞后的简单处理：停止 → 后退 → 右转90度。"""
    # 立即停止
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(200)
    
    # 后退150mm
    drive_straight_pid(-150, speed=DRIVE_SPEED * 0.7)
    wait(100)
    
    # 右转90度
    turn_in_place_simple(90, speed=TURN_SPEED)
    wait(100)
    
    update_odometry()


def adjust_heading_after_step(distance_mm):
    """中文：谨慎地根据超声测距调整姿态，确保距离始终在正确范围内。
    
    目标：保持机器人与墙面的距离在 170-230mm 之间（目标200mm ±30mm）
    策略：距离太近→右转远离，距离太远→左转贴近，距离正常→不调整
    """
    
    if distance_mm is None:
        return  # 无有效读数，跳过调整
    
    deviation = distance_mm - TARGET_WALL_DISTANCE_MM
    
    # === 情况1：距离在理想范围内（170-230mm），保持当前姿态 ===
    if abs(deviation) <= DISTANCE_TOLERANCE_MM:
        # 距离正确，继续保持
        return
    
    # === 情况2：距离太近（< 170mm），右转远离墙面 ===
    if deviation < 0:
        if abs(deviation) > 60:
            # 非常近（< 140mm），需要大角度右转（15度）
            turn_angle = LARGE_TURN_ANGLE
        elif abs(deviation) > 30:
            # 较近（140-170mm），中等角度右转（10度）
            turn_angle = MEDIUM_TURN_ANGLE
        else:
            # 稍近（170-200mm），小角度右转（5度）
            turn_angle = SMALL_TURN_ANGLE
        
        turn_in_place_simple(turn_angle, speed=TURN_SPEED * 0.8)
        wait(50)  # 等待调整稳定
    
    # === 情况3：距离太远（> 230mm），左转贴近墙面 ===
    else:
        if deviation > 100:
            # 非常远（> 300mm），需要大角度左转（-15度）
            turn_angle = -LARGE_TURN_ANGLE
        elif deviation > 50:
            # 较远（250-300mm），中等角度左转（-10度）
            turn_angle = -MEDIUM_TURN_ANGLE
        else:
            # 稍远（230-250mm），小角度左转（-5度）
            turn_angle = -SMALL_TURN_ANGLE
        
        turn_in_place_simple(turn_angle, speed=TURN_SPEED * 0.8)
        wait(50)  # 等待调整稳定

# ---------------------------------------------------------------------------
# 沿墙循环（简化版）
# ---------------------------------------------------------------------------
def follow_wall_until_hit_point(hit_point_x, hit_point_y):
    """中文：谨慎地沿墙行走直到回到 hit point。
    每步流程：测距(前) → 调整姿态 → 前进小步 → 测距(后) → 再次调整。"""
    
    step_count = 0
    max_distance_from_hit = 0
    initial_distance_to_hit = distance_to_point(hit_point_x, hit_point_y)
    
    while True:
        step_count += 1
        
        # === 1. 检查碰撞（最优先） ===
        if touch_left.pressed() or touch_right.pressed():
            handle_collision()
            wait(100)  # 等待稳定
            continue
        
        # === 2. 前进前先测距检查 ===
        wall_distance_before = measure_wall_distance()
        wait(30)  # 等待传感器稳定
        
        # === 3. 如果距离偏离太多，先调整姿态再前进 ===
        if wall_distance_before is not None:
            deviation_before = abs(wall_distance_before - TARGET_WALL_DISTANCE_MM)
            if deviation_before > DISTANCE_TOLERANCE_MM:
                # 距离不理想，先调整姿态
                adjust_heading_after_step(wall_distance_before)
                wait(50)
        
        # === 4. 谨慎前进小步 ===
        result = drive_straight_pid(STEP_DISTANCE_MM, speed=DRIVE_SPEED * 0.9, abort_on_touch=True)
        if result == 'collision':
            handle_collision()
            wait(100)
            continue
        
        wait(50)  # 前进后等待稳定
        
        # === 5. 前进后立即测距 ===
        wall_distance_after = measure_wall_distance()
        wait(30)
        
        # === 6. 根据前进后的距离进行姿态调整 ===
        if wall_distance_after is not None:
            adjust_heading_after_step(wall_distance_after)
            wait(50)
        
        # === 7. 检查是否回到 hit point ===
        current_dist_to_hit = distance_to_point(hit_point_x, hit_point_y)
        max_distance_from_hit = max(max_distance_from_hit, current_dist_to_hit)
        
        # 回到hit point的条件：距离<150mm 且 已经走了一圈（离开过hit point）
        if current_dist_to_hit < HIT_POINT_TOLERANCE_MM:
            if step_count > 20 and max_distance_from_hit > initial_distance_to_hit + 300:
                break
        
        wait(30)  # 每步之间的短暂等待
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    update_odometry()

# ---------------------------------------------------------------------------
# 主流程：直行至碰撞 → 建立 hit point → 沿墙一圈
# ---------------------------------------------------------------------------
def drive_forward_until_obstacle(speed=DRIVE_SPEED, max_time_ms=30000):
    """中文：向前推进直到检测到障碍物（距离<30cm）或发生碰撞。
    符合实验要求：在距离<30cm时蜂鸣。"""
    step_mm = 50
    stopwatch = StopWatch()
    
    while stopwatch.time() < max_time_ms:
        # 测量距离
        distance = measure_wall_distance()
        
        # 如果检测到障碍物在30cm内，蜂鸣并停止
        if distance is not None and distance < 300:
            ev3.speaker.beep(frequency=1000, duration=200)
            wait(250)
            # 调整到理想距离（20cm）
            if distance > TARGET_WALL_DISTANCE_MM + 30:
                advance_mm = distance - TARGET_WALL_DISTANCE_MM
                advance_mm = min(advance_mm, 80)
                drive_straight_pid(advance_mm, speed=speed * 0.6, abort_on_touch=True)
            elif distance < TARGET_WALL_DISTANCE_MM - 30:
                backup_mm = TARGET_WALL_DISTANCE_MM - distance
                backup_mm = min(backup_mm, 80)
                drive_straight_pid(-backup_mm, speed=speed * 0.6)
            return True
        
        # 小步前进并检查碰撞
        status = drive_straight_pid(step_mm, speed=speed, abort_on_touch=True)
        if status == 'collision':
            ev3.speaker.beep(frequency=1000, duration=200)
            wait(250)
            return True
    
    return False


def main():
    """中文：谨慎的主程序流程，每个阶段都有充分的检查和等待。"""
    try:
        ev3.speaker.beep()

        # 等待中心按钮按下
        while Button.CENTER not in ev3.buttons.pressed():
            wait(20)
        wait(400)

        # 重置陀螺仪并等待稳定
        gyro.reset_angle(0)
        wait(150)

        # === 阶段1：谨慎地直行找到障碍物（hit point）===
        if not drive_forward_until_obstacle():
            ev3.speaker.beep(frequency=500, duration=300)
            return
        
        wait(100)  # 到达hit point后等待稳定
        update_odometry()
        
        # 记录hit point位置（距离障碍物约20cm）
        hit_point_x = robot_x
        hit_point_y = robot_y

        # === 阶段2：后退并右转90度准备沿墙 ===
        drive_straight_pid(-200, speed=DRIVE_SPEED * 0.8)
        wait(150)  # 等待后退完成并稳定
        
        update_odometry()
        # 更新hit point为后退后的位置
        hit_point_x = robot_x
        hit_point_y = robot_y

        turn_in_place_simple(90, speed=TURN_SPEED)
        wait(150)  # 等待转向完成并稳定

        # === 阶段3：谨慎地沿墙走一圈 ===
        # 在这个阶段，每一步都会：
        # 1. 前进前测距并调整
        # 2. 小步前进（50mm）
        # 3. 前进后测距并再次调整
        # 确保始终保持与墙面的正确距离（170-230mm）
        follow_wall_until_hit_point(hit_point_x, hit_point_y)

        # === 完成任务 ===
        ev3.speaker.beep(frequency=800, duration=200)
        wait(150)
        ev3.speaker.beep(frequency=1000, duration=200)

    except Exception as e:
        # 异常处理：立即停止并发出警告音
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        ev3.speaker.beep(frequency=350, duration=300)
        wait(180)
        ev3.speaker.beep(frequency=350, duration=300)


if __name__ == "__main__":
    main()