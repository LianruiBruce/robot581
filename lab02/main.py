#!/usr/bin/env pybricks-micropython
# Team Members: [Your Names Here]
# PIDs: [Your PIDs Here]
# Team Number: [Your Team Number Here]

import math
from collections import deque

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait, StopWatch

# ============================ CONFIGURATION =============================

# Hardware Ports
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_LEFT_PORT = Port.S1
TOUCH_RIGHT_PORT = Port.S3
GYRO_PORT = Port.S2
ULTRA_PORT = Port.S4  # Ultrasonic sensor facing LEFT side

# Robot Geometry
WHEEL_DIAMETER_MM = 56.0
AXLE_TRACK_MM = 125.0

# Movement Parameters
DRIVE_SPEED = 180  # deg/s
TURN_SPEED = 80    # deg/s

# ============================ INITIALIZATION =============================

# Initialize EV3 Brick
ev3 = EV3Brick()

# Initialize motors
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

# Initialize sensors
touch_left = TouchSensor(TOUCH_LEFT_PORT)
touch_right = TouchSensor(TOUCH_RIGHT_PORT)
gyro = GyroSensor(GYRO_PORT)
ultrasonic = UltrasonicSensor(ULTRA_PORT)

# Reset gyro sensor
gyro.reset_angle(0)
wait(10)

# ============================ HELPER FUNCTIONS =============================

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def get_distance_mm(last_distance, retries=3, alpha=0.5, min_mm=40, max_mm=600):
    """
    稳健超声读数：重试 -> 中值 -> EMA 平滑
    alpha 越大越灵敏；已做异常值筛除
    """
    vals = []
    for _ in range(retries):
        try:
            v = ultrasonic.distance()
            if min_mm <= v <= max_mm:
                vals.append(v)
        except:
            pass
        wait(5)  # 给 I2C/回波一点时间
    if not vals:
        raw = last_distance
    else:
        vals.sort()
        raw = vals[len(vals)//2]  # 中值抗尖峰
    # EMA 平滑
    return alpha * raw + (1 - alpha) * last_distance

def slew_toward(prev, target, max_step):
    """纠偏爬升限速：每周期最多变化 max_step，避免猛甩。"""
    delta = target - prev
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return prev + delta

def drive_straight_pid(distance_mm, speed=DRIVE_SPEED):
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

    print("Driving straight: " + str(distance_mm) + " mm")

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
        gyro_integral = clamp(gyro_integral, -30, 30)
        gyro_i = GYRO_KI * gyro_integral
        gyro_derivative = (gyro_error - gyro_last_error) / dt
        gyro_d = GYRO_KD * gyro_derivative
        gyro_last_error = gyro_error

        correction = clamp(gyro_p + gyro_i + gyro_d, -50, 50)

        if direction > 0:
            left_speed = speed - correction
            right_speed = speed + correction
        else:
            left_speed = speed + correction
            right_speed = speed - correction

        left_speed = left_speed * direction
        right_speed = right_speed * direction

        max_abs_speed = speed * 1.2
        left_speed = clamp(left_speed, -max_abs_speed, max_abs_speed)
        right_speed = clamp(right_speed, -max_abs_speed, max_abs_speed)

        left_motor.run(left_speed)
        right_motor.run(right_speed)

        wait(20)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    print("Drive complete.")

def turn_in_place_pid(angle_degrees, speed=TURN_SPEED):
    """Turn in place by specified angle using gyro feedback."""
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

    # Coarse turn
    while True:
        current_gyro = gyro.angle()
        error = target_gyro - current_gyro

        if abs(error) < 5:
            break

        if stopwatch.time() > 5000:
            print("Coarse turn timeout!")
            break

        turn_speed = clamp(COARSE_KP * error, -speed, speed)

        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)

        wait(10)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)

    stopwatch.reset()

    # Fine turn
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
        integral = clamp(integral, -10, 10)
        i = FINE_KI * integral
        derivative = (error - last_error) / dt
        d = FINE_KD * derivative
        last_error = error

        turn_speed = clamp(p + i + d, -speed * 0.6, speed * 0.6)

        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)

        wait(20)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(200)
    print("Turn complete!")

def drive_until_collision_controlled(speed=DRIVE_SPEED):
    """Drive forward until collision."""
    print("Driving forward until collision...")

    wait(10)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_gyro = gyro.angle()
    wait(10)

    GYRO_KP = 2.5

    while True:
        left_pressed = touch_left.pressed()
        right_pressed = touch_right.pressed()

        if left_pressed or right_pressed:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("Collision detected!")
            return

        gyro_error = gyro.angle() - initial_gyro
        correction = clamp(GYRO_KP * gyro_error, -30, 30)

        left_speed = speed - correction
        right_speed = speed + correction

        left_motor.run(left_speed)
        right_motor.run(right_speed)

        wait(10)

def follow_wall_diagnostic(target_distance_mm=300, wall_length_mm=2400, speed=DRIVE_SPEED):
    """
    改进的墙壁跟随算法：
      - 修复了外折墙壁时不能左转的问题
      - 移除了阻止"距离过远"纠正的逻辑
      - 改进了真/假变远的判别
    """
    print("="*50)
    print("DIAGNOSTIC WALL FOLLOWING (FIXED)")
    print("="*50)
    print("Target: " + str(target_distance_mm) + "mm")
    print("Length: " + str(wall_length_mm) + "mm")

    # ========== Key Parameters ==========
    TARGET_DISTANCE = target_distance_mm

    # 基础比例增益
    CORRECTION_GAIN = 1.4
    MAX_CORRECTION = 100

    # Gyro assist（保持关闭）
    GYRO_ASSIST = 0.0

    # 纠偏爬升限速（抑制猛甩）
    MAX_DELTA_CORR = 20

    # 读取与平滑
    ALPHA = 0.5
    READ_RETRIES = 3

    # 真/假变远判别
    POS_STEP = 15        # 单周期"变远"阈值(mm)
    K_PERSIST = 3        # 连续变远次数阈值
    THETA_SMALL = 5      # 陀螺显著转角阈值(°)

    # 外拐角/探空（只用于检测开放空间，不是外折墙壁）
    MAX_WALL_MM = 600
    EDGE_RISE_MM = 50    # 提高阈值，只在真正探空时触发
    EDGE_TICKS_HOLD = 10
    EDGE_BIAS = 60      # 负=右转

    # 内拐角（真变近）
    CORNER_DROP_MM = -20
    CORNER_NEAR_MM = max(200, TARGET_DISTANCE - 80)
    CORNER_TICKS = 10
    CORNER_BIAS = -60    # 正=左转

    # 方向反转（保留开关）
    REVERSE_CORRECTION = False

    print("CORRECTION_GAIN:", CORRECTION_GAIN)
    print("MAX_CORRECTION:", MAX_CORRECTION)
    print("GYRO_ASSIST:", GYRO_ASSIST)
    print("REVERSE_CORRECTION:", REVERSE_CORRECTION)
    print("="*50)

    # Initialization
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    parallel_gyro_reference = gyro.angle()
    prev_gyro = parallel_gyro_reference

    last_distance = TARGET_DISTANCE
    last_correction = 0.0

    iteration = 0

    # 状态
    edge_ticks = 0
    corner_ticks = 0
    persist_pos = 0

    while True:
        iteration += 1

        # ---------- Robust distance read ----------
        current_distance = get_distance_mm(
            last_distance,
            retries=READ_RETRIES,
            alpha=ALPHA,
            max_mm=MAX_WALL_MM
        )

        # ---------- Deltas & classification ----------
        current_gyro = gyro.angle()
        gyro_deviation = current_gyro - parallel_gyro_reference
        delta_theta_step = current_gyro - prev_gyro
        delta_d = current_distance - last_distance

        # 持续性统计：连续"变远"
        if delta_d > POS_STEP:
            persist_pos += 1
        else:
            persist_pos = max(0, persist_pos - 1)

        # 判断是否是"真正的墙壁变远"（外折）vs "探空"（开放空间）
        # 外折：距离缓慢增加，没有急剧变远
        # 探空：距离急剧增加到很远
        is_true_far = (
            persist_pos >= K_PERSIST or
            (delta_d > POS_STEP and abs(delta_theta_step) >= THETA_SMALL)
        )
        
        # 探空检测：只有在距离变得非常远时才判断为探空
        is_open_space = current_distance >= MAX_WALL_MM * 0.8  # 480mm以上

        # 内拐角：快速变近且已较近 → 左转增强
        if corner_ticks == 0:
            if delta_d <= CORNER_DROP_MM and current_distance <= CORNER_NEAR_MM:
                corner_ticks = CORNER_TICKS

        # 探空检测：只在真正的开放空间时右转寻墙
        if edge_ticks == 0 and is_open_space:
            edge_ticks = EDGE_TICKS_HOLD

        # ---------- Compute distance error ----------
        distance_error = current_distance - TARGET_DISTANCE

        # ---------- Compute correction ----------
        distance_correction = distance_error * CORRECTION_GAIN

        # ✅ 修复：移除了阻止"距离过远"纠正的逻辑
        # 原来的代码在这里会清零distance_correction，导致外折墙壁时不能左转
        # if distance_correction > 15 and continue_far < 3:
        #     continue_far += 1
        #     distance_correction = 0  # ❌ 这是问题所在！
        # else:
        #     continue_far = 0

        distance_correction = clamp(distance_correction, -MAX_CORRECTION, MAX_CORRECTION)

        # Gyro assist（保持关闭）
        gyro_correction = gyro_deviation * GYRO_ASSIST

        total_correction = distance_correction + gyro_correction

        # 陀螺大偏差保护
        if abs(gyro_deviation) > 60:
            total_correction = 0

        # 角点/边缘微策略
        if corner_ticks > 0:
            total_correction += CORNER_BIAS   # 左转
            corner_ticks -= 1
        elif edge_ticks > 0:
            # 只在探空时右转，不在外折墙壁时右转
            total_correction += EDGE_BIAS     # 右转
            edge_ticks -= 1

        # 纠偏爬升限速
        total_correction = slew_toward(last_correction, total_correction, MAX_DELTA_CORR)
        last_correction = total_correction

        # 反向选项
        if REVERSE_CORRECTION:
            total_correction = -total_correction

        # ---------- Apply to motors ----------
        left_speed = speed - total_correction
        right_speed = speed + total_correction

        # Limit speed
        min_speed = 40
        max_speed = speed * 1.6
        left_speed = clamp(left_speed, min_speed, max_speed)
        right_speed = clamp(right_speed, min_speed, max_speed)

        # Run motors
        left_motor.run(left_speed)
        right_motor.run(right_speed)

        # ========== Detailed output ==========
        print("="*50)
        print("Iter:", iteration)
        print("Distance:", int(current_distance), "mm",
              "| Raw:", ultrasonic.distance(), "mm")
        print("Error:", int(distance_error), "mm")
        print("Δd:", int(delta_d), "mm",
              "Δθ(step):", int(delta_theta_step), "deg",
              "gyro_dev:", int(gyro_deviation), "deg")
        print("PersistFar:", persist_pos, "TrueFar:", is_true_far, "OpenSpace:", is_open_space)
        print("Corner:", corner_ticks, "Edge:", edge_ticks)
        print("Correction:", int(total_correction))
        print("Left Speed:", int(left_speed), "Right Speed:", int(right_speed))

        # Expected turning direction
        if distance_error < -20:
            print(">>> TOO CLOSE - Should turn RIGHT (away)")
            print(">>> Expected: Left FASTER, Right SLOWER")
        elif distance_error > 20:
            print(">>> TOO FAR - Should turn LEFT (toward)")
            print(">>> Expected: Left SLOWER, Right FASTER")
        else:
            print(">>> GOOD DISTANCE")

        # Actual turning direction
        if left_speed > right_speed + 20:
            print(">>> ACTUAL: Turning RIGHT")
        elif right_speed > left_speed + 20:
            print(">>> ACTUAL: Turning LEFT")
        else:
            print(">>> ACTUAL: Going STRAIGHT")

        print("="*50)

        # Display on screen
        ev3.screen.clear()
        ev3.screen.draw_text(5, 5,  "D:" + str(int(current_distance)))
        ev3.screen.draw_text(5, 20, "Raw:" + str(ultrasonic.distance()))
        ev3.screen.draw_text(5, 35, "Err:" + str(int(distance_error)))
        ev3.screen.draw_text(5, 50, "Corr:" + str(int(total_correction)))
        ev3.screen.draw_text(5, 65, "L/R:" + str(int(left_speed)) + "/" + str(int(right_speed)))
        if corner_ticks > 0:
            ev3.screen.draw_text(5, 80, "CORNER<<")
        elif edge_ticks > 0:
            ev3.screen.draw_text(5, 80, "OPEN SPACE>>")
        else:
            if current_distance < TARGET_DISTANCE - 20:
                ev3.screen.draw_text(5, 80, "TOO CLOSE >>")
            elif current_distance > TARGET_DISTANCE + 20:
                ev3.screen.draw_text(5, 80, "<< TOO FAR")
            else:
                ev3.screen.draw_text(5, 80, "GOOD")

        # Compute traveled distance
        avg_motor_angle = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        wheel_circumference = math.pi * WHEEL_DIAMETER_MM
        distance_traveled = (avg_motor_angle / 360) * wheel_circumference

        # Update histories
        prev_gyro = current_gyro
        last_distance = current_distance

        # Check completion
        if distance_traveled >= wall_length_mm:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("COMPLETE!")
            break

        wait(10)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)

# ============================ MAIN PROGRAM =============================

def main():
    """Main program"""
    try:
        ev3.speaker.beep()
        print("="*50)
        print("FIXED VERSION - 外折墙壁问题已修复")
        print("="*50)
        print("")
        print("Press CENTER to start...")

        while True:
            if Button.CENTER in ev3.buttons.pressed():
                break
            wait(10)

        ev3.speaker.beep()
        wait(3000)

        # ============== Objective 1: Detect Wall ==============
        print("")
        print("="*50)
        print("OBJECTIVE 1: Detect Wall")
        print("="*50)

        drive_until_collision_controlled(speed=DRIVE_SPEED)

        ev3.speaker.beep()
        wait(500)

        print("Backing up 30cm...")
        drive_straight_pid(-300, speed=DRIVE_SPEED)
        wait(500)

        # ============== Objective 2: Turn Right 90° ==============
        print("")
        print("="*50)
        print("OBJECTIVE 2: Turn Right 90°")
        print("="*50)

        print("Turning...")
        turn_in_place_pid(90, speed=TURN_SPEED)
        wait(500)

        print("Resetting gyro...")
        gyro.reset_angle(0)
        wait(300)

        ev3.speaker.beep()
        print("Turn complete!")
        wait(500)

        # ============== Objective 3: Fixed Wall Following ==============
        print("")
        print("="*50)
        print("OBJECTIVE 3: Fixed Wall Following")
        print("="*50)
        print("修复：")
        print("1. 移除了阻止'距离过远'纠正的逻辑")
        print("2. 改进了外折墙壁 vs 探空的判别")
        print("3. 现在能正确左转靠近外折墙壁")
        print("="*50)

        wait(10)
        follow_wall_diagnostic(
            target_distance_mm=300,
            wall_length_mm=2400,
            speed=DRIVE_SPEED
        )
        wait(10)

        # ============== SUCCESS! ==============
        print("")
        print("="*50)
        print("SUCCESS!")
        print("="*50)

        for i in range(4):
            ev3.speaker.beep(frequency=800 + i*200, duration=100)
            wait(150)

    except Exception as e:
        print("")
        print("="*50)
        print("ERROR!")
        print("="*50)
        print("Error:", str(e))

        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)

        ev3.speaker.beep(frequency=400, duration=300)
        wait(200)
        ev3.speaker.beep(frequency=400, duration=300)

# ============================ RUN PROGRAM =============================

if __name__ == "__main__":
    main()