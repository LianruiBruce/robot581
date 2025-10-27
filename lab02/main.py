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

# Movement Parameters (conservative)
DRIVE_SPEED = 180  # deg/s
TURN_SPEED  = 80   # deg/s

# ============================ INITIALIZATION =============================

ev3 = EV3Brick()
left_motor  = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

touch_left  = TouchSensor(TOUCH_LEFT_PORT)
touch_right = TouchSensor(TOUCH_RIGHT_PORT)
gyro        = GyroSensor(GYRO_PORT)
ultrasonic  = UltrasonicSensor(ULTRA_PORT)

gyro.reset_angle(0)
wait(10)

# ============================ HELPER FUNCTIONS =============================

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def get_distance_mm(last_distance, retries=3, alpha=0.5, min_mm=40, max_mm=600):
    """Robust ultrasonic: retries -> median -> EMA."""
    vals = []
    for _ in range(retries):
        try:
            v = ultrasonic.distance()
            if min_mm <= v <= max_mm:
                vals.append(v)
        except:
            pass
        wait(5)
    if not vals:
        raw = last_distance
    else:
        vals.sort()
        raw = vals[len(vals)//2]
    return alpha * raw + (1 - alpha) * last_distance

def slew_toward(prev, target, max_step):
    """Slew rate limit for correction."""
    delta = target - prev
    if delta >  max_step: delta =  max_step
    if delta < -max_step: delta = -max_step
    return prev + delta

def drive_straight_pid(distance_mm, speed=DRIVE_SPEED):
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

    print("Driving straight:", distance_mm, "mm")

    while True:
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0
        if dt <= 0:
            dt = 0.05
        last_time = current_time

        avg_rotation = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        if avg_rotation >= target_rotation:
            break

        gyro_error = gyro.angle() - initial_gyro
        p = GYRO_KP * gyro_error
        gyro_integral += gyro_error * dt
        gyro_integral = clamp(gyro_integral, -30, 30)
        i = GYRO_KI * gyro_integral
        d = GYRO_KD * (gyro_error - gyro_last_error) / dt
        gyro_last_error = gyro_error

        correction = clamp(p + i + d, -50, 50)

        if direction > 0:
            left_speed  = speed - correction
            right_speed = speed + correction
        else:
            left_speed  = speed + correction
            right_speed = speed - correction

        left_speed  *= direction
        right_speed *= direction

        max_abs_speed = speed * 1.2
        left_speed  = clamp(left_speed,  -max_abs_speed, max_abs_speed)
        right_speed = clamp(right_speed, -max_abs_speed, max_abs_speed)

        left_motor.run(left_speed)
        right_motor.run(right_speed)
        wait(20)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)
    print("Drive complete.")

def turn_in_place_pid(angle_degrees, speed=TURN_SPEED):
    COARSE_KP = 2.5
    FINE_KP   = 5.0
    FINE_KI   = 0.08
    FINE_KD   = 3.0

    initial_gyro = gyro.angle()
    target_gyro  = initial_gyro + angle_degrees

    integral   = 0
    last_error = 0

    sw = StopWatch()
    last_time = 0
    stable_count = 0

    print("Turning from", initial_gyro, "to", target_gyro, "deg")

    # Coarse
    while True:
        current_gyro = gyro.angle()
        error = target_gyro - current_gyro
        if abs(error) < 5:
            break
        if sw.time() > 5000:
            print("Coarse turn timeout!")
            break
        turn_speed = clamp(COARSE_KP * error, -speed, speed)
        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)
        wait(10)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(100)

    sw.reset()

    # Fine
    while True:
        current_time = sw.time()
        dt = (current_time - last_time) / 1000.0
        if dt < 0.01:
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

        if sw.time() > 3000:
            print("Fine turn timeout")
            break

        p = FINE_KP * error
        integral += error * dt
        integral = clamp(integral, -10, 10)
        i = FINE_KI * integral
        d = FINE_KD * (error - last_error) / dt
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
    print("Driving forward until collision...")

    wait(10)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    initial_gyro = gyro.angle()
    wait(10)

    GYRO_KP = 2.5

    while True:
        if touch_left.pressed() or touch_right.pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("Collision detected!")
            return

        gyro_error = gyro.angle() - initial_gyro
        correction = clamp(GYRO_KP * gyro_error, -30, 30)

        left_speed  = speed - correction
        right_speed = speed + correction

        left_motor.run(left_speed)
        right_motor.run(right_speed)
        wait(10)

def follow_wall_diagnostic(target_distance_mm=300, wall_length_mm=2400, speed=DRIVE_SPEED):
    """
    Diagnostic wall following (conservative):
      - robust ultrasonic (median + EMA)
      - classify true/false 'far'
      - gentle corner/edge biases
      - slew-limited correction
    """
    print("="*50)
    print("DIAGNOSTIC WALL FOLLOWING")
    print("="*50)
    print("Target:", target_distance_mm, "mm")
    print("Length:", wall_length_mm, "mm")

    TARGET_DISTANCE = target_distance_mm

    # 控制参数（保守）
    CORRECTION_GAIN = 1.2
    MAX_CORRECTION  = 90
    GYRO_ASSIST     = 0.0
    REVERSE_CORRECTION = False

    # 读数与滤波
    ALPHA        = 0.5
    READ_RETRIES = 3

    # “变远”判别（与循环间隔相关，当前 wait(10)）
    POS_STEP     = 15
    K_PERSIST    = 3
    THETA_SMALL  = 5

    # 外折/探空（假变远时的温和左靠）
    MAX_WALL_MM     = 600
    EDGE_RISE_MM    = 35
    EDGE_TICKS_HOLD = 8
    EDGE_BIAS       = +50   # 温和左转

    # 内折（快速变近时的温和左靠）
    CORNER_DROP_MM  = -20
    CORNER_NEAR_MM  = max(200, TARGET_DISTANCE - 80)
    CORNER_TICKS    = 8
    CORNER_BIAS     = +50   # 温和左转

    # 爬升限速
    MAX_DELTA_CORR  = 15

    print("CORRECTION_GAIN:", CORRECTION_GAIN)
    print("MAX_CORRECTION:", MAX_CORRECTION)
    print("GYRO_ASSIST:", GYRO_ASSIST)
    print("REVERSE_CORRECTION:", REVERSE_CORRECTION)
    print("="*50)

    # 初始化
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    parallel_gyro_reference = gyro.angle()
    prev_gyro = parallel_gyro_reference

    last_distance   = TARGET_DISTANCE
    last_correction = 0.0
    iteration       = 0
    continue_far    = 0

    edge_ticks   = 0
    corner_ticks = 0
    persist_pos  = 0

    while True:
        iteration += 1

        # --- 读数 ---
        current_distance = get_distance_mm(
            last_distance, retries=READ_RETRIES, alpha=ALPHA, max_mm=MAX_WALL_MM
        )

        # --- 差分 & 分类 ---
        current_gyro = gyro.angle()
        gyro_deviation   = current_gyro - parallel_gyro_reference
        delta_theta_step = current_gyro - prev_gyro
        delta_d          = current_distance - last_distance

        if delta_d > POS_STEP:
            persist_pos += 1
        else:
            persist_pos = max(0, persist_pos - 1)

        is_true_far = (
            persist_pos >= K_PERSIST or
            (delta_d > POS_STEP and abs(delta_theta_step) >= THETA_SMALL)
        )

        # 内折触发（真变近且已较近）
        if corner_ticks == 0:
            if delta_d <= CORNER_DROP_MM and current_distance <= CORNER_NEAR_MM:
                corner_ticks = CORNER_TICKS

        # 外折/探空触发（不像真变远）
        if edge_ticks == 0 and not is_true_far:
            if delta_d >= EDGE_RISE_MM or current_distance >= MAX_WALL_MM:
                edge_ticks = EDGE_TICKS_HOLD

        # --- 误差 & 纠偏 ---
        distance_error = current_distance - TARGET_DISTANCE
        distance_correction = clamp(distance_error * CORRECTION_GAIN, -MAX_CORRECTION, MAX_CORRECTION)

        # 你原本的“连续远→清零一次纠偏”逻辑，保守保留
        if distance_correction > 15 and continue_far < 3:
            continue_far += 1
            distance_correction = 0
        else:
            continue_far = 0

        gyro_correction = gyro_deviation * GYRO_ASSIST
        total_correction = distance_correction + gyro_correction

        # 陀螺大偏差保护
        if abs(gyro_deviation) > 60:
            total_correction = 0

        # 温和偏置（内折与外折/探空，都向左靠）
        if corner_ticks > 0:
            total_correction += CORNER_BIAS
            corner_ticks -= 1
        elif edge_ticks > 0 and not is_true_far:
            total_correction += EDGE_BIAS
            edge_ticks -= 1

        # 爬升限速
        total_correction = slew_toward(last_correction, total_correction, MAX_DELTA_CORR)
        last_correction  = total_correction

        if REVERSE_CORRECTION:
            total_correction = -total_correction

        # --- 电机 ---
        left_speed  = speed - total_correction
        right_speed = speed + total_correction

        min_speed = 40
        max_speed = speed * 1.6
        left_speed  = clamp(left_speed,  min_speed, max_speed)
        right_speed = clamp(right_speed, min_speed, max_speed)

        left_motor.run(left_speed)
        right_motor.run(right_speed)

        # --- 输出 ---
        print("="*50)
        print("Iter:", iteration)
        try:
            raw_ultra = ultrasonic.distance()
        except:
            raw_ultra = -1
        print("Distance:", int(current_distance), "mm | Raw:", raw_ultra, "mm")
        print("Error:", int(distance_error), "mm")
        print("Δd:", int(delta_d), "mm", "Δθ:", int(delta_theta_step), "deg", "gyro_dev:", int(gyro_deviation), "deg")
        print("PersistFar:", persist_pos, "TrueFar:", is_true_far, "Corner:", corner_ticks, "Edge:", edge_ticks)
        print("Correction:", int(total_correction))
        print("Left Speed:", int(left_speed), "Right Speed:", int(right_speed))

        ev3.screen.clear()
        ev3.screen.draw_text(5, 5,  "D:" + str(int(current_distance)))
        ev3.screen.draw_text(5, 20, "Raw:" + str(raw_ultra))
        ev3.screen.draw_text(5, 35, "Err:" + str(int(distance_error)))
        ev3.screen.draw_text(5, 50, "Corr:" + str(int(total_correction)))
        ev3.screen.draw_text(5, 65, "L/R:" + str(int(left_speed)) + "/" + str(int(right_speed)))
        if corner_ticks > 0:
            ev3.screen.draw_text(5, 80, "CORNER<<")
        elif edge_ticks > 0 and not is_true_far:
            ev3.screen.draw_text(5, 80, "EDGE<<")  # 左靠
        else:
            if current_distance < TARGET_DISTANCE - 20:
                ev3.screen.draw_text(5, 80, "TOO CLOSE >>")
            elif current_distance > TARGET_DISTANCE + 20:
                ev3.screen.draw_text(5, 80, "<< TOO FAR")
            else:
                ev3.screen.draw_text(5, 80, "GOOD")

        # 里程
        avg_motor_angle = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        wheel_circumference = math.pi * WHEEL_DIAMETER_MM
        distance_traveled = (avg_motor_angle / 360) * wheel_circumference

        prev_gyro = current_gyro
        last_distance = current_distance

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
    try:
        ev3.speaker.beep()
        print("="*50)
        print("DIAGNOSTIC VERSION")
        print("Detailed Speed Output")
        print("="*50)
        print("Press CENTER to start...")

        while True:
            if Button.CENTER in ev3.buttons.pressed():
                break
            wait(10)

        ev3.speaker.beep()
        wait(1000)

        # Objective 1
        print("\n" + "="*50)
        print("OBJECTIVE 1: Detect Wall")
        print("="*50)
        drive_until_collision_controlled(speed=DRIVE_SPEED)

        ev3.speaker.beep()
        wait(300)

        print("Backing up 30cm...")
        drive_straight_pid(-300, speed=DRIVE_SPEED)
        wait(300)

        # Objective 2
        print("\n" + "="*50)
        print("OBJECTIVE 2: Turn Right 90°")
        print("="*50)
        print("Turning...")
        turn_in_place_pid(90, speed=TURN_SPEED)
        wait(300)

        print("Resetting gyro...")
        gyro.reset_angle(0)
        wait(200)

        ev3.speaker.beep()
        print("Turn complete!")
        wait(300)

        # Objective 3
        print("\n" + "="*50)
        print("OBJECTIVE 3: Diagnostic Wall Following")
        print("="*50)
        follow_wall_diagnostic(
            target_distance_mm=300,
            wall_length_mm=2400,
            speed=DRIVE_SPEED
        )

        print("\n" + "="*50)
        print("SUCCESS!")
        print("="*50)
        for i in range(3):
            ev3.speaker.beep(frequency=800 + i*200, duration=100)
            wait(150)

    except Exception as e:
        print("\n" + "="*50)
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
