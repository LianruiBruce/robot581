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
    Conservative wall following:
      - median-like raw gate + EMA (raw 用于内折触发，EMA 用于控制)
      - symmetric deadband
      - gentle left-bias on outside corner / drop to void
      - stronger but safe right-bias on inside corner
      - slew-limited correction
    """
    print("="*50)
    print("DIAGNOSTIC WALL FOLLOWING")
    print("="*50)
    print("Target:", target_distance_mm, "mm")
    print("Length:", wall_length_mm, "mm")

    TARGET_DISTANCE   = target_distance_mm

    # 基础控制（温和、稳定）
    CORRECTION_GAIN   = 1.4
    MAX_CORRECTION    = 100
    MAX_DELTA_CORR    = 20
    GYRO_ASSIST       = 0.0
    REVERSE_CORRECTION = False

    # 平滑 & 量程
    ALPHA             = 0.5      # 若抖可降到 0.45；若迟钝可升到 0.6
    MAX_WALL_MM       = 600      # 超出视为无效

    # 对称死区
    DEADBAND_MM       = 12

    # 外折/探空（读数突然变远 → 左靠墙）
    EDGE_RISE_MM      = 35
    EDGE_TICKS_HOLD   = 8
    EDGE_BIAS         = +50      # 正=左转（你的速度定义下）

    # 内折（读数突然变近/已很近 → 右离墙）
    CORNER_TICKS      = 8        # 保持几拍，避免刚触发就放掉
    CORNER_BIAS       = -60      # 负=右转（比外折稍强一点）
    # 触发阈值（用 raw，更灵敏）
    RAW_NEAR_OFFSET   = 45       # raw <= TARGET-45 视为已很近
    RAW_DROP          = -40      # 单拍下降超过 40mm 视为内折

    print("CORRECTION_GAIN:", CORRECTION_GAIN)
    print("MAX_CORRECTION:", MAX_CORRECTION)
    print("="*50)

    # 初始化
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    parallel_gyro_reference = gyro.angle()

    last_distance   = TARGET_DISTANCE     # 上一拍 EMA 后的距离
    last_correction = 0.0
    edge_ticks      = 0
    corner_ticks    = 0
    iteration       = 0

    while True:
        iteration += 1

        # ---------- 读原始距离 raw（带量程门控） ----------
        try:
            raw = ultrasonic.distance()
        except:
            raw = last_distance
        if raw <= 0 or raw > MAX_WALL_MM:
            raw = last_distance

        # ---------- 计算 EMA 后的 current_distance ----------
        current_distance = ALPHA * raw + (1 - ALPHA) * last_distance

        # ---------- 差分 ----------
        delta_d    = current_distance - last_distance
        current_gyro = gyro.angle()
        gyro_dev     = current_gyro - parallel_gyro_reference

        # ---------- 外折/探空触发（raw 或 EMA 突然变远） ----------
        if edge_ticks == 0 and ( (current_distance - last_distance) >= EDGE_RISE_MM or raw >= MAX_WALL_MM ):
            edge_ticks = EDGE_TICKS_HOLD

        # ---------- 内折触发（用 raw：更早感知“靠近”） ----------
        if corner_ticks == 0:
            raw_near = TARGET_DISTANCE - RAW_NEAR_OFFSET
            if (raw <= raw_near) or (raw - last_distance <= RAW_DROP):
                corner_ticks = CORNER_TICKS

        # ---------- 误差 & 纠偏（对称死区） ----------
        distance_error = current_distance - TARGET_DISTANCE
        if abs(distance_error) <= DEADBAND_MM:
            distance_correction = 0.0
        else:
            distance_correction = CORRECTION_GAIN * distance_error

        distance_correction = clamp(distance_correction, -MAX_CORRECTION, MAX_CORRECTION)

        # ---------- 陀螺辅助（保持关闭），大偏差只限幅不改向 ----------
        total_correction = distance_correction + GYRO_ASSIST * gyro_dev
        if abs(gyro_dev) > 60:
            total_correction = clamp(total_correction, -MAX_CORRECTION, MAX_CORRECTION)

        # ---------- 角点偏置（短时、温和） ----------
        if corner_ticks > 0:
            # 内折：右转远离墙
            total_correction += CORNER_BIAS
            # 若已明显太近，给一个右转力度的底线（避免太弱）
            if current_distance <= TARGET_DISTANCE - 60:
                MIN_RIGHT = 35  # 可调 30~45
                if total_correction > -MIN_RIGHT:
                    total_correction = -MIN_RIGHT
            corner_ticks -= 1

        elif edge_ticks > 0:
            # 外折/探空：左转贴回墙
            total_correction += EDGE_BIAS
            edge_ticks -= 1

        # ---------- 爬升限速（抑制猛甩） ----------
        total_correction = slew_toward(last_correction, total_correction, MAX_DELTA_CORR)
        last_correction  = total_correction

        if REVERSE_CORRECTION:
            total_correction = -total_correction

        # ---------- 电机输出 ----------
        left_speed  = speed - total_correction
        right_speed = speed + total_correction

        # 内折期间允许更低最小速度，便于“掉头”远离墙
        min_speed = 30 if corner_ticks > 0 else 40
        max_speed = speed * 1.6
        left_speed  = clamp(left_speed,  min_speed, max_speed)
        right_speed = clamp(right_speed, min_speed, max_speed)

        left_motor.run(left_speed)
        right_motor.run(right_speed)

        # ---------- 终端 & 屏显 ----------
        print("="*50)
        print("Iter:", iteration,
              "| D:", int(current_distance), "raw:", int(raw),
              "| Err:", int(distance_error), "dD:", int(delta_d),
              "| edge:", edge_ticks, "corner:", corner_ticks,
              "| Corr:", int(total_correction),
              "| L/R:", int(left_speed), "/", int(right_speed))

        ev3.screen.clear()
        ev3.screen.draw_text(5, 5,  "D:" + str(int(current_distance)))
        ev3.screen.draw_text(5, 20, "Raw:" + str(int(raw)))
        ev3.screen.draw_text(5, 35, "Err:" + str(int(distance_error)))
        ev3.screen.draw_text(5, 50, "Corr:" + str(int(total_correction)))
        if edge_ticks > 0:
            ev3.screen.draw_text(5, 80, "EDGE<<")    # 外折：左靠
        elif corner_ticks > 0:
            ev3.screen.draw_text(5, 80, "CORNER>>")  # 内折：右离
        else:
            if distance_error < -20:
                ev3.screen.draw_text(5, 80, "TOO CLOSE >>")
            elif distance_error > 20:
                ev3.screen.draw_text(5, 80, "<< TOO FAR")
            else:
                ev3.screen.draw_text(5, 80, "GOOD")

        # ---------- 里程终止 ----------
        avg_motor_angle   = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        wheel_circumference = math.pi * WHEEL_DIAMETER_MM
        distance_traveled = (avg_motor_angle / 360) * wheel_circumference

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
    """Main program"""
    try:
        ev3.speaker.beep()
        print("="*50)
        print("DIAGNOSTIC VERSION")
        print("Detailed Speed Output")
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
