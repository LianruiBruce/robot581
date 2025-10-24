#!/usr/bin/env pybricks-micropython
# ======================================================================
# COMP 581 - Lab 02: Right-turn then Left-Wall Following (PID + Gyro)
# Authors: Lianrui Geng, Xinyi Guo
# Date:    Oct 2025
#
# API policy: only use ev3brick, ev3devices, parameters, tools (+ math).
# No "robotics" module. No wireless. (See lab PDF.)
# ======================================================================

import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch

# ============================ USER TUNABLES =============================

# Ports
LEFT_MOTOR_PORT  = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_PORT       = Port.S1          # front bumper (safety)
GYRO_PORT        = Port.S2
ULTRA_PORT       = Port.S4          # ultrasonic faces LEFT side

# Geometry
WHEEL_DIAMETER_MM = 56.0
TRACK_WIDTH_MM    = 125.0           # wheel-to-wheel (approx; for reference)

# Speeds
CRUISE_MM_S       = 180             # base forward speed (wall-follow)
APPROACH_MM_S     = 180             # approach speed (task 1)
MAX_MM_S          = 260             # absolute cap for motors
TURN_DPS          = 140             # spin speed for yaw control

# Task thresholds / goals
WALL_DETECT_MM    = 300             # start turn when <= 30 cm
TARGET_LATERAL_MM = 220             # keep ~22 cm to wall (<=30 cm requirement)
FOLLOW_DISTANCE_MM= 2200            # travel along wall 2.2 m (odometry)
TIMEOUT_MS        = 90_000          # 90 s limit

# Ultrasonic smoothing (0..1, higher means smoother)
ULTRA_SMOOTHING   = 0.6

# ---- PID on lateral distance (to LEFT wall) ----
# error e = TARGET_LATERAL_MM - distance_left_mm
PID_KP = 0.022
PID_KI = 0.000    # keep 0 first; add ~0.0002 if存在固定偏差
PID_KD = 0.009

# ---- Gyro heading hold ----
# steer_yaw = K_YAW * (heading_set_deg - gyro.angle())
K_YAW = 0.9

# ---- Adaptive "bend" tracking (update heading_set_deg slowly) ----
# heading_set_deg += K_BEND * (e - e_prev)
K_BEND = 0.006     # deg per mm of error change; 0.004~0.010 常见

# Steering limit (dimensionless; 1.0 -> full split)
STEER_LIMIT = 0.60

# ============================== HELPERS =================================

def clamp(x, lo, hi):
    return hi if x > hi else lo if x < lo else x

def mmps_to_dps(v_mm_s: float) -> float:
    return (v_mm_s / (math.pi * WHEEL_DIAMETER_MM)) * 360.0

def run_lr_mmps(v_l_mm_s: float, v_r_mm_s: float):
    v_l_mm_s = clamp(v_l_mm_s, -MAX_MM_S, MAX_MM_S)
    v_r_mm_s = clamp(v_r_mm_s, -MAX_MM_S, MAX_MM_S)
    left.run(mmps_to_dps(v_l_mm_s))
    right.run(mmps_to_dps(v_r_mm_s))

def stop_hold():
    left.hold(); right.hold()

def smooth_ultra(prev_mm: float) -> float:
    d = ultra.distance()
    if d is None:          # keep previous when None
        return prev_mm
    if prev_mm <= 0:
        return d
    return ULTRA_SMOOTHING * prev_mm + (1.0 - ULTRA_SMOOTHING) * d

# ============================== DEVICES =================================

ev3   = EV3Brick()
left  = Motor(LEFT_MOTOR_PORT)
right = Motor(RIGHT_MOTOR_PORT)
bump  = TouchSensor(TOUCH_PORT)
gyro  = GyroSensor(GYRO_PORT)
ultra = UltrasonicSensor(ULTRA_PORT)

# ============================ CONTROL PRIMS ==============================

def spin_to_heading(target_deg: float):
    """Spin-in-place to absolute gyro angle (simple PID)."""
    Kp, Ki, Kd = 3.0, 0.0, 8.0
    integ, last_e = 0.0, 0.0
    sw = StopWatch(); sw.reset()
    while True:
        e = target_deg - gyro.angle()
        if abs(e) < 1.5:
            break
        dt = max(sw.time(), 1)/1000.0; sw.reset()
        integ += e*dt
        deriv = (e - last_e)/dt
        last_e = e
        u = clamp(Kp*e + Ki*integ + Kd*deriv, -200, 200)  # deg/s like
        left.run(-abs(u))
        right.run(+abs(u))
        wait(10)
    stop_hold()

def run_straight_with_yaw(v_mm_s: float, heading_deg: float):
    """P-only yaw hold using gyro."""
    yaw_err = heading_deg - gyro.angle()
    steer_yaw = clamp(K_YAW * (yaw_err/90.0), -0.25, 0.25)  # small normalization
    v_l = v_mm_s * (1.0 + steer_yaw)
    v_r = v_mm_s * (1.0 - steer_yaw)
    run_lr_mmps(v_l, v_r)

# ============================== STAGES ==================================

def stage_1_approach_wall():
    """Straight until <=30cm to wall (front-facing)."""
    gyro.reset_angle(0)
    wait(300)

    ev3.screen.clear()
    ev3.screen.print("Press center to start")
    # clean press-release
    while ev3.buttons.pressed(): wait(10)
    while not ev3.buttons.pressed(): wait(10)
    while ev3.buttons.pressed(): wait(10)
    ev3.speaker.beep()

    filt = 0.0
    while True:
        run_straight_with_yaw(APPROACH_MM_S, 0.0)
        filt = smooth_ultra(filt)
        if filt > 0 and filt <= WALL_DETECT_MM:
            break
        if bump.pressed():
            break
        wait(10)
    stop_hold()

def stage_2_right_turn_90():
    cur = gyro.angle()
    spin_to_heading(cur - 90.0)
    ev3.speaker.beep(frequency=900, duration=150)

def stage_3_follow_left_wall_pid():
    """
    After right turn, follow the LEFT wall:
    - Ultrasonic faces LEFT (perpendicular to wall ideally).
    - PID on lateral distance.
    - Slowly adapt desired heading via error change (handles bends).
    """
    heading_set = gyro.angle()    # start from current yaw after right turn
    e_int = 0.0
    e_prev = 0.0
    filt = 0.0
    last_l, last_r = left.angle(), right.angle()
    traveled = 0.0

    t_guard = StopWatch(); t_guard.reset()
    loop = StopWatch(); loop.reset()

    while traveled < FOLLOW_DISTANCE_MM:
        # time limit safeguard (per lab)
        if t_guard.time() > TIMEOUT_MS:
            break

        # read/filter distance to LEFT wall
        filt = smooth_ultra(filt)
        left_dist = filt  # sensor正左，读数即近似垂直距离（注意安装高度尽量水平）

        # --- PID on lateral error ---
        e = TARGET_LATERAL_MM - left_dist
        dt = max(loop.time(), 1)/1000.0; loop.reset()
        e_int += e * dt
        de = (e - e_prev) / dt
        e_prev = e

        steer_pid = PID_KP*e + PID_KI*e_int + PID_KD*de

        # --- Adaptive heading setpoint for bends ---
        # 如果误差在前进中持续上升(我们逐渐远离墙)，就把目标航向左调一点；反之右调。
        heading_set += K_BEND * (e)    # 也可以用 (e - e_prev)；这里用 e 累积效果更平滑

        # --- Yaw hold using gyro ---
        yaw_err = heading_set - gyro.angle()
        steer_yaw = 0.012 * yaw_err     # 小比例就够了（直接用度）

        # 合成转向并限幅（正->左转）
        steer = clamp(steer_pid + steer_yaw, -STEER_LIMIT, STEER_LIMIT)

        v = CRUISE_MM_S
        v_l = v * (1.0 + steer)
        v_r = v * (1.0 - steer)
        run_lr_mmps(v_l, v_r)

        # odometry
        a_l, a_r = left.angle(), right.angle()
        ds_l = (a_l - last_l)/360.0 * (math.pi*WHEEL_DIAMETER_MM)
        ds_r = (a_r - last_r)/360.0 * (math.pi*WHEEL_DIAMETER_MM)
        last_l, last_r = a_l, a_r
        traveled += abs(0.5*(ds_l + ds_r))

        # safety:碰撞->后退+微调再继续
        if bump.pressed():
            stop_hold()
            back = mmps_to_dps(140)
            left.run(-back); right.run(-back); wait(500)
            stop_hold()
            spin_to_heading(gyro.angle() - 8.0)  # 稍微向右，避免刮墙
            heading_set = gyro.angle()

        # 丢墙处理：读数过大(>600mm)时先按当前heading直行几百毫秒再轻微向左搜索
        if left_dist is not None and left_dist > 600:
            run_straight_with_yaw(v, heading_set)
            wait(300)
            heading_set += 4.0  # 向墙轻探
        wait(10)

    stop_hold()
    ev3.speaker.beep(frequency=1200, duration=300)
    ev3.screen.clear()
    ev3.screen.print("Done. dist=%.0fmm" % traveled)

# =============================== MAIN ==================================

def main():
    stage_1_approach_wall()   # 直行探墙 (<=30cm)
    stage_2_right_turn_90()   # 右转 90°
    stage_3_follow_left_wall_pid()  # 贴左墙 2.2m

if __name__ == "__main__":
    main()
