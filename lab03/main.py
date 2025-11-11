#!/usr/bin/env pybricks-micropython
# Refactor of Yunqi Li & Chenrui Li wall-follow algorithm
# Ports remapped to:
#   LEFT_MOTOR_PORT = Port.B
#   RIGHT_MOTOR_PORT = Port.C
#   TOUCH_LEFT_PORT = Port.S1
#   TOUCH_RIGHT_PORT = Port.S3
#   GYRO_PORT = Port.S2
#   ULTRA_PORT = Port.S4  (ultrasonic facing LEFT)
#
# Behavior preserved: initial approach → backoff → right 90° → wall-follow (left) → close loop → finish
# Added: global (x,y) mapping to lab's world frame with HIT_POINT at (2000, 500) for exit condition.

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop, Button
from pybricks.tools import wait, StopWatch
from math import pi, sqrt, cos, sin, radians

# ============================ HARDWARE PORTS =============================
LEFT_MOTOR_PORT  = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_LEFT_PORT  = Port.S1
TOUCH_RIGHT_PORT = Port.S3
GYRO_PORT        = Port.S2
ULTRA_PORT       = Port.S4

# ============================ HARDWARE ==================================
ev3 = EV3Brick()
left_motor  = Motor(LEFT_MOTOR_PORT,  positive_direction=Direction.CLOCKWISE)
right_motor = Motor(RIGHT_MOTOR_PORT, positive_direction=Direction.CLOCKWISE)
gyro        = GyroSensor(GYRO_PORT)
touch_left  = TouchSensor(TOUCH_LEFT_PORT)
touch_right = TouchSensor(TOUCH_RIGHT_PORT)
ultrasonic  = UltrasonicSensor(ULTRA_PORT)

def bump_pressed():
    """Return True if either front bumper is pressed."""
    return touch_left.pressed() or touch_right.pressed()

# ============================ GEOMETRY / UNITS ===========================
WHEEL_DIAM_MM  = 56
TRACK_WIDTH_MM = 118
MM_PER_DEG     = (pi * WHEEL_DIAM_MM) / 360.0

# ============================ CONTROL GAINS ==============================
Kp_h, Kd_h = 4.0, 0.04        # heading PD (gyro angle+rate)
Kp_d, Kd_d = 0.007, 0.002     # wall PD (distance and d/dt)
EMA_ALPHA  = 0.25
MAX_D_STEP = 30
MAX_DIFF   = 250

# ============================ SPEEDS ====================================
SPEED_MM_S        = 220       # initial approach speed (mm/s)
FOLLOW_SPEED_MM_S = 160       # wall-follow speed (mm/s)
MAX_SPEED         = 400       # deg/s caps for Motor.run
MIN_SPEED         = 120
MAX_SPEED_FOLLOW  = 340
MIN_SPEED_FOLLOW  = 90

# ============================ MISSION PARAMS =============================
TARGET_DIST_MM   = 135        # desired left clearance
PERIM_MIN_MM     = 1500       # min perimeter before loop-close check
HIT_RADIUS_MM    = 100        # distance-to-HIT in global frame to stop following
HIT_BACKOFF_MM   = 140.0      # after first bump to set hitpoint
BUMP_BACKOFF_MM  = 100.0
BUMP_RIGHT_DEG   = 45.0
BUMP_SURGE_MM    = 120.0

# Corner recovery / sensor artifacts
CORNER_PEEK_RIGHT_DEG = 30.0
CORNER_SURGE_MM       = 120.0
ULTRA_TAPE_GLITCH     = 337
ULTRA_MAX_RANGE       = 2500

# Sharp LEFT-corner (~120°) handling
LEFT_CORNER_GAP      = 90.0
LEFT_CORNER_DE_DOT   = 700.0
CORNER_PEEK_LEFT_DEG = 60.0
CORNER_SURGE_LEFT_MM = 160.0

# ============================ LAB WORLD FRAME (NEW) ======================
# Define the lab's global coordinates where the hit point is known/required.
HIT_POINT_X_MM = 2000.0
HIT_POINT_Y_MM = 500.0

# Local odom (x,y) origin is set at hit point after backoff+right turn.
# We'll map local -> global by a constant offset set at that moment.
global_x = 0.0
global_y = 0.0
_gx_offset = 0.0   # world_x = _gx_offset + x
_gy_offset = 0.0   # world_y = _gy_offset + y

def set_global_origin_at_current_local_hit_point():
    """Map current local odom origin (x=0,y=0) to lab's HIT_POINT (2000, 500)."""
    global _gx_offset, _gy_offset
    _gx_offset = HIT_POINT_X_MM
    _gy_offset = HIT_POINT_Y_MM
    update_global_xy()  # sync once

def update_global_xy():
    """Recompute global_x/global_y from local odom (x,y)."""
    global global_x, global_y
    global_x = _gx_offset + x
    global_y = _gy_offset + y

def dist_global_to_hit():
    dx = global_x - HIT_POINT_X_MM
    dy = global_y - HIT_POINT_Y_MM
    return sqrt(dx*dx + dy*dy)

# ============================ HELPERS ====================================
def mps_to_dps(v_mm_s):
    """Convert linear wheel speed (mm/s) to motor deg/s."""
    return (v_mm_s / (pi * WHEEL_DIAM_MM)) * 360.0

def set_wheels(vL, vR, vmax, vmin):
    """Apply speed with clamp and deadband floor."""
    vL = max(-vmax, min(vmax, vL))
    vR = max(-vmax, min(vmax, vR))
    if 0 < abs(vL) < vmin: vL = vmin if vL > 0 else -vmin
    if 0 < abs(vR) < vmin: vR = vmin if vR > 0 else -vmin
    left_motor.run(vL)
    right_motor.run(vR)

def gyro_zero():
    return gyro.angle()

def gyro_angle_rel(base):
    """Current yaw (deg) relative to base reading."""
    return gyro.angle() - base

def turn_to_heading_with_base(target_deg, base, tol=2.0, kp=3.0, max_sp=200):
    """In-place turn to absolute heading (relative to `base`)."""
    while True:
        e = target_deg - gyro_angle_rel(base)
        if abs(e) < tol:
            break
        turn = max(-max_sp, min(max_sp, kp * e))
        left_motor.run( turn)
        right_motor.run(-turn)
        wait(10)  # keep inside loop
    left_motor.stop(Stop.HOLD)
    right_motor.stop(Stop.HOLD)

def drive_straight_heading_with_base(target_deg, base, speed_mm_s, stop_cond_fn, max_time_ms=300000):
    """Drive straight using heading PD, stop on `stop_cond_fn()` or timeout."""
    v_cmd = mps_to_dps(speed_mm_s)
    timer = StopWatch()
    while timer.time() < max_time_ms:
        theta = gyro_angle_rel(base) - target_deg
        omega = gyro.speed()
        head_cmd = Kp_h * (-theta) - Kd_h * omega
        headroom = MAX_SPEED - v_cmd
        head_cmd = max(-headroom, min(headroom, head_cmd))
        vL = v_cmd - head_cmd
        vR = v_cmd + head_cmd
        set_wheels(vL, vR, MAX_SPEED, MIN_SPEED)
        if stop_cond_fn():
            break
        wait(20)
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)

# ============================ ODOMETRY ===================================
x, y = 0.0, 0.0                 # origin set at hit point after backoff+right turn
_last_deg_avg = 0.0
base_heading = 0                # gyro baseline at odom_reset

def odom_reset():
    """Zero position; set gyro baseline."""
    global x, y, _last_deg_avg, base_heading
    x = 0.0
    y = 0.0
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    _last_deg_avg = 0.0
    base_heading = gyro_zero()
    update_global_xy()  # keep global in sync

def odom_update():
    """Integrate translation only; in-place turns give ~0 ds (avg wheel motion)."""
    global x, y, _last_deg_avg
    deg_avg = 0.5 * (left_motor.angle() + right_motor.angle())
    ddeg = deg_avg - _last_deg_avg
    _last_deg_avg = deg_avg
    ds = ddeg * MM_PER_DEG
    th = radians(gyro_angle_rel(base_heading))
    x += ds * cos(th)
    y += ds * sin(th)
    update_global_xy()  # NEW: sync global each update
    return ds

def odom_freeze_wheels():
    """Reset odom encoder baseline WITHOUT changing x,y (use after in-place turns)."""
    global _last_deg_avg
    _last_deg_avg = 0.5 * (left_motor.angle() + right_motor.angle())

def dist_mm(ax, ay, bx, by):
    dx, dy = (ax - bx), (ay - by)
    return sqrt(dx*dx + dy*dy)

def turn_to_heading(target_deg, tol=2.0, kp=3.0, max_sp=200):
    turn_to_heading_with_base(target_deg, base_heading, tol, kp, max_sp)

def drive_straight_heading(target_deg, speed_mm_s, stop_cond_fn, max_time_ms=300000):
    drive_straight_heading_with_base(target_deg, base_heading, speed_mm_s, stop_cond_fn, max_time_ms)

def print_xy():
    ev3.screen.clear()
    ev3.screen.print("loc x={:.0f} y={:.0f}".format(x, y))
    ev3.screen.print("G  X={:.0f} Y={:.0f}".format(global_x, global_y))

# ============================ MAIN SEQUENCE ==============================
def main():
    ev3.speaker.beep()
    ev3.screen.clear()
    ev3.screen.print("按下中键开始")
    while Button.CENTER not in ev3.buttons.pressed():
        wait(50)
    ev3.speaker.beep()
    wait(300)

    # 1) INITIAL APPROACH: drive straight until bump; record approach distance from START
    base_init = gyro_zero()
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    drive_straight_heading_with_base(
        target_deg=0.0,
        base=base_init,
        speed_mm_s=SPEED_MM_S,
        stop_cond_fn=bump_pressed
    )
    ev3.speaker.beep()

    encoder_avg_deg = 0.5 * (left_motor.angle() + right_motor.angle())
    approach_contact_mm = encoder_avg_deg * MM_PER_DEG  # used at the very end to return to start

    # 2) BACK OFF → RIGHT 90° (relative to base_init) → SET ODOM ORIGIN
    backoff_deg = (HIT_BACKOFF_MM / (pi * WHEEL_DIAM_MM)) * 360.0
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    left_motor.run_target(speed=240, target_angle=-backoff_deg, then=Stop.HOLD, wait=False)
    right_motor.run_target(speed=240, target_angle=-backoff_deg, then=Stop.HOLD, wait=True)

    base_init = gyro_zero()
    current_init = gyro_angle_rel(base_init)
    turn_to_heading_with_base(current_init + 90.0, base=base_init, tol=2.0, kp=3.0, max_sp=200)

    # Set odometry origin and heading (local origin is the hit point)
    odom_reset()

    # === NEW: set global mapping so local (0,0) aligns to lab HIT_POINT (2000,500)
    set_global_origin_at_current_local_hit_point()

    # 3) WALL FOLLOW (LEFT ULTRASONIC) until back near HIT_POINT in global frame after enough perimeter
    timer = StopWatch()
    S_wall = 0.0
    d_s = ultrasonic.distance() or TARGET_DIST_MM
    d_prev = d_s
    e_prev = 0.0
    v_cmd_follow = mps_to_dps(FOLLOW_SPEED_MM_S)

    # Dead-reckoning timing (for x,y and S_wall only)
    last_time_ms = timer.time()
    
    while True:
        loop_t = timer.time()

        # Real elapsed dt (sec) for odometry/perimeter
        now_ms = timer.time()
        dt_s = max(0.01, (now_ms - last_time_ms) / 1000.0)

        # (A) Bump safety
        if bump_pressed():
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)

            # back off (translation)
            bump_backoff_deg = (BUMP_BACKOFF_MM / (pi * WHEEL_DIAM_MM)) * 360.0
            left_motor.run_angle(speed=240, rotation_angle=-bump_backoff_deg, then=Stop.HOLD, wait=False)
            right_motor.run_angle(speed=240, rotation_angle=-bump_backoff_deg, then=Stop.HOLD, wait=True)
            odom_update()

            # small right turn (in-place)
            curr = gyro_angle_rel(base_heading)
            turn_to_heading(curr + BUMP_RIGHT_DEG, tol=2.0, kp=3.0, max_sp=200)
            odom_freeze_wheels()   # prevent phantom translation after in-place turn

            # short surge (translation)
            surge_deg = (BUMP_SURGE_MM / (pi * WHEEL_DIAM_MM)) * 360.0
            left_motor.run_angle(speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=False)
            right_motor.run_angle(speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=True)
            odom_update()

            wait(40)
            last_time_ms = timer.time()
            continue

        # (B) Ultrasonic read + tape/corner handling
        d_raw = ultrasonic.distance()

        # "No wall" → peek right + surge
        if d_raw is not None and d_raw >= ULTRA_MAX_RANGE:
            curr = gyro_angle_rel(base_heading)
            peek_right = curr - CORNER_PEEK_RIGHT_DEG
            turn_to_heading(peek_right, tol=3.0, kp=3.0, max_sp=200)
            odom_freeze_wheels()
            surge_deg = 360.0 * CORNER_SURGE_MM / (pi * WHEEL_DIAM_MM)
            left_motor.run_angle(speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=False)
            right_motor.run_angle(speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=True)
            odom_update()
            d_raw = ultrasonic.distance() or d_s

        # Tape glitch window → treat as mild increase; also clamp per-step jump
        if d_raw is None or (d_raw > ULTRA_TAPE_GLITCH and d_raw < ULTRA_MAX_RANGE):
            d_raw = d_s + MAX_D_STEP
        if abs(d_raw - d_s) > MAX_D_STEP:
            d_raw = d_s + (MAX_D_STEP if d_raw > d_s else -MAX_D_STEP)

        # EMA smoothing
        d_s = (1 - EMA_ALPHA) * d_s + EMA_ALPHA * d_raw

        # (B2) Sharp LEFT-corner detect (~20 ms derivative timing)
        e_prov  = d_s - TARGET_DIST_MM
        de_prov = (d_s - d_prev) / 0.02

        if (e_prov > LEFT_CORNER_GAP) and (de_prov > LEFT_CORNER_DE_DOT):
            curr = gyro_angle_rel(base_heading)
            turn_to_heading(curr + CORNER_PEEK_LEFT_DEG, tol=3.0, kp=3.0, max_sp=220)
            odom_freeze_wheels()
            surge_deg = 360.0 * CORNER_SURGE_LEFT_MM / (pi * WHEEL_DIAM_MM)
            left_motor.run_angle(speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=False)
            right_motor.run_angle(speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=True)
            odom_update()

            d_prev = d_s
            d_raw = ultrasonic.distance() or d_s
            wait(20)
            last_time_ms = timer.time()
            continue

        # (C) Wall PD + gyro damping (~20 ms diff)
        e  = d_s - TARGET_DIST_MM
        de = (e - e_prev) / 0.02
        e_prev = e
        steer_d = Kp_d * e + Kd_d * de
        omega = gyro.speed()
        steer_h = -Kd_h * omega

        diff = (steer_d * 180.0) + steer_h
        diff = max(-MAX_DIFF, min(MAX_DIFF, diff))

        # (D) Apply speeds
        vL = v_cmd_follow - diff
        vR = v_cmd_follow + diff
        set_wheels(vL, vR, MAX_SPEED_FOLLOW, MIN_SPEED_FOLLOW)

        # (E) Odom + perimeter (use real dt)
        ds = odom_update()
        v_mm_s = abs(ds) / dt_s
        d_dot  = (d_s - d_prev) / dt_s
        d_prev = d_s
        v_parallel = sqrt(max(v_mm_s*v_mm_s - d_dot*d_dot, 0.0))
        S_wall += v_parallel * dt_s

        # (F) Exit condition: only after enough perimeter and near HIT_POINT in GLOBAL frame
        if S_wall > PERIM_MIN_MM and dist_global_to_hit() < HIT_RADIUS_MM:
            ev3.speaker.beep()
            break

        print_xy()
        spent = timer.time() - loop_t
        wait(max(1, 20 - spent))
        last_time_ms = timer.time()

    left_motor.stop(Stop.HOLD)
    right_motor.stop(Stop.HOLD)

    # 4) FINAL: TURN LEFT 90° (relative), HIT WALL, BACK OFF BY INITIAL APPROACH TO RETURN TO START
    curr = gyro_angle_rel(base_heading)
    target_after_left = curr - 90.0
    turn_to_heading(target_after_left, tol=2.0, kp=3.0, max_sp=200)

    drive_straight_heading(
        target_deg=target_after_left,
        speed_mm_s=FOLLOW_SPEED_MM_S,
        stop_cond_fn=bump_pressed,
        max_time_ms=120000
    )
    ev3.speaker.beep()

    final_backoff_deg = (approach_contact_mm / (pi * WHEEL_DIAM_MM)) * 360.0
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    left_motor.run_target(speed=240, target_angle=-final_backoff_deg, then=Stop.HOLD, wait=False)
    right_motor.run_target(speed=240, target_angle=-final_backoff_deg, then=Stop.HOLD, wait=True)

    left_motor.stop(Stop.HOLD)
    right_motor.stop(Stop.HOLD)
    ev3.speaker.beep()

# ============================ ENTRYPOINT ================================
if __name__ == "__main__":
    main()
