#!/usr/bin/env pybricks-micropython
# COMP 581 Lab 03 – Boundary Tracing & Return to Start
# Wiring (as requested):
#   LEFT_MOTOR_PORT = Port.B
#   RIGHT_MOTOR_PORT = Port.C
#   TOUCH_LEFT_PORT = Port.S1
#   TOUCH_RIGHT_PORT = Port.S3
#   GYRO_PORT = Port.S2
#   ULTRA_PORT = Port.S4   (LEFT-facing ultrasonic)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop, Button
from pybricks.tools import wait, StopWatch
from math import pi, sqrt, cos, sin, radians

# ============================ PORTS / HARDWARE ============================
LEFT_MOTOR_PORT  = Port.B
RIGHT_MOTOR_PORT = Port.C
TOUCH_LEFT_PORT  = Port.S1
TOUCH_RIGHT_PORT = Port.S3
GYRO_PORT        = Port.S2
ULTRA_PORT       = Port.S4

ev3 = EV3Brick()
LEFT   = Motor(LEFT_MOTOR_PORT,  positive_direction=Direction.CLOCKWISE)
RIGHT  = Motor(RIGHT_MOTOR_PORT, positive_direction=Direction.CLOCKWISE)
GYRO   = GyroSensor(GYRO_PORT)
TOUCH_L = TouchSensor(TOUCH_LEFT_PORT)
TOUCH_R = TouchSensor(TOUCH_RIGHT_PORT)
ULTRA   = UltrasonicSensor(ULTRA_PORT)

def bump_pressed():
    return TOUCH_L.pressed() or TOUCH_R.pressed()

# ============================ GEOMETRY / UNITS ============================
WHEEL_DIAM_MM  = 56.0
TRACK_WIDTH_MM = 118.0
MM_PER_DEG     = (pi * WHEEL_DIAM_MM) / 360.0

# ============================ GAINS / SPEEDS ==============================
# 角度 & 贴墙控制
Kp_h, Kd_h = 4.0, 0.05
Kp_d, Kd_d = 0.007, 0.002

# 速度（整体放慢，更稳）
SPEED_MM_S        = 180     # 初始靠墙直冲速度（mm/s）
FOLLOW_SPEED_MM_S = 140     # 贴墙巡航速度（mm/s）
MAX_RUN_DPS       = 380
MIN_RUN_DPS       = 90
MAX_RUN_FOLLOW    = 320
MIN_RUN_FOLLOW    = 80
MAX_DIFF_DPS      = 240

# ============================ MISSION PARAMS =============================
# 贴墙参数
TARGET_DIST_MM   = 135
EMA_ALPHA        = 0.25
MAX_D_STEP       = 30
ULTRA_TAPE_GLITCH = 337
ULTRA_MAX_RANGE   = 2500

# 转角增强
CORNER_PEEK_RIGHT_DEG = 30.0
CORNER_SURGE_MM       = 120.0
LEFT_CORNER_GAP       = 90.0
LEFT_CORNER_DE_DOT    = 700.0
CORNER_PEEK_LEFT_DEG  = 60.0
CORNER_SURGE_LEFT_MM  = 160.0

# 撞击/回退
HIT_BACKOFF_MM  = 140.0   # 碰撞后回退（设置 Hit 点）
BUMP_BACKOFF_MM = 100.0
BUMP_RIGHT_DEG  = 48.0    # 稍微加大，避免"角度略小"
BUMP_SURGE_MM   = 120.0

# 退出判据（基于绕墙起点）
PERIM_MIN_MM      = 1400           # 至少绕过一定周长才允许判定回到起点
RETURN_RADIUS_MM  = 100            # 与"绕墙起点"距离 < 100mm 触发
LOOP_CHECK_COOLDOWN_MS = 800       # 防止刚启动就误判

# ============================ STATE: GLOBAL ODOM ==========================
# 全局坐标：原点 = 物理出发点（按下开始键后）
x, y = 0.0, 0.0
_last_deg_avg = 0.0
gyro_offset = 0.0           # 保持全局航向的一致（重置陀螺时补偿）
heading_deg = 0.0           # 全局航向
start_x, start_y = 0.0, 0.0 # 记录真正的出发点（按键后）
wall_follow_start_x = 0.0   # 记录开始绕墙的点（右转90度后）
wall_follow_start_y = 0.0
approach_contact_mm = 0.0   # 记录"出发→碰撞"的直线距离（用于最终返回）
_last_heading_deg = GYRO.angle() + gyro_offset

def reset_encoder_baseline():
    """重置编码器基线：把当前位置当作(0,0)，同步编码器基线"""
    global _last_deg_avg
    LEFT.reset_angle(0); RIGHT.reset_angle(0)
    _last_deg_avg = 0.0

def sync_encoders_after_turn():
    """原地转弯后同步编码器：重设编码器基线，避免虚假位移"""
    global _last_deg_avg
    _last_deg_avg = 0.5 * (LEFT.angle() + RIGHT.angle())

def update_odometry():
    """
    修正版：基于相对运动积分的里程计更新
    解决问题：绕圈时距离只增加、不回原点
    """
    global x, y, heading_deg, _last_deg_avg, _last_heading_deg

    # --- 1️⃣ 当前编码器平均角度 ---
    deg_avg = 0.5 * (LEFT.angle() + RIGHT.angle())
    ddeg = deg_avg - _last_deg_avg
    _last_deg_avg = deg_avg

    # --- 2️⃣ 平均前进距离（mm）---
    ds = ddeg * MM_PER_DEG  # 单位：mm

    # --- 3️⃣ 当前与上次陀螺仪角度（°）---
    curr_heading_deg = - (GYRO.angle() + gyro_offset)
    dtheta_deg = curr_heading_deg - _last_heading_deg

    # 初始化时防止第一次跳变
    if '_last_heading_deg' not in globals():
        _last_heading_deg = curr_heading_deg
        dtheta_deg = 0

    # 更新记录
    _last_heading_deg = curr_heading_deg

    # --- 4️⃣ 转弧度 ---
    dtheta = radians(dtheta_deg)

    # 使用“上次角 + 一半旋转角”近似中间朝向
    th_mid = radians(heading_deg + dtheta_deg / 2.0)

    # --- 5️⃣ 微分运动学积分 ---
    if abs(dtheta) < 1e-6:
        # 小角近似直线
        dx = ds * cos(th_mid)
        dy = ds * sin(th_mid)
    else:
        # 弧形路径积分（精确版）
        R = ds / dtheta
        dx = R * (sin(th_mid + dtheta / 2) - sin(th_mid - dtheta / 2))
        dy = -R * (cos(th_mid + dtheta / 2) - cos(th_mid - dtheta / 2))

    # --- 6️⃣ 更新全局位置 ---
    x += dx
    y += dy
    heading_deg = curr_heading_deg

    return ds


def calc_distance(x1, y1, x2, y2):
    """计算两点之间的欧几里得距离(mm)"""
    dx, dy = x1 - x2, y1 - y2
    return sqrt(dx*dx + dy*dy)

# ============================ MOTION PRIMITIVES ===========================
def mps_to_dps(v_mm_s):
    return (v_mm_s / (pi * WHEEL_DIAM_MM)) * 360.0

def set_wheels(vL, vR, vmax, vmin):
    vL = max(-vmax, min(vmax, vL))
    vR = max(-vmax, min(vmax, vR))
    if 0 < abs(vL) < vmin: vL = vmin if vL > 0 else -vmin
    if 0 < abs(vR) < vmin: vR = vmin if vR > 0 else -vmin
    LEFT.run(vL); RIGHT.run(vR)

def turn_to_heading(target_deg, tol=1.5, kp=3.0, max_sp=240):
    """转到指定角度：原地转向到全局绝对航向（使用当前GYRO+offset）"""
    timer = StopWatch()
    while True:
        err = target_deg - (GYRO.angle() + gyro_offset)
        if abs(err) < tol:
            break
        turn = max(-max_sp, min(max_sp, kp * err))
        LEFT.run( turn); RIGHT.run(-turn)
        wait(10)
        update_odometry()  # 原地转弯理论上ds≈0，但保持刷新
        if timer.time() > 10000:
            break
    LEFT.stop(Stop.HOLD); RIGHT.stop(Stop.HOLD)
    sync_encoders_after_turn()

def drive_straight_with_heading(target_deg, speed_mm_s, stop_cond_fn=lambda: False, max_ms=300000):
    """保持航向直行：含陀螺微分阻尼，周期性刷新里程计与停止条件"""
    v_cmd = mps_to_dps(speed_mm_s)
    timer = StopWatch()
    while timer.time() < max_ms:
        theta = (GYRO.angle() + gyro_offset) - target_deg
        omega = GYRO.speed()
        head_cmd = Kp_h * (-theta) - Kd_h * omega
        headroom = MAX_RUN_DPS - v_cmd
        head_cmd = max(-headroom, min(headroom, head_cmd))
        vL = v_cmd - head_cmd; vR = v_cmd + head_cmd
        set_wheels(vL, vR, MAX_RUN_DPS, MIN_RUN_DPS)
        if stop_cond_fn():
            LEFT.stop(Stop.BRAKE); RIGHT.stop(Stop.BRAKE)
            update_odometry()
            return True
        wait(15)
        update_odometry()
    LEFT.stop(Stop.BRAKE); RIGHT.stop(Stop.BRAKE)
    update_odometry()
    return False

# ============================ MAIN SEQUENCE ===============================
def main():
    global x, y, gyro_offset, start_x, start_y, wall_follow_start_x, wall_follow_start_y, approach_contact_mm
    ev3.speaker.beep()
    # ----- Start-up -----
    print("Press CENTER to start")
    while Button.CENTER not in ev3.buttons.pressed():
        wait(50)
    ev3.speaker.beep()
    print("Starting...")
    wait(200)

    # 初始化里程计/陀螺
    LEFT.reset_angle(0); RIGHT.reset_angle(0)
    reset_encoder_baseline()
    gyro_offset = 0.0
    wait(50)
    update_odometry()  # x,y=0, heading=GYRO+offset
    start_x, start_y = x, y

    # 记录开始时的绝对朝向（用于最后回程的参考）
    start_heading_abs = GYRO.angle() + gyro_offset

    # ----- Phase 1: 直行撞墙（立停），记录"初始前进距离" -----
    # 为了快速稳停，采用较低速度 + 紧密循环
    approached = drive_straight_with_heading(
        target_deg=start_heading_abs,
        speed_mm_s=SPEED_MM_S,
        stop_cond_fn=bump_pressed,
        max_ms=120000
    )
    ev3.speaker.beep()
    print("Hit obstacle!")
    # 从出发到碰撞：用编码器平均角度换算，也可用 calc_distance(start, now)
    encoder_avg_deg = 0.5 * (LEFT.angle() + RIGHT.angle())
    approach_contact_mm = encoder_avg_deg * MM_PER_DEG
    print("Approach distance: {:.0f} mm".format(approach_contact_mm))

    # 保险：若编码器法异常，用里程计距离
    if approach_contact_mm < 10:
        approach_contact_mm = calc_distance(x, y, start_x, start_y)
        print("Using odometry distance: {:.0f} mm".format(approach_contact_mm))

    # ----- Phase 2: 回退至 Hit 点（与正前墙保持20cm），然后右转90° -----
    backoff_deg = (HIT_BACKOFF_MM / (pi * WHEEL_DIAM_MM)) * 360.0
    # 先清算再回退
    update_odometry()
    LEFT.run_target(speed=240, target_angle=LEFT.angle() - backoff_deg, then=Stop.HOLD, wait=False)
    RIGHT.run_target(speed=240, target_angle=RIGHT.angle() - backoff_deg, then=Stop.HOLD, wait=True)
    update_odometry()

    # 以当前绝对朝向为基准，右转90°
    curr_abs = GYRO.angle() + gyro_offset
    turn_to_heading(curr_abs + 90.0, tol=1.5, kp=3.2, max_sp=260)

    # ★★★ 记录开始绕墙的点（右转90度后的位置）★★★
    update_odometry()
    wall_follow_start_x = x
    wall_follow_start_y = y
    print("Wall follow start point: ({:.0f}, {:.0f}) mm".format(wall_follow_start_x, wall_follow_start_y))
    wait(500)

    # ----- Phase 3: 左侧贴墙绕行直到回到"绕墙起点"半径 < 10cm -----
    timer = StopWatch()
    S_wall = 0.0
    last_time_ms = timer.time()
    d_s = ULTRA.distance() or TARGET_DIST_MM
    d_prev = d_s
    e_prev = 0.0
    v_cmd_follow = mps_to_dps(FOLLOW_SPEED_MM_S)
    loop_start_ms = timer.time()
    
    while True:
        loop_t = timer.time()
        now_ms = timer.time()
        dt_s = max(0.01, (now_ms - last_time_ms) / 1000.0)

        # A) 撞击保护：立停→回退→小右转→小前冲（避免贴到凹角）
        if bump_pressed():
            LEFT.stop(Stop.BRAKE); RIGHT.stop(Stop.BRAKE)
            update_odometry()

            bump_back_deg = (BUMP_BACKOFF_MM / (pi * WHEEL_DIAM_MM)) * 360.0
            LEFT.run_angle( speed=240, rotation_angle=-bump_back_deg, then=Stop.HOLD, wait=False)
            RIGHT.run_angle(speed=240, rotation_angle=-bump_back_deg, then=Stop.HOLD, wait=True)
            update_odometry()

            curr_abs = GYRO.angle() + gyro_offset
            turn_to_heading(curr_abs + BUMP_RIGHT_DEG, tol=1.5, kp=3.2, max_sp=260)

            surge_deg = 360.0 * BUMP_SURGE_MM / (pi * WHEEL_DIAM_MM)
            LEFT.run_angle( speed=240, rotation_angle= surge_deg, then=Stop.HOLD, wait=False)
            RIGHT.run_angle(speed=240, rotation_angle= surge_deg, then=Stop.HOLD, wait=True)
            update_odometry()

            last_time_ms = timer.time()
            continue

        # B) 超声+拐角处理
        d_raw = ULTRA.distance()
        if d_raw is not None and d_raw >= ULTRA_MAX_RANGE:
            curr_abs = GYRO.angle() + gyro_offset
            turn_to_heading(curr_abs - CORNER_PEEK_RIGHT_DEG, tol=2.5, kp=3.0, max_sp=240)
            surge_deg = 360.0 * CORNER_SURGE_MM / (pi * WHEEL_DIAM_MM)
            LEFT.run_angle( speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=False)
            RIGHT.run_angle(speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=True)
            update_odometry()
            d_raw = ULTRA.distance() or d_s

        if d_raw is None or (d_raw > ULTRA_TAPE_GLITCH and d_raw < ULTRA_MAX_RANGE):
            d_raw = d_s + MAX_D_STEP
        if abs(d_raw - d_s) > MAX_D_STEP:
            d_raw = d_s + (MAX_D_STEP if d_raw > d_s else -MAX_D_STEP)
        d_s = (1 - EMA_ALPHA) * d_s + EMA_ALPHA * d_raw

        # (B2) Sharp LEFT-corner detect (~20 ms derivative timing)
        d_filtered = (1 - EMA_ALPHA) * d_prev + EMA_ALPHA * (ULTRA.distance() or d_prev)
        e_prov = d_filtered - TARGET_DIST_MM
        de_prov = (d_filtered - d_prev) / 0.02

        if (e_prov > LEFT_CORNER_GAP) and (de_prov > LEFT_CORNER_DE_DOT):
            curr_abs = (GYRO.angle() + gyro_offset)
            turn_to_heading(curr_abs + CORNER_PEEK_LEFT_DEG, tol=2.5, kp=3.0, max_sp=240)
            surge_deg = 360.0 * CORNER_SURGE_LEFT_MM / (pi * WHEEL_DIAM_MM)
            LEFT.run_angle( speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=False)
            RIGHT.run_angle(speed=240, rotation_angle=surge_deg, then=Stop.HOLD, wait=True)
            update_odometry()
            d_prev = d_s
            last_time_ms = timer.time()
            print("LEFT-corner? e_prov=%.1f de_prov=%.1f d_s=%.1f" % (e_prov, de_prov, d_s))
            continue

        # C) 贴墙控制 + 陀螺阻尼
        e  = d_s - TARGET_DIST_MM
        de = (e - e_prev) / 0.02
        e_prev = e
        steer_d = Kp_d * e + Kd_d * de
        omega = GYRO.speed()
        steer_h = -0.05 * omega
        diff = (steer_d * 180.0) + steer_h
        diff = max(-MAX_DIFF_DPS, min(MAX_DIFF_DPS, diff))

        vL = v_cmd_follow - diff
        vR = v_cmd_follow + diff
        set_wheels(vL, vR, MAX_RUN_FOLLOW, MIN_RUN_FOLLOW)

        # D) 里程计 + 周长
        ds = update_odometry()
        v_mm_s = abs(ds) / dt_s
        d_dot  = (d_s - d_prev) / dt_s
        d_prev = d_s
        v_parallel = sqrt(max(v_mm_s*v_mm_s - d_dot*d_dot, 0.0))
        S_wall += v_parallel * dt_s

        # E) ★★★ 回到"绕墙起点"的判定（必须已绕过一定距离 + 冷却时间）★★★
        dist_to_wall_start = calc_distance(x, y, wall_follow_start_x, wall_follow_start_y)
        if (timer.time() - loop_start_ms) > LOOP_CHECK_COOLDOWN_MS:
            if S_wall > PERIM_MIN_MM:
                if dist_to_wall_start < RETURN_RADIUS_MM:
                    # 命中：立停 + 完成回程动作
                    LEFT.stop(Stop.HOLD); RIGHT.stop(Stop.HOLD)
                    update_odometry()
                    ev3.speaker.beep()
                    print("Returned to wall start! Distance: {:.0f} mm".format(dist_to_wall_start))
                    print("Total perimeter: {:.0f} mm".format(S_wall))
                    wait(1000)
                    break

        # 定期打印状态（显示距离绕墙起点的距离）
        if (now_ms // 1000) != (last_time_ms // 1000):  # 每秒打印一次
            print("Position: ({:.0f}, {:.0f}) | Dist to wall-start: {:.0f} mm | Perimeter: {:.0f} mm".format(
                x, y, dist_to_wall_start, S_wall))
        spent = timer.time() - loop_t
        wait(max(1, 20 - spent))
        last_time_ms = timer.time()

    # ----- Phase 4: 命中起点 → 右转90° → 直行到真正的物理出发点 -----
    print("\n=== Phase 4: Returning to physical start point ===")
    curr_abs = GYRO.angle() + gyro_offset
    turn_to_heading(curr_abs + 90.0, tol=1.5, kp=3.2, max_sp=260)
    print("Turned 90 degrees right")

    # 计算需要直行的距离：从当前位置回到真正的物理出发点
    return_distance = calc_distance(x, y, start_x, start_y)
    print("Return distance to physical start: {:.0f} mm".format(return_distance))
    wait(1000)
    
    # 直行回到物理出发点
    print("Driving straight to start...")
    drive_straight_with_heading(
        target_deg=GYRO.angle() + gyro_offset,   # 当前朝向就是要前进的方向
        speed_mm_s=FOLLOW_SPEED_MM_S,
        stop_cond_fn=lambda: False,
        max_ms=int(max(5000, 1000 + 1000 * return_distance / FOLLOW_SPEED_MM_S))
    )

    LEFT.stop(Stop.HOLD); RIGHT.stop(Stop.HOLD)
    ev3.speaker.beep(); ev3.speaker.beep()
    print("\n=== MISSION COMPLETE ===")
    print("Final position: ({:.0f}, {:.0f}) mm".format(x, y))
    print("Distance from start: {:.0f} mm".format(calc_distance(x, y, start_x, start_y)))

# ============================ ENTRY ===============================
if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # 紧急停车
        try:
            LEFT.stop(Stop.BRAKE); RIGHT.stop(Stop.BRAKE)
        except:
            pass
        ev3.speaker.beep(); wait(150); ev3.speaker.beep()
        raise
