# #!/usr/bin/env pybricks-micropython
# # EV3 MicroPython (Pybricks)

# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
# from pybricks.parameters import Port, Stop, Button
# from pybricks.robotics import DriveBase
# from pybricks.tools import wait

# # ===================== 端口与底盘参数（按需修改） =====================
# PORT_LEFT   = Port.B        # 左轮电机端口
# PORT_RIGHT  = Port.C        # 右轮电机端口
# PORT_TOUCH  = Port.S1       # 触碰（bump）传感器端口
# PORT_SONAR  = Port.S4       # 超声波（距离）传感器端口

# WHEEL_DIAMETER = 56         # 轮直径(mm) - EV3 标准轮 56mm
# AXLE_TRACK     = 114        # 轮距(mm) - 需要根据你的车宽微调

# # 速度/阈值配置
# CRUISE_SPEED   = 220        # 直线巡航速度 (mm/s) —— Objective 1
# APPROACH_SPEED = 140        # 贴近/倒退时速度 (mm/s) —— Obj2/3
# TURN_RATE      = 120        # (未用到转向，这里仅完善 DriveBase 配置)

# SENSOR_TO_FRONT_OFFSET = 30
# TARGET_FRONT_MM = 400 + SENSOR_TO_FRONT_OFFSET      # 题目要求：离墙 40cm（“机器人最近点”到墙）,需要加上碰撞传感器的长度。
# SENSOR_TO_FRONT_OFFSET = 0  # 传感器相对“机器人最前端”的后缩量(mm)；若超声装在车头内凹位置，请测量后填入正数
# DIST_TOL_MM    = 10         # 距离允许误差(抖动容忍)
# POLL_MS        = 30         # 传感器轮询周期

# # ===================== 初始化 =====================
# ev3    = EV3Brick()
# left   = Motor(PORT_LEFT)
# right  = Motor(PORT_RIGHT)
# touch  = TouchSensor(PORT_TOUCH)
# sonar  = UltrasonicSensor(PORT_SONAR)
# robot  = DriveBase(left, right, WHEEL_DIAMETER, AXLE_TRACK)
# robot.settings(straight_speed=CRUISE_SPEED, turn_rate=TURN_RATE)

# # ===================== 小工具函数 =====================
# def _center_pressed() -> bool:
#     return Button.CENTER in ev3.buttons.pressed()

# def wait_center_press(prompt: str):
#     """等待中心键按下（含去抖：等释放→等按下→再等释放）"""
#     ev3.screen.clear()
#     ev3.screen.print(prompt)
#     # 等待释放
#     while _center_pressed():
#         wait(10)
#     # 等待按下
#     while not _center_pressed():
#         wait(10)
#     # 去抖：等释放
#     while _center_pressed():
#         wait(10)
#     ev3.speaker.beep()

# def stop_at_front_distance_mm(target_front_mm: int, forward: bool):
#     """
#     以APPROACH_SPEED向前/后行驶，直到“机器人最前端”到墙为target_front_mm。
#     若超声波与车头最近点有后缩量，请在SENSOR_TO_FRONT_OFFSET中填入正数。
#     """
#     target_sensor_mm = target_front_mm + SENSOR_TO_FRONT_OFFSET
#     consecutive_ok, need_ok = 0, 3   # 连续满足几次再停，抑制抖动
#     v = APPROACH_SPEED if forward else -APPROACH_SPEED
#     robot.drive(v, 0)

#     while True:
#         d = sonar.distance()  # mm；个别情况下可能抖动
#         # 计算是否满足“达到目标”的条件（带容忍）
#         if d is None:
#             ok = False
#         else:
#             if forward:
#                 ok = (d <= target_sensor_mm + DIST_TOL_MM)
#             else:
#                 ok = (d >= target_sensor_mm - DIST_TOL_MM)

#         consecutive_ok = consecutive_ok + 1 if ok else 0
#         if consecutive_ok >= need_ok:
#             break

#         # 屏幕调试显示（可注释）
#         ev3.screen.clear()
#         ev3.screen.print("Dist(mm):", d if d is not None else "None")
#         ev3.screen.print("Target:", target_sensor_mm)
#         wait(POLL_MS)

#     robot.stop(Stop.BRAKE)

# def drive_until_bump():
#     """向前开直到触碰传感器被按下。"""
#     robot.drive(APPROACH_SPEED, 0)
#     while not touch.pressed():
#         wait(POLL_MS)
#     robot.stop(Stop.BRAKE)

# # ===================== 主流程：三个 Objective 一次完成 =====================
# ev3.speaker.beep()  # 上电提示

# # -------- Objective 1: 直行1.4m并停止、蜂鸣、等待按键 --------
# wait_center_press("Obj1:press to go 1.4m")
# robot.straight(1400)   # 1.4 m
# ev3.speaker.beep(frequency=1000, duration=200)
# wait_center_press("Obj1 done. Press CENTER for Obj2")

# # -------- Objective 2: 再次前进，离墙40cm时停止并蜂鸣，等待按键 --------
# stop_at_front_distance_mm(TARGET_FRONT_MM, forward=True)
# ev3.speaker.beep(frequency=1200, duration=200)
# wait_center_press("Obj2 done. Press CENTER for Obj3")

# # -------- Objective 3: 前进触碰墙→倒车到40cm，结束 --------
# drive_until_bump()  # 碰撞（bump）
# ev3.speaker.beep(frequency=800, duration=150)
# stop_at_front_distance_mm(TARGET_FRONT_MM, forward=False)  # 倒车回到40cm
# robot.stop(Stop.BRAKE)
# ev3.speaker.beep(frequency=600, duration=300)
# ev3.screen.clear()
# ev3.screen.print("All objectives complete.")


# v 2.0
#!/usr/bin/env pybricks-micropython
# EV3 MicroPython (Pybricks)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# ===================== 端口与底盘参数（按需修改） =====================
PORT_LEFT   = Port.B        # 左轮电机端口
PORT_RIGHT  = Port.C        # 右轮电机端口
PORT_TOUCH  = Port.S1       # 触碰（bump）传感器端口
PORT_SONAR  = Port.S4       # 超声波（距离）传感器端口

WHEEL_DIAMETER = 56         # 轮直径(mm) - EV3 标准 56mm
AXLE_TRACK     = 114        # 轮距(mm)

# ===================== 速度 / 阈值配置（场地可微调） =====================
CRUISE_SPEED     = 200      # 常速直行 (mm/s)
APPROACH_SPEED   = 120      # 贴近/倒退 (mm/s)
SLOWDOWN_SPEED   = 80       # 临近目标时的慢速 (mm/s)
TURN_RATE        = 120      # 仅为 DriveBase 配置完整性

TARGET_FRONT_MM  = 400      # 目标：车头最近点离墙 40 cm
DIST_TOL_MM      = 10       # 目标距离容差
POLL_MS          = 30       # 轮询周期(ms)

# 传感器&安全相关
DEFAULT_NOSE_OFFSET = 30    # “鼻子”(碰撞头)前伸量，默认 30mm（= 3cm）
AUTO_CALIBRATE_OFFSET = False   # 如需自动标定鼻子前伸量，改为 True
SAFETY_MARGIN_FRONT  = 15    # 除鼻子外的额外安全边际 (mm)
MIN_SAFE_GAP_SENSOR  = DEFAULT_NOSE_OFFSET + SAFETY_MARGIN_FRONT  # 传感器读数小于该值则视为危险
MAX_VALID_DISTANCE   = 2500   # 超声有效上限(mm)；无回波等异常将被忽略
SMOOTH_WINDOW        = 3      # 超声滑动窗口(去抖)

# 行进距离修正（若你觉得里程有轻微偏差，可微调此倍率）
DIST_SCALE = 1.00            # 实际=命令×DIST_SCALE（>1走更远，<1走更近）

# ===================== 初始化 =====================
ev3    = EV3Brick()
left   = Motor(PORT_LEFT)
right  = Motor(PORT_RIGHT)
touch  = TouchSensor(PORT_TOUCH)
sonar  = UltrasonicSensor(PORT_SONAR)
robot  = DriveBase(left, right, WHEEL_DIAMETER, AXLE_TRACK)
robot.settings(straight_speed=CRUISE_SPEED, turn_rate=TURN_RATE)

# =============== 工具：按钮/提示 ===============
def _center_pressed():
    return Button.CENTER in ev3.buttons.pressed()

def wait_center_press(prompt):
    ev3.screen.clear()
    ev3.screen.print(prompt)
    # 等待释放
    while _center_pressed():
        wait(10)
    # 等待按下
    while not _center_pressed():
        wait(10)
    # 去抖：等释放
    while _center_pressed():
        wait(10)
    ev3.speaker.beep()

def note(msg):
    ev3.screen.clear()
    ev3.screen.print(msg)

# =============== 工具：超声平滑读取 ===============
# 维护一个小窗口，返回中位数，遇到 None/异常用上次有效值兜底
_last_valid = None
_us_win = []

def _push_window(val):
    global _us_win
    _us_win.append(val)
    if len(_us_win) > SMOOTH_WINDOW:
        _us_win.pop(0)

def _median(lst):
    n = len(lst)
    if n == 0:
        return None
    s = sorted(lst)
    m = n // 2
    if n % 2:
        return s[m]
    else:
        return (s[m-1] + s[m]) // 2

def read_distance_mm():
    """返回平滑后的超声(mm)，若无效则返回上次有效值。"""
    global _last_valid
    raw = sonar.distance()
    if raw is not None and 0 < raw < MAX_VALID_DISTANCE:
        _push_window(raw)
        med = _median(_us_win)
        _last_valid = med if med is not None else raw
    # 若本次无效，使用上次有效值（可能为 None）
    return _last_valid

# =============== 工具：鼻子前伸量标定（可选） ===============
def calibrate_sensor_offset():
    """
    慢速前进触碰墙，读取超声，作为 '鼻子前伸量'(offset)。
    读取值若不合理，退回默认 30mm。随后后退一点。
    """
    DEFAULT = DEFAULT_NOSE_OFFSET
    note("Calibrating nose...")
    robot.drive(60, 0)
    last = None
    while not touch.pressed():
        d = read_distance_mm()
        if d is not None:
            last = d
        wait(POLL_MS)
    robot.stop(Stop.BRAKE)
    ev3.speaker.beep(frequency=700, duration=100)

    d = read_distance_mm() or last
    if d is None or d < 5 or d > 150:
        d = DEFAULT

    # 轻微后退，避免继续贴墙
    robot.straight(-60)
    note("Nose offset: {}mm".format(int(d)))
    wait(500)
    return int(d)

# =============== 安全监控与处理 ===============
def is_bumped():
    return touch.pressed()

def too_close(sensor_to_front_offset):
    """若传感器读数过小（小于鼻子+安全边际）则判为危险。"""
    d = read_distance_mm()
    if d is None:
        return False
    return d < (sensor_to_front_offset + SAFETY_MARGIN_FRONT)

def handle_safety(sensor_to_front_offset):
    """
    统一安全处理：急停->蜂鸣->自动退让到安全距离->等待用户确认继续。
    """
    robot.stop(Stop.BRAKE)
    ev3.speaker.beep(frequency=400, duration=200)
    note("SAFETY: stopping")

    # 若已顶到墙或距离过近，则后退至安全值
    # 以超声目标 = 鼻子 + 2*安全边际 作缓冲
    target_sensor = sensor_to_front_offset + 2 * SAFETY_MARGIN_FRONT
    robot.drive(-APPROACH_SPEED, 0)
    safe_cnt = 0
    while True:
        d = read_distance_mm()
        if d is not None and d >= target_sensor:
            safe_cnt += 1
        else:
            safe_cnt = 0
        if safe_cnt >= 3:
            break
        wait(POLL_MS)
    robot.stop(Stop.BRAKE)
    ev3.speaker.beep(frequency=520, duration=120)

    # 等用户检查环境后继续
    wait_center_press("Obstacle handled. Press CENTER to resume")

# =============== 行进：带安全监控的“可续跑”直行 ===============
def drive_distance_with_safety(distance_mm, sensor_to_front_offset):
    """
    以 CRUISE_SPEED 驾驶到指定里程，期间若触发安全（碰撞/过近），
    则执行 handle_safety() 后自动续跑剩余距离。
    """
    target = distance_mm * DIST_SCALE
    robot.reset()
    robot.drive(CRUISE_SPEED, 0)

    while True:
        # 安全检查
        if is_bumped() or too_close(sensor_to_front_offset):
            handle_safety(sensor_to_front_offset)
            # 重新开始驱动，续跑剩余路程
            remaining = target - robot.distance()
            if remaining <= 0:
                break
            robot.drive(CRUISE_SPEED, 0)

        # 完成判定
        if robot.distance() >= target:
            break

        # 临近目标时减速，减少“温差”造成的过冲
        remain = target - robot.distance()
        if remain < 200 and remain > 0:
            robot.drive(SLOWDOWN_SPEED, 0)

        # 调试显示（可注释）
        d = read_distance_mm()
        ev3.screen.clear()
        ev3.screen.print("Dist:", int(robot.distance()), "/", int(target))
        ev3.screen.print("US:", d if d is not None else "None")
        wait(POLL_MS)

    robot.stop(Stop.BRAKE)

# =============== 贴近/回退到“车头距墙=目标”的控制 ===============
def stop_at_front_distance_mm(target_front_mm, forward, sensor_to_front_offset):
    """
    以 APPROACH_SPEED 前进/后退，使“车头最近点到墙”的真实距离 ≈ target_front_mm。
    自动考虑鼻子前伸量。靠近时带抖动抑制，随时安全监控。
    """
    target_sensor = target_front_mm + sensor_to_front_offset
    v = APPROACH_SPEED if forward else -APPROACH_SPEED
    robot.drive(v, 0)

    consecutive_ok, need_ok = 0, 4
    while True:
        # 安全优先
        if is_bumped() or too_close(sensor_to_front_offset):
            handle_safety(sensor_to_front_offset)
            # 恢复朝向
            robot.drive(v, 0)

        d = read_distance_mm()
        # 无测距时仅维持慢速并等待下一次有效读数
        if d is None:
            wait(POLL_MS)
            continue

        # 临近目标减速
        if abs(d - target_sensor) < 80:
            robot.drive(SLOWDOWN_SPEED if forward else -SLOWDOWN_SPEED, 0)

        # 目标判定（带容差）
        if forward:
            ok = (d <= target_sensor + DIST_TOL_MM)
        else:
            ok = (d >= target_sensor - DIST_TOL_MM)

        consecutive_ok = consecutive_ok + 1 if ok else 0
        if consecutive_ok >= need_ok:
            break

        # 调试屏显（可注释）
        ev3.screen.clear()
        ev3.screen.print("US:", d, "->", target_sensor)
        wait(POLL_MS)

    robot.stop(Stop.BRAKE)

# =============== 前进行至“触碰”为止（带安全监控） ===============
def drive_until_bump(sensor_to_front_offset):
    """
    前进直到触碰传感器触发。期间仍对“超近”做保护（先停下&退避，而不是硬撞）。
    """
    robot.drive(APPROACH_SPEED, 0)
    while True:
        if is_bumped():
            break
        if too_close(sensor_to_front_offset):
            # 还没真正触发，但距离过近：当作危险处理
            handle_safety(sensor_to_front_offset)
            # 继续尝试靠近触碰（若确实需要“顶墙”）
            robot.drive(APPROACH_SPEED, 0)
        wait(POLL_MS)
    robot.stop(Stop.BRAKE)

# ===================== 主流程 =====================
def main():
    ev3.speaker.beep()
    note("Init...")

    # 计算鼻子前伸量（自动或固定值）
    if AUTO_CALIBRATE_OFFSET:
        sensor_to_front_offset = calibrate_sensor_offset()
    else:
        sensor_to_front_offset = DEFAULT_NOSE_OFFSET
        note("Nose offset={}mm".format(sensor_to_front_offset))
        wait(500)

    # -------- Objective 1: 直行 1.4m（可续跑&避碰） --------
    wait_center_press("Obj1: press CENTER to go 1.4m")
    drive_distance_with_safety(1400, sensor_to_front_offset)
    ev3.speaker.beep(frequency=1000, duration=200)
    wait_center_press("Obj1 done. CENTER for Obj2")

    # -------- Objective 2: 前进并在车头距墙 40cm 停止 --------
    note("Approach to 40cm")
    stop_at_front_distance_mm(TARGET_FRONT_MM, forward=True,
                              sensor_to_front_offset=sensor_to_front_offset)
    ev3.speaker.beep(frequency=1200, duration=200)
    wait_center_press("Obj2 done. CENTER for Obj3")

    # -------- Objective 3: 顶墙->后退到 40cm --------
    note("Drive until bump")
    drive_until_bump(sensor_to_front_offset)
    ev3.speaker.beep(frequency=800, duration=150)
    note("Back to 40cm")
    stop_at_front_distance_mm(TARGET_FRONT_MM, forward=False,
                              sensor_to_front_offset=sensor_to_front_offset)
    robot.stop(Stop.BRAKE)
    ev3.speaker.beep(frequency=600, duration=300)
    note("All objectives complete.")

# 入口
if __name__ == "__main__":
    main()
