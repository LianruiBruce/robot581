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

WHEEL_DIAMETER = 56         # 轮直径(mm) - EV3 标准轮 56mm
AXLE_TRACK     = 114        # 轮距(mm) - 需要根据你的车宽微调

# 速度/阈值配置
CRUISE_SPEED   = 220        # 直线巡航速度 (mm/s) —— Objective 1
APPROACH_SPEED = 140        # 贴近/倒退时速度 (mm/s) —— Obj2/3
TURN_RATE      = 120        # (未用到转向，这里仅完善 DriveBase 配置)

TARGET_FRONT_MM = 400       # 题目要求：离墙 40cm（“机器人最近点”到墙）
SENSOR_TO_FRONT_OFFSET = 0  # 传感器相对“机器人最前端”的后缩量(mm)；若超声装在车头内凹位置，请测量后填入正数
DIST_TOL_MM    = 10         # 距离允许误差(抖动容忍)
POLL_MS        = 30         # 传感器轮询周期

# ===================== 初始化 =====================
ev3    = EV3Brick()
left   = Motor(PORT_LEFT)
right  = Motor(PORT_RIGHT)
touch  = TouchSensor(PORT_TOUCH)
sonar  = UltrasonicSensor(PORT_SONAR)
robot  = DriveBase(left, right, WHEEL_DIAMETER, AXLE_TRACK)
robot.settings(straight_speed=CRUISE_SPEED, turn_rate=TURN_RATE)

# ===================== 小工具函数 =====================
def _center_pressed() -> bool:
    return Button.CENTER in ev3.buttons.pressed()

def wait_center_press(prompt: str):
    """等待中心键按下（含去抖：等释放→等按下→再等释放）"""
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

def stop_at_front_distance_mm(target_front_mm: int, forward: bool):
    """
    以APPROACH_SPEED向前/后行驶，直到“机器人最前端”到墙为target_front_mm。
    若超声波与车头最近点有后缩量，请在SENSOR_TO_FRONT_OFFSET中填入正数。
    """
    target_sensor_mm = target_front_mm + SENSOR_TO_FRONT_OFFSET
    consecutive_ok, need_ok = 0, 3   # 连续满足几次再停，抑制抖动
    v = APPROACH_SPEED if forward else -APPROACH_SPEED
    robot.drive(v, 0)

    while True:
        d = sonar.distance()  # mm；个别情况下可能抖动
        # 计算是否满足“达到目标”的条件（带容忍）
        if d is None:
            ok = False
        else:
            if forward:
                ok = (d <= target_sensor_mm + DIST_TOL_MM)
            else:
                ok = (d >= target_sensor_mm - DIST_TOL_MM)

        consecutive_ok = consecutive_ok + 1 if ok else 0
        if consecutive_ok >= need_ok:
            break

        # 屏幕调试显示（可注释）
        ev3.screen.clear()
        ev3.screen.print("Dist(mm):", d if d is not None else "None")
        ev3.screen.print("Target:", target_sensor_mm)
        wait(POLL_MS)

    robot.stop(Stop.BRAKE)

def drive_until_bump():
    """向前开直到触碰传感器被按下。"""
    robot.drive(APPROACH_SPEED, 0)
    while not touch.pressed():
        wait(POLL_MS)
    robot.stop(Stop.BRAKE)

# ===================== 主流程：三个 Objective 一次完成 =====================
ev3.speaker.beep()  # 上电提示

# -------- Objective 1: 直行1.4m并停止、蜂鸣、等待按键 --------
wait_center_press("Obj1: Press CENTER to start (1.4m)")
robot.straight(1400)   # 1.4 m
ev3.speaker.beep(frequency=1000, duration=200)
wait_center_press("Obj1 done. Press CENTER for Obj2")

# -------- Objective 2: 再次前进，离墙40cm时停止并蜂鸣，等待按键 --------
stop_at_front_distance_mm(TARGET_FRONT_MM, forward=True)
ev3.speaker.beep(frequency=1200, duration=200)
wait_center_press("Obj2 done. Press CENTER for Obj3")

# -------- Objective 3: 前进触碰墙→倒车到40cm，结束 --------
drive_until_bump()  # 碰撞（bump）
ev3.speaker.beep(frequency=800, duration=150)
stop_at_front_distance_mm(TARGET_FRONT_MM, forward=False)  # 倒车回到40cm
robot.stop(Stop.BRAKE)
ev3.speaker.beep(frequency=600, duration=300)
ev3.screen.clear()
ev3.screen.print("All objectives complete.")
