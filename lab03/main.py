#!/usr/bin/env pybricks-micropython
# Gyroscope Test Program
# 陀螺仪完整测试程序
# 
# 功能：
# 1. 全面测试陀螺仪功能
# 2. 如果发现问题立即报警
# 3. 提供实时监控模式

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Button, Stop
from pybricks.tools import wait
import math

# ============================ CONFIGURATION =============================

# 硬件端口配置
GYRO_PORT = Port.S2           # 陀螺仪端口
LEFT_MOTOR_PORT = Port.B      # 左电机（用于旋转测试）
RIGHT_MOTOR_PORT = Port.C     # 右电机

# 机器人几何参数
WHEEL_DIAMETER_MM = 56.0      # 轮子直径（毫米）
AXLE_TRACK_MM = 125.0         # 轮距（两轮之间的距离，毫米）

# 测试参数
TURN_SPEED = 100              # 旋转测试速度（度/秒）
MAX_DRIFT_DEGREES = 3         # 最大允许漂移（度）
MAX_ROTATION_ERROR = 10       # 旋转测试最大误差（度）
MAX_RESET_ERROR = 2           # 重置后最大误差（度）

# ============================ INITIALIZATION =============================

ev3 = EV3Brick()
gyro = GyroSensor(GYRO_PORT)
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

# 重置陀螺仪
gyro.reset_angle(0)
wait(10)

# ============================ ALARM FUNCTIONS =============================

def sound_alarm_critical():
    """
    发出严重错误警报 - 快速高音哔哔声
    """
    for i in range(10):
        ev3.speaker.beep(frequency=1200, duration=100)
        wait(100)

def sound_alarm_warning():
    """
    发出警告警报 - 中速中音哔哔声
    """
    for i in range(5):
        ev3.speaker.beep(frequency=800, duration=200)
        wait(200)

def sound_alarm_minor():
    """
    发出轻微警报 - 低音长哔声
    """
    for i in range(3):
        ev3.speaker.beep(frequency=400, duration=500)
        wait(300)

def display_error(test_name, error_msg, alarm_level="critical"):
    """
    显示错误信息并发出警报
    
    Args:
        test_name: 测试名称
        error_msg: 错误信息
        alarm_level: 警报级别 ("critical", "warning", "minor")
    """
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "!!! GYRO ERROR !!!")
    ev3.screen.draw_text(10, 30, "Test: " + test_name)
    ev3.screen.draw_text(10, 50, error_msg[:18])  # 限制长度
    if len(error_msg) > 18:
        ev3.screen.draw_text(10, 70, error_msg[18:36])
    ev3.screen.draw_text(10, 90, "Press any button")
    
    # 根据级别发出不同警报
    if alarm_level == "critical":
        sound_alarm_critical()
    elif alarm_level == "warning":
        sound_alarm_warning()
    else:
        sound_alarm_minor()
    
    # 等待按钮按下
    while True:
        if len(ev3.buttons.pressed()) > 0:
            break
        wait(10)
    
    wait(500)  # 防止按钮连按

# ============================ ROTATION HELPER =============================

def turn_in_place_precise(angle_degrees, speed=TURN_SPEED):
    """
    精确的原地旋转函数，基于机器人轮距计算
    
    Args:
        angle_degrees: 旋转角度（正数=顺时针，负数=逆时针）
        speed: 电机速度（度/秒）
    
    Returns:
        实际旋转的角度（从陀螺仪读取）
    """
    # 计算需要的弧长
    # 机器人旋转时，每个轮子走的弧长 = (轮距 * π * 角度) / 360
    wheel_circumference = math.pi * WHEEL_DIAMETER_MM
    turn_circumference = math.pi * AXLE_TRACK_MM
    arc_length = (abs(angle_degrees) / 360.0) * turn_circumference
    
    # 转换为电机需要转的角度
    motor_rotation_degrees = (arc_length / wheel_circumference) * 360
    
    # 记录初始陀螺仪角度
    initial_gyro = gyro.angle()
    
    # 重置电机编码器
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Turning...")
    ev3.screen.draw_text(10, 30, "Target: " + str(int(angle_degrees)) + " deg")
    ev3.screen.draw_text(10, 50, "Motor: " + str(int(motor_rotation_degrees)) + " deg")
    
    # 根据方向设置电机速度
    if angle_degrees > 0:  # 顺时针（右转）
        left_motor.run(speed)
        right_motor.run(-speed)
    else:  # 逆时针（左转）
        left_motor.run(-speed)
        right_motor.run(speed)
    
    # 等待电机转到目标角度
    while True:
        avg_motor_angle = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2
        current_gyro = gyro.angle()
        
        # 实时显示进度
        ev3.screen.clear()
        ev3.screen.draw_text(10, 10, "Turning...")
        ev3.screen.draw_text(10, 30, "Motor: " + str(int(avg_motor_angle)))
        ev3.screen.draw_text(10, 50, "Gyro: " + str(int(current_gyro - initial_gyro)))
        
        if avg_motor_angle >= motor_rotation_degrees:
            break
        
        wait(10)
    
    # 停止电机
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(300)  # 等待稳定
    
    # 返回实际旋转的角度
    final_gyro = gyro.angle()
    actual_rotation = final_gyro - initial_gyro
    
    return actual_rotation

def turn_using_gyro_pid(target_angle, speed=TURN_SPEED):
    """
    使用陀螺仪PID控制的精确旋转
    这个方法直接用陀螺仪反馈来控制，更准确
    
    Args:
        target_angle: 目标旋转角度（正数=顺时针，负数=逆时针）
        speed: 最大速度
    
    Returns:
        实际旋转的角度
    """
    initial_gyro = gyro.angle()
    target_gyro = initial_gyro + target_angle
    
    # PID参数
    Kp = 2.5
    Ki = 0.02
    Kd = 0.5
    
    integral = 0
    last_error = 0
    
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "PID Turning...")
    ev3.screen.draw_text(10, 30, "Target: " + str(int(target_angle)))
    
    # 归一化角度到-180到180
    def normalize_angle(deg):
        while deg > 180:
            deg -= 360
        while deg < -180:
            deg += 360
        return deg
    
    while True:
        current_gyro = gyro.angle()
        error = normalize_angle(target_gyro - current_gyro)
        
        # 如果误差小于2度，认为到达
        if abs(error) < 2:
            break
        
        # PID计算
        p = Kp * error
        integral += error * 0.02  # dt=20ms
        integral = max(-10, min(10, integral))  # 限制积分
        i = Ki * integral
        d = Kd * (error - last_error) / 0.02
        last_error = error
        
        turn_power = p + i + d
        turn_power = max(-speed, min(speed, turn_power))
        
        # 应用到电机
        left_motor.run(turn_power)
        right_motor.run(-turn_power)
        
        # 显示进度
        if abs(error) > 10:  # 只在误差较大时更新屏幕，节省时间
            ev3.screen.clear()
            ev3.screen.draw_text(10, 10, "PID Turning...")
            ev3.screen.draw_text(10, 30, "Error: " + str(int(error)))
            ev3.screen.draw_text(10, 50, "Current: " + str(int(current_gyro - initial_gyro)))
        
        wait(20)
    
    # 停止
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(300)
    
    final_gyro = gyro.angle()
    actual_rotation = final_gyro - initial_gyro
    
    return actual_rotation

# ============================ TEST FUNCTIONS =============================

def test_gyro_basic_read():
    """
    测试1: 基本读数测试
    检查是否能正常读取陀螺仪数据
    
    Returns:
        True if passed, False otherwise
    """
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Test 1: Basic Read")
    ev3.screen.draw_text(10, 30, "Testing...")
    
    try:
        initial_angle = gyro.angle()
        ev3.screen.draw_text(10, 50, "Reading: " + str(initial_angle))
        wait(1000)
        
        # 再读几次确保稳定
        for i in range(5):
            angle = gyro.angle()
            wait(100)
        
        ev3.screen.clear()
        ev3.screen.draw_text(10, 40, "Test 1: PASS")
        ev3.speaker.beep(frequency=600, duration=200)
        wait(1500)
        return True
        
    except Exception as e:
        display_error("Basic Read", "Cannot read gyro", "critical")
        return False

def test_gyro_drift():
    """
    测试2: 漂移测试
    机器人静止5秒，检查陀螺仪读数是否稳定
    
    Returns:
        True if passed, False otherwise
    """
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Test 2: Drift Test")
    ev3.screen.draw_text(10, 30, "Keep robot STILL!")
    ev3.screen.draw_text(10, 50, "Testing in 3s...")
    wait(3000)
    
    gyro.reset_angle(0)
    wait(100)
    
    drift_readings = []
    
    # 5秒测试，每100ms读一次
    for i in range(50):
        angle = gyro.angle()
        drift_readings.append(angle)
        
        # 每秒更新一次屏幕
        if i % 10 == 0:
            ev3.screen.clear()
            ev3.screen.draw_text(10, 10, "Drift Test")
            ev3.screen.draw_text(10, 30, "Time: " + str(i // 10) + "/5 s")
            ev3.screen.draw_text(10, 50, "Angle: " + str(angle) + " deg")
            ev3.screen.draw_text(10, 70, "Stay STILL!")
        
        wait(100)
    
    # 计算漂移量
    max_drift = max(drift_readings)
    min_drift = min(drift_readings)
    total_drift = max_drift - min_drift
    
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Drift Test Result:")
    ev3.screen.draw_text(10, 30, "Max: " + str(int(max_drift)) + " deg")
    ev3.screen.draw_text(10, 50, "Min: " + str(int(min_drift)) + " deg")
    ev3.screen.draw_text(10, 70, "Total: " + str(int(total_drift)) + " deg")
    
    wait(2000)
    
    # 判断是否通过
    if total_drift > MAX_DRIFT_DEGREES:
        error_msg = "Drift: " + str(int(total_drift)) + ">" + str(MAX_DRIFT_DEGREES) + " deg"
        display_error("Drift Test", error_msg, "warning")
        return False
    else:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 40, "Test 2: PASS")
        ev3.speaker.beep(frequency=600, duration=200)
        wait(1500)
        return True

def test_gyro_rotation():
    """
    测试3: 旋转响应测试
    让机器人旋转90度，检查陀螺仪读数是否准确
    提供两种方法供选择
    
    Returns:
        True if passed, False otherwise
    """
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Test 3: Rotation")
    ev3.screen.draw_text(10, 30, "Choose method:")
    ev3.screen.draw_text(10, 45, "UP: Calculated")
    ev3.screen.draw_text(10, 60, "DOWN: Gyro PID")
    ev3.screen.draw_text(10, 75, "CENTER: Both")
    
    # 等待选择
    method = None
    while method is None:
        buttons = ev3.buttons.pressed()
        if Button.UP in buttons:
            method = "calculated"
            ev3.speaker.beep(frequency=600, duration=100)
        elif Button.DOWN in buttons:
            method = "pid"
            ev3.speaker.beep(frequency=600, duration=100)
        elif Button.CENTER in buttons:
            method = "both"
            ev3.speaker.beep(frequency=600, duration=100)
        wait(10)
    
    wait(500)
    
    results = []
    
    # 方法1: 基于计算的旋转
    if method in ["calculated", "both"]:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 10, "Method 1:")
        ev3.screen.draw_text(10, 30, "Calculated Turn")
        wait(1500)
        
        gyro.reset_angle(0)
        wait(100)
        
        actual_angle = turn_in_place_precise(90, speed=TURN_SPEED)
        error = abs(actual_angle - 90)
        
        ev3.screen.clear()
        ev3.screen.draw_text(10, 10, "Calculated Result:")
        ev3.screen.draw_text(10, 30, "Expected: 90 deg")
        ev3.screen.draw_text(10, 50, "Actual: " + str(int(actual_angle)))
        ev3.screen.draw_text(10, 70, "Error: " + str(int(error)))
        wait(3000)
        
        results.append(("Calculated", actual_angle, error))
    
    # 方法2: 基于陀螺仪PID的旋转
    if method in ["pid", "both"]:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 10, "Method 2:")
        ev3.screen.draw_text(10, 30, "Gyro PID Turn")
        wait(1500)
        
        gyro.reset_angle(0)
        wait(100)
        
        actual_angle = turn_using_gyro_pid(90, speed=TURN_SPEED)
        error = abs(actual_angle - 90)
        
        ev3.screen.clear()
        ev3.screen.draw_text(10, 10, "PID Result:")
        ev3.screen.draw_text(10, 30, "Expected: 90 deg")
        ev3.screen.draw_text(10, 50, "Actual: " + str(int(actual_angle)))
        ev3.screen.draw_text(10, 70, "Error: " + str(int(error)))
        wait(3000)
        
        results.append(("PID", actual_angle, error))
    
    # 显示综合结果
    ev3.screen.clear()
    ev3.screen.draw_text(10, 5, "Rotation Summary:")
    y = 25
    all_passed = True
    
    for name, angle, error in results:
        status = "PASS" if error <= MAX_ROTATION_ERROR else "FAIL"
        ev3.screen.draw_text(10, y, name + ": " + str(int(error)) + "d " + status)
        y += 20
        if error > MAX_ROTATION_ERROR:
            all_passed = False
    
    wait(3000)
    
    # 判断是否通过
    if not all_passed:
        worst_error = max(r[2] for r in results)
        error_msg = "Max error: " + str(int(worst_error)) + ">" + str(MAX_ROTATION_ERROR)
        display_error("Rotation Test", error_msg, "warning")
        return False
    else:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 40, "Test 3: PASS")
        ev3.speaker.beep(frequency=600, duration=200)
        wait(1500)
        return True

def test_gyro_reset():
    """
    测试4: 重置功能测试
    检查reset_angle()是否正常工作
    
    Returns:
        True if passed, False otherwise
    """
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Test 4: Reset")
    ev3.screen.draw_text(10, 30, "Testing...")
    
    # 先读一个非零的角度
    current_angle = gyro.angle()
    ev3.screen.draw_text(10, 50, "Before: " + str(int(current_angle)))
    wait(1000)
    
    # 重置
    gyro.reset_angle(0)
    wait(200)
    
    # 检查重置后的值
    reset_angle = gyro.angle()
    
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Reset Test Result:")
    ev3.screen.draw_text(10, 30, "After reset: " + str(reset_angle))
    
    wait(2000)
    
    # 判断是否通过
    if abs(reset_angle) > MAX_RESET_ERROR:
        error_msg = "Reset to " + str(reset_angle) + " not 0"
        display_error("Reset Test", error_msg, "minor")
        return False
    else:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 40, "Test 4: PASS")
        ev3.speaker.beep(frequency=600, duration=200)
        wait(1500)
        return True

def test_gyro_continuous_monitoring():
    """
    持续监控模式
    实时显示陀螺仪读数，按CENTER退出
    用于手动检查和长时间观察
    """
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Continuous Monitor")
    ev3.screen.draw_text(10, 30, "Rotate robot")
    ev3.screen.draw_text(10, 50, "to test response")
    ev3.screen.draw_text(10, 70, "CENTER to exit")
    wait(2000)
    
    gyro.reset_angle(0)
    wait(100)
    
    last_angle = 0
    max_change_per_cycle = 0
    reading_count = 0
    anomaly_count = 0
    
    while True:
        # 检查退出按钮
        if Button.CENTER in ev3.buttons.pressed():
            break
        
        current_angle = gyro.angle()
        change = abs(current_angle - last_angle)
        reading_count += 1
        
        # 记录最大变化
        if change > max_change_per_cycle:
            max_change_per_cycle = change
        
        # 检测异常跳变（单次变化>100度可能有问题）
        if change > 100:
            anomaly_count += 1
            ev3.speaker.beep(frequency=1500, duration=50)
        
        # 更新屏幕
        ev3.screen.clear()
        ev3.screen.draw_text(10, 10, "Monitor Mode")
        ev3.screen.draw_text(10, 25, "Angle: " + str(int(current_angle)))
        ev3.screen.draw_text(10, 40, "Change: " + str(int(change)))
        ev3.screen.draw_text(10, 55, "Max: " + str(int(max_change_per_cycle)))
        ev3.screen.draw_text(10, 70, "Anomaly: " + str(anomaly_count))
        ev3.screen.draw_text(10, 85, "Readings: " + str(reading_count))
        ev3.screen.draw_text(10, 100, "CENTER=exit")
        
        last_angle = current_angle
        wait(100)
    
    # 显示统计
    ev3.screen.clear()
    ev3.screen.draw_text(10, 10, "Monitor Summary:")
    ev3.screen.draw_text(10, 30, "Readings: " + str(reading_count))
    ev3.screen.draw_text(10, 50, "Anomalies: " + str(anomaly_count))
    ev3.screen.draw_text(10, 70, "Max change: " + str(int(max_change_per_cycle)))
    
    if anomaly_count > 5:
        ev3.screen.draw_text(10, 90, "WARNING: Unstable!")
        sound_alarm_warning()
    else:
        ev3.screen.draw_text(10, 90, "Looks good!")
        ev3.speaker.beep(frequency=600, duration=200)
    
    wait(3000)

def run_all_tests():
    """
    运行所有测试并生成报告
    
    Returns:
        True if all tests passed, False otherwise
    """
    test_results = []
    test_names = []
    
    # 测试1: 基本读数
    test_names.append("Basic Read")
    test_results.append(test_gyro_basic_read())
    
    # 测试2: 漂移
    test_names.append("Drift Test")
    test_results.append(test_gyro_drift())
    
    # 测试3: 旋转
    test_names.append("Rotation")
    test_results.append(test_gyro_rotation())
    
    # 测试4: 重置
    test_names.append("Reset")
    test_results.append(test_gyro_reset())
    
    # 生成报告
    ev3.screen.clear()
    ev3.screen.draw_text(10, 5, "=== TEST REPORT ===")
    
    y_pos = 25
    for i in range(len(test_names)):
        status = "PASS" if test_results[i] else "FAIL"
        ev3.screen.draw_text(10, y_pos, test_names[i][:10] + ": " + status)
        y_pos += 15
    
    # 统计
    passed = sum(test_results)
    total = len(test_results)
    
    ev3.screen.draw_text(10, y_pos + 10, "Result: " + str(passed) + "/" + str(total))
    
    wait(2000)
    
    # 最终判断
    if all(test_results):
        ev3.screen.clear()
        ev3.screen.draw_text(10, 30, "ALL TESTS PASSED!")
        ev3.screen.draw_text(10, 50, "Gyro is healthy")
        
        # 播放成功音乐
        for i in range(4):
            ev3.speaker.beep(frequency=600 + i*100, duration=150)
            wait(100)
        
        wait(3000)
        return True
    else:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 20, "TESTS FAILED!")
        ev3.screen.draw_text(10, 40, "Gyro has issues")
        ev3.screen.draw_text(10, 60, str(total - passed) + " test(s) failed")
        ev3.screen.draw_text(10, 80, "Check sensor!")
        
        # 发出严重警报
        sound_alarm_critical()
        
        wait(3000)
        return False

# ============================ MAIN PROGRAM =============================

def main():
    """
    主程序 - 陀螺仪测试菜单
    """
    try:
        # 欢迎界面
        ev3.screen.clear()
        ev3.screen.draw_text(10, 20, "GYRO TEST PROGRAM")
        ev3.screen.draw_text(10, 40, "==================")
        ev3.speaker.beep(frequency=800, duration=200)
        wait(2000)
        
        while True:
            # 显示菜单
            ev3.screen.clear()
            ev3.screen.draw_text(10, 5, "Select Test:")
            ev3.screen.draw_text(10, 25, "UP: All Tests")
            ev3.screen.draw_text(10, 40, "CENTER: Monitor")
            ev3.screen.draw_text(10, 55, "DOWN: Individual")
            ev3.screen.draw_text(10, 70, "LEFT: Basic Read")
            ev3.screen.draw_text(10, 85, "RIGHT: Exit")
            
            # 等待按钮
            while True:
                buttons = ev3.buttons.pressed()
                
                if Button.UP in buttons:
                    # 运行所有测试
                    ev3.speaker.beep(frequency=600, duration=100)
                    wait(300)
                    run_all_tests()
                    break
                
                elif Button.CENTER in buttons:
                    # 连续监控模式
                    ev3.speaker.beep(frequency=600, duration=100)
                    wait(300)
                    test_gyro_continuous_monitoring()
                    break
                
                elif Button.DOWN in buttons:
                    # 单独测试菜单
                    ev3.speaker.beep(frequency=600, duration=100)
                    wait(300)
                    individual_test_menu()
                    break
                
                elif Button.LEFT in buttons:
                    # 快速基本读数测试
                    ev3.speaker.beep(frequency=600, duration=100)
                    wait(300)
                    test_gyro_basic_read()
                    break
                
                elif Button.RIGHT in buttons:
                    # 退出程序
                    ev3.screen.clear()
                    ev3.screen.draw_text(10, 40, "Exiting...")
                    ev3.speaker.beep(frequency=400, duration=200)
                    wait(1000)
                    return
                
                wait(10)
            
            wait(500)  # 防止按钮连按
    
    except Exception as e:
        # 异常处理
        ev3.screen.clear()
        ev3.screen.draw_text(10, 20, "PROGRAM ERROR!")
        ev3.screen.draw_text(10, 40, "Exception caught")
        
        sound_alarm_critical()
        
        wait(3000)

def individual_test_menu():
    """
    单独测试菜单
    """
    while True:
        ev3.screen.clear()
        ev3.screen.draw_text(10, 5, "Individual Tests:")
        ev3.screen.draw_text(10, 25, "UP: Basic Read")
        ev3.screen.draw_text(10, 40, "CENTER: Drift")
        ev3.screen.draw_text(10, 55, "DOWN: Rotation")
        ev3.screen.draw_text(10, 70, "LEFT: Reset")
        ev3.screen.draw_text(10, 85, "RIGHT: Back")
        
        while True:
            buttons = ev3.buttons.pressed()
            
            if Button.UP in buttons:
                ev3.speaker.beep(frequency=600, duration=100)
                wait(300)
                test_gyro_basic_read()
                break
            
            elif Button.CENTER in buttons:
                ev3.speaker.beep(frequency=600, duration=100)
                wait(300)
                test_gyro_drift()
                break
            
            elif Button.DOWN in buttons:
                ev3.speaker.beep(frequency=600, duration=100)
                wait(300)
                test_gyro_rotation()
                break
            
            elif Button.LEFT in buttons:
                ev3.speaker.beep(frequency=600, duration=100)
                wait(300)
                test_gyro_reset()
                break
            
            elif Button.RIGHT in buttons:
                ev3.speaker.beep(frequency=600, duration=100)
                wait(300)
                return
            
            wait(10)
        
        wait(500)

# ============================ RUN PROGRAM =============================

if __name__ == "__main__":
    main()