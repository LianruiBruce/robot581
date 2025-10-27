#!/usr/bin/env pybricks-micropython
# Team Members: [Your Names Here]
# PIDs: [Your PIDs Here]
# Team Number: [Your Team Number Here]

import math
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
# test

# Robot Geometry
WHEEL_DIAMETER_MM = 56.0
AXLE_TRACK_MM = 125.0

# Movement Parameters
DRIVE_SPEED = 180  # degrees per second
TURN_SPEED = 80   # degrees per second

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
# test branch




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
        gyro_integral = max(-30, min(30, gyro_integral))
        gyro_i = GYRO_KI * gyro_integral
        gyro_derivative = (gyro_error - gyro_last_error) / dt
        gyro_d = GYRO_KD * gyro_derivative
        gyro_last_error = gyro_error
        
        correction = gyro_p + gyro_i + gyro_d
        correction = max(-50, min(50, correction))
        
        if direction > 0:
            left_speed = speed - correction
            right_speed = speed + correction
        else:
            left_speed = speed + correction
            right_speed = speed - correction
        
        left_speed = left_speed * direction
        right_speed = right_speed * direction
        
        max_abs_speed = speed * 1.2
        left_speed = max(-max_abs_speed, min(max_abs_speed, left_speed))
        right_speed = max(-max_abs_speed, min(max_abs_speed, right_speed))
        
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
        
        turn_speed = max(-speed, min(speed, COARSE_KP * error))
        
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
        integral = max(-10, min(10, integral))
        i = FINE_KI * integral
        derivative = (error - last_error) / dt
        d = FINE_KD * derivative
        last_error = error
        
        turn_speed = p + i + d
        turn_speed = max(-speed * 0.6, min(speed * 0.6, turn_speed))
        
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
        correction = GYRO_KP * gyro_error
        correction = max(-30, min(30, correction))
        
        left_speed = speed - correction
        right_speed = speed + correction
        
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        wait(10)


def follow_wall_diagnostic(target_distance_mm=300, wall_length_mm=2400, speed=DRIVE_SPEED):
    """
    Diagnostic version - outputs detailed information
    
    Added:
    1. Print left/right wheel speeds every iteration
    2. Increased correction gain for stronger steering
    3. Option to reverse correction sign
    """
    print("="*50)
    print("DIAGNOSTIC WALL FOLLOWING")
    print("="*50)
    print("Target: " + str(target_distance_mm) + "mm")
    print("Length: " + str(wall_length_mm) + "mm")
    
    # ========== Key Parameters ==========
    TARGET_DISTANCE = target_distance_mm
    
    # Increase correction gain to make steering more obvious
    # TODO: need adjust
    CORRECTION_GAIN = 1.4
    MAX_CORRECTION = 100
    
    # Gyro assist temporarily disabled, test distance-only control first
    GYRO_ASSIST = 0.0
    
    # Reverse correction option
    REVERSE_CORRECTION = False  # If direction is wrong, change to True
    K_FAR = 50

    ALPHA = 0.35
    
    # ⚠️ If the robot still doesn’t turn properly, try:
    # 1. Increase CORRECTION_GAIN to 2.5
    # 2. Change REVERSE_CORRECTION to True
    # 3. Change min_speed below to 30
    
    print("CORRECTION_GAIN: " + str(CORRECTION_GAIN))
    print("MAX_CORRECTION: " + str(MAX_CORRECTION))
    print("GYRO_ASSIST: " + str(GYRO_ASSIST))
    print("REVERSE_CORRECTION: " + str(REVERSE_CORRECTION))
    print("="*50)
    
    # Initialization
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    parallel_gyro_reference = gyro.angle()
    last_distance = TARGET_DISTANCE
    
    iteration = 0
    continue_far = 0

    while True:
        iteration += 1
        
        # Read distance
        try:
            current_distance = ultrasonic.distance()
        except:
            current_distance = last_distance
        
        if current_distance <= 0 or current_distance > 600:
            current_distance = last_distance
        else:
            current_distance = ALPHA*current_distance + (1-ALPHA)*last_distance

        # Compute distance error
        distance_error = current_distance - TARGET_DISTANCE
        
        # Compute correction
        distance_correction = distance_error * CORRECTION_GAIN
        if distance_error > 15 and continue_far < K_FAR:
            continue_far += 1
            distance_correction = 0
        elif distance_error < 15:
            continue_far = 0
    
        # Limit correction
        distance_correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, distance_correction))
        
        # Gyro assist (currently disabled)
        current_gyro = gyro.angle()
        gyro_deviation = current_gyro - parallel_gyro_reference
        gyro_correction = gyro_deviation * GYRO_ASSIST
        
        # Total correction
        total_correction = distance_correction + gyro_correction
        
        if gyro_deviation>60 or gyro_deviation<-60:
            total_correction=0
        
        # Reverse correction if enabled
        if REVERSE_CORRECTION:
            total_correction = -total_correction
        
        # Apply to motors
        left_speed = speed - total_correction
        right_speed = speed + total_correction
        
        # Limit speed (allow larger difference)
        min_speed = 40
        max_speed = speed * 1.6
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))
        
        # Run motors
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        # ========== Detailed output ==========
        print("="*50)
        print("Iter: " + str(iteration))
        print("Distance: " + str(int(current_distance)) + "mm")
        print("Error: " + str(int(distance_error)) + "mm")
        print("Correction: " + str(int(total_correction)))
        print("Left Speed: " + str(int(left_speed)))
        print("Right Speed: " + str(int(right_speed)))
        
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
        ev3.screen.draw_text(5, 5, "D:" + str(current_distance))
        # ev3.screen.draw_text(5, 20, "Err:" + str(int(distance_error)))
        # ev3.screen.draw_text(5, 35, "Corr:" + str(int(total_correction)))
        # ev3.screen.draw_text(5, 50, "L:" + str(int(left_speed)))
        # ev3.screen.draw_text(5, 65, "R:" + str(int(right_speed)))
        ev3.screen.draw_text(5, 50, "X: "+ str(ultrasonic.distance()))
        
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
        print("Error: " + str(e))
        
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        
        ev3.speaker.beep(frequency=400, duration=300)
        wait(200)
        ev3.speaker.beep(frequency=400, duration=300)


# ============================ RUN PROGRAM =============================

if __name__ == "__main__":
    main()
