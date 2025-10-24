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

# Robot Geometry
WHEEL_DIAMETER_MM = 56.0
AXLE_TRACK_MM = 125.0

# Movement Parameters
DRIVE_SPEED = 200  # degrees per second
TURN_SPEED = 150   # degrees per second

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

# ============================ HELPER FUNCTIONS =============================

def drive_straight(distance_mm, speed=DRIVE_SPEED):
    """
    Drive straight for a specific distance.
    Positive distance = forward, negative = backward
    """
    # Calculate rotation needed
    wheel_circumference = math.pi * WHEEL_DIAMETER_MM
    rotation_degrees = (distance_mm / wheel_circumference) * 360
    
    # Reset motor angles
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    # Drive
    left_motor.run_target(speed, rotation_degrees, Stop.HOLD, wait=False)
    right_motor.run_target(speed, rotation_degrees, Stop.HOLD, wait=True)


def turn_in_place(angle_degrees, speed=TURN_SPEED):
    """
    Turn in place by specified angle.
    Positive angle = turn right, negative = turn left
    """
    # Calculate wheel rotation needed for turn
    arc_length = (AXLE_TRACK_MM * math.pi * abs(angle_degrees)) / 360
    wheel_circumference = math.pi * WHEEL_DIAMETER_MM
    wheel_rotation = (arc_length / wheel_circumference) * 360
    
    # Reset motor angles
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    if angle_degrees > 0:  # Turn right
        left_motor.run_target(speed, wheel_rotation, Stop.HOLD, wait=False)
        right_motor.run_target(speed, -wheel_rotation, Stop.HOLD, wait=True)
    else:  # Turn left
        left_motor.run_target(speed, -wheel_rotation, Stop.HOLD, wait=False)
        right_motor.run_target(speed, wheel_rotation, Stop.HOLD, wait=True)


def drive_until_collision(speed=DRIVE_SPEED):
    """
    Drive forward until either touch sensor is pressed.
    """
    left_motor.run(speed)
    right_motor.run(speed)
    
    while not touch_left.pressed() and not touch_right.pressed():
        wait(10)
    
    # Stop motors
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)


def get_distance_to_wall():
    """
    Get distance to wall from ultrasonic sensor (in mm).
    """
    return ultrasonic.distance()


def follow_wall_distance(target_distance_mm, wall_length_mm, speed=DRIVE_SPEED):
    """
    Follow the wall maintaining target distance for specified wall length.
    Uses gyro to maintain parallel orientation and distance thresholds for correction.
    Ultrasonic sensor is on the LEFT side of the robot.
    
    Strategy:
    - Use gyro to keep robot parallel to wall (gyro angle = 0)
    - Use distance thresholds to adjust when too far or too close
    - Combine both corrections for smooth wall following
    """
    # Distance thresholds
    DISTANCE_UPPER_LIMIT = target_distance_mm + 50  # 350mm - turn left toward wall
    DISTANCE_LOWER_LIMIT = target_distance_mm - 50  # 250mm - turn right away from wall
    
    # Control parameters
    DISTANCE_KP = 0.3  # Proportional gain for distance correction
    GYRO_KP = 2.0      # Proportional gain for gyro correction (keep parallel)
    
    # Reset for tracking
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    print("Starting wall following...")
    
    while True:
        # Get current distance to wall
        current_distance = get_distance_to_wall()
        
        # Get current gyro angle
        current_gyro = gyro.angle()
        
        # Display info on screen
        ev3.screen.clear()
        ev3.screen.draw_text(5, 5, "Dist: " + str(current_distance) + " mm")
        ev3.screen.draw_text(5, 25, "Target: " + str(target_distance_mm) + " mm")
        ev3.screen.draw_text(5, 45, "Gyro: " + str(current_gyro) + " deg")
        
        # Calculate wall travel distance (approximate from motor encoders)
        avg_motor_angle = (left_motor.angle() + right_motor.angle()) / 2
        wheel_circumference = math.pi * WHEEL_DIAMETER_MM
        distance_traveled = (avg_motor_angle / 360) * wheel_circumference
        
        ev3.screen.draw_text(5, 65, "Travel: " + str(int(distance_traveled)) + " mm")
        
        # Check if we've traveled 2.2m along the wall
        if distance_traveled >= wall_length_mm:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            print("Wall following complete!")
            break
        
        # Distance-based correction
        # distance > target: too far, need to turn LEFT (toward wall)
        # distance < target: too close, need to turn RIGHT (away from wall)
        distance_error = current_distance - target_distance_mm
        distance_correction = DISTANCE_KP * distance_error
        
        # Add status display
        if current_distance > DISTANCE_UPPER_LIMIT:
            ev3.screen.draw_text(5, 85, "Status: TOO FAR")
        elif current_distance < DISTANCE_LOWER_LIMIT:
            ev3.screen.draw_text(5, 85, "Status: TOO CLOSE")
        else:
            ev3.screen.draw_text(5, 85, "Status: GOOD")
        
        # Gyro-based correction (maintain parallel to wall)
        # gyro > 0: robot turned right, need to turn LEFT to correct
        # gyro < 0: robot turned left, need to turn RIGHT to correct
        gyro_error = current_gyro
        gyro_correction = GYRO_KP * gyro_error
        
        # Total correction calculation:
        # Positive value = need to turn LEFT (right motor faster, left motor slower)
        # Negative value = need to turn RIGHT (left motor faster, right motor slower)
        total_correction = gyro_correction + distance_correction
        
        # Limit total correction
        total_correction = max(-80, min(80, total_correction))
        
        # Apply correction to motor speeds
        # Turn LEFT: slow down left wheel, speed up right wheel
        # Turn RIGHT: speed up left wheel, slow down right wheel
        left_speed = speed - total_correction
        right_speed = speed + total_correction
        
        # Ensure speeds stay positive and reasonable
        left_speed = max(80, min(speed * 1.3, left_speed))
        right_speed = max(80, min(speed * 1.3, right_speed))
        
        # Apply speeds
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        
        wait(50)  # Update every 50ms
    
    # Stop motors
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)


# ============================ MAIN PROGRAM =============================

def main():
    ev3.speaker.beep()
    print("Starting Wall Following Lab")
    print("Press CENTER button to start...")
    
    # Wait for center button press
    while True:
        if Button.CENTER in ev3.buttons.pressed():
            break
        wait(10)
    
    ev3.speaker.beep()
    print("Starting program!")
    wait(500)
    
    # ============== OBJECTIVE 1: DETECT WALL ==============
    print("Objective 1: Driving to wall...")
    
    # Drive forward until collision
    drive_until_collision(speed=DRIVE_SPEED)
    ev3.speaker.beep()
    print("Wall detected!")
    
    # Back up 300mm (30cm)
    print("Backing up 30cm...")
    drive_straight(-300, speed=DRIVE_SPEED)
    wait(500)
    
    # ============== OBJECTIVE 2: TURN AT WALL ==============
    print("Objective 2: Turning right...")
    
    # Turn 90 degrees right
    turn_in_place(90, speed=TURN_SPEED)
    wait(500)
    
    # RESET GYRO after turning - this is critical!
    print("Resetting gyro sensor...")
    gyro.reset_angle(0)
    wait(200)
    
    ev3.speaker.beep()
    print("Turn complete! Gyro reset to 0.")
    print("Current gyro: " + str(gyro.angle()))
    
    # ============== OBJECTIVE 3: FOLLOW THE WALL ==============
    print("Objective 3: Following wall for 2.2m...")
    
    # Follow wall at 300mm (30cm) distance for 2200mm (2.2m)
    follow_wall_distance(target_distance_mm=300, wall_length_mm=2200, speed=DRIVE_SPEED)
    
    # Stop and beep to announce completion
    ev3.speaker.beep()
    ev3.speaker.beep()
    ev3.speaker.beep()
    print("Destination reached!")
    print("Lab complete!")


# ============================ RUN PROGRAM =============================

if __name__ == "__main__":
    main()