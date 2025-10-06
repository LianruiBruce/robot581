#!/usr/bin/env pybricks-micropython
# EV3 MicroPython (Pybricks)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait, StopWatch

# ===================================================================
#                          CONFIGURATION
# ===================================================================

# --- Port Assignments ---
PORT_MOTOR_LEFT = Port.B
PORT_MOTOR_RIGHT = Port.C
PORT_TOUCH_SENSOR = Port.S1
PORT_ULTRASONIC_SENSOR = Port.S4

# --- Robot Physical Parameters (ADJUST AS NEEDED) ---
WHEEL_DIAMETER_MM = 56.0
AXLE_TRACK_MM = 114.0 # For reference, not used in this code
PI = 3.14159
WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * PI

# The distance the sensor is set back from the robot's front bumper (mm).
SENSOR_OFFSET_FROM_BUMPER_MM = 30

# --- Behavior and Speed Settings ---
# Note: Motor speed is in degrees per second (deg/s)
DRIVE_SPEED_MM_S = 200
DRIVE_SPEED_DEG_S = (DRIVE_SPEED_MM_S / WHEEL_CIRCUMFERENCE_MM) * 360

APPROACH_SPEED_MM_S = 140
APPROACH_SPEED_DEG_S = (APPROACH_SPEED_MM_S / WHEEL_CIRCUMFERENCE_MM) * 360

# --- Objective & Sensor Parameters ---
DISTANCE_OBJ1_MM = 1400
BUMPER_TARGET_DISTANCE_MM = 400
DISTANCE_TOLERANCE_MM = 10
SENSOR_POLL_INTERVAL_MS = 30


# ===================================================================
#                        INITIALIZATION
# ===================================================================

ev3 = EV3Brick()
left_motor = Motor(PORT_MOTOR_LEFT)
right_motor = Motor(PORT_MOTOR_RIGHT)
touch_sensor = TouchSensor(PORT_TOUCH_SENSOR)
ultrasonic_sensor = UltrasonicSensor(PORT_ULTRASONIC_SENSOR)


# ===================================================================
#                  CUSTOM MOVEMENT FUNCTIONS
# ===================================================================

def move_straight_distance(speed_deg_s: float, distance_mm: float):
    """
    Drives the robot straight for a specific distance using direct motor control.
    """
    # Calculate the required rotation angle for the wheels.
    rotation_angle = (distance_mm / WHEEL_CIRCUMFERENCE_MM) * 360

    # Reset motor angles before starting.
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    # Run both motors to the target angle.
    # 'wait=False' on the first motor allows both to start simultaneously.
    left_motor.run_target(speed_deg_s, rotation_angle, then=Stop.BRAKE, wait=False)
    right_motor.run_target(speed_deg_s, rotation_angle, then=Stop.BRAKE, wait=True)

def start_moving(speed_deg_s: float):
    """
    Starts the robot moving forward or backward indefinitely.
    Positive speed moves forward, negative speed moves backward.
    """
    left_motor.run(speed_deg_s)
    right_motor.run(speed_deg_s)

def stop_robot():
    """Stops both motors with a brake."""
    left_motor.brake()
    right_motor.brake()


# ===================================================================
#                        HELPER FUNCTIONS
# ===================================================================

def wait_for_center_button(prompt: str):
    """
    Displays a prompt and waits for the center button to be pressed.
    The action will trigger the moment the button is pressed down.
    """
    ev3.screen.clear()
    ev3.screen.print(prompt)
    # Wait until the button is pressed.
    while not Button.CENTER in ev3.buttons.pressed():
        wait(10)
    ev3.speaker.beep()
    # A small delay to avoid the same press triggering the next action if the
    # user holds the button for a fraction of a second too long.
    wait(200)

# ===================================================================
#                           MAIN PROGRAM
# ===================================================================

def main():
    """Runs the main sequence of objectives for the robot challenge."""
    ev3.speaker.beep() # Signal that the program has started.

    # --- Objective 1: Drive straight for 1.4m, stop, and wait. ---
    wait_for_center_button("Obj1: Press to\nstart 1.4m run.")
    move_straight_distance(DRIVE_SPEED_DEG_S, DISTANCE_OBJ1_MM)
    ev3.speaker.beep(frequency=1000, duration=200)

    # --- Objective 2: Move forward until 40cm from the wall, stop, and wait. ---
    wait_for_center_button("Obj1 Done.\nPress for Obj2.")
    
    target_sensor_dist_mm = BUMPER_TARGET_DISTANCE_MM + SENSOR_OFFSET_FROM_BUMPER_MM
    consecutive_hits = 0
    required_hits = 3

    start_moving(APPROACH_SPEED_DEG_S)

    while consecutive_hits < required_hits:
        current_distance = ultrasonic_sensor.distance()
        
        if current_distance is not None and (current_distance <= target_sensor_dist_mm + DISTANCE_TOLERANCE_MM):
            consecutive_hits += 1
        else:
            consecutive_hits = 0 # Reset counter if reading is invalid or out of range.
        
        wait(SENSOR_POLL_INTERVAL_MS)
    
    stop_robot()
    ev3.speaker.beep(frequency=1200, duration=200)

    # --- Objective 3: Touch the wall, then reverse to 40cm and stop. ---
    wait_for_center_button("Obj2 Done.\nPress for Obj3.")
    
    # Part A: Move forward until bump
    start_moving(APPROACH_SPEED_DEG_S)
    while not touch_sensor.pressed():
        wait(SENSOR_POLL_INTERVAL_MS)
    stop_robot()
    ev3.speaker.beep(frequency=800, duration=150)
    
    # Part B: Reverse until 40cm from wall
    consecutive_hits = 0 # Reset hit counter
    start_moving(-APPROACH_SPEED_DEG_S) # Move in reverse

    while consecutive_hits < required_hits:
        current_distance = ultrasonic_sensor.distance()

        if current_distance is not None and (current_distance >= target_sensor_dist_mm - DISTANCE_TOLERANCE_MM):
            consecutive_hits += 1
        else:
            consecutive_hits = 0

        wait(SENSOR_POLL_INTERVAL_MS)
    
    stop_robot()
    ev3.speaker.beep(frequency=600, duration=300)

    # --- End of Program ---
    ev3.screen.clear()
    ev3.screen.print("All objectives\ncomplete.")
    wait(5000)

if __name__ == "__main__":
    main()