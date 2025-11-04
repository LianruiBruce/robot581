# COMP 581 – Lab 3 Pre-Lab Reflection

## Team 4

### Members

- Lianrui Geng
- Xinyi Guo

---

## Task 1 – Robot Design Description

Our robot uses a **two-wheel differential drive** with a rear passive skid for stability and low friction. This enables reliable point turns and straight motion, which are essential for the obstacle approach, boundary tracing, and return-to-start phases.

To accomplish the task, we have integrated the following sensors:

- **Two Touch Sensors (Bumpers)**: Located at the front left and front right of the robot for collision detection. These sensors are used during the initial approach phase to detect when the robot makes contact with the obstacle's front wall. When either bumper is pressed, the robot stops, beeps to indicate it has found the obstacle, records the hit point, then backs away from the obstacle. The **measuring point** is defined at the front collision point of the robot.
- **One Ultrasonic Sensor**: Positioned on the left side of the robot, facing directly to the left (perpendicular to the forward direction). This sensor is used exclusively during the **left-side wall following** phase, providing continuous range data to ensure the robot's measuring point stays within the required 30 cm of the obstacle boundary at all times.
- **One Gyro Sensor**: Helps maintain the robot's heading and enables accurate 90-degree turns, ensuring precise angular control throughout the task.

Our control strategy involves using **PID distance control** based on readings from the left ultrasonic sensor to maintain a target offset of approximately 20 cm from the obstacle boundary. Steering corrections are applied by modulating the speeds of the left and right motors. We selected **left-side wall following** because after the robot finds the obstacle and performs its initial right turn, the obstacle will be on its left side, allowing for a straightforward clockwise traversal around the obstacle perimeter.

---

This design satisfies all required constraints:

- Built entirely from the LEGO Mindstorms EV3 kit
- Diameter is fixed and under 40 cm
- The center button is accessible without moving the robot
- Operates fully autonomously after the initial button press

---

## Task 2 – High-Level Flowchart

```text
┌───────────────────────┐
│   Wait for Button     │
│       Press           │
└───────────┬───────────┘
            │
            ▼
┌───────────────────────┐
│  Drive Forward Until  │
│  Bumper Pressed       │
│  (Collision Detected) │
└───────────┬───────────┘
            │  (Beep - Found Obstacle)
            ▼
┌───────────────────────┐
│ Record Hit Point Pose │
│ (At collision point) │
└───────────┬───────────┘
            │
            ▼
┌───────────────────────┐
│  Back Away from       │
│      Obstacle         │
└───────────┬───────────┘
            │
            ▼
┌───────────────────────┐
│  Right Turn ~90° via  │
│        Gyro           │
└───────────┬───────────┘
            │
            ▼
┌───────────────────────────────────┐
│  Left-side Wall Following (PID)   │
│  Keep measuring point ≤ 30 cm     │
│  from obstacle boundary            │
└───────────┬───────────────────────┘
            │ loop until back near hit point
            ▼
┌───────────────────────┐
│  Turn Away From Wall  │
└───────────┬───────────┘
            │
            ▼
┌──────────────────────────────┐
│ Return to Start (2.0m, 0.5m) │
│      Using Odom+Gyro          │
└───────────┬──────────────────┘
            │
            ▼
┌───────────────────────┐
│     Stop / Finish     │
└───────────────────────┘
```

---

## Task 3 – Pseudocode

```python
# ==================== Initialization ====================
initialize_motors()
initialize_touch_left()   # Left bumper touch sensor
initialize_touch_right()  # Right bumper touch sensor
initialize_ultrasonic_left()  # Single ultrasonic sensor on left side (for wall following)
initialize_gyro()

WAIT_FOR_BUTTON_PRESS()

# ==================== Phase 1: Drive Forward to Obstacle ====================
# Drive straight forward from starting point until collision with obstacle
while not (touch_left.pressed() or touch_right.pressed()):
    drive_forward(BASE_SPEED)

# Robot has collided with obstacle (bumper pressed)
beep()  # Indicate obstacle found
record_hit_point()  # Record current pose (at collision point)

# Back away from obstacle
drive_backward(BACKUP_DISTANCE)

# ==================== Phase 2: Turn Right and Begin Tracing ====================
turn_right_90deg_using_gyro()

# ==================== Phase 3: Wall Following Loop ====================
# Follow obstacle boundary clockwise, keeping measuring point within 30cm
# of obstacle boundary at all times
while not back_near_hit_point():
    dist = read_left_ultrasonic()  # Distance to obstacle boundary
    error = TARGET_DISTANCE - dist  # Target ~20cm, keep within 30cm
    correction = PID(error)
    
    left_motor_speed  = BASE_SPEED + correction
    right_motor_speed = BASE_SPEED - correction
    set_motor_speeds(left_motor_speed, right_motor_speed)

# ==================== Phase 4: Return to Start ====================
turn_away_from_wall()  # Turn away from obstacle
navigate_back_to_start_using_odometry()  # Return to starting point (2.0m, 0.5m)
stop_all_motors()  # Stop at start point
```
