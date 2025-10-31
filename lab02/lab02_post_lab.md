# COMP 581: Lab 2 Post-Lab Reflection
Team Members: Lianrui Geng && Xinyi Guo

---

### Task 1: Final Design and Iteration

Final Design:
Our final robot design utilized a differential drive system stabilized by a rear caster wheel. For sensing, it was equipped with two front-facing touch sensors for initial wall detection and a single, left-facing ultrasonic sensor for wall following. We also integrated a gyro sensor, which was crucial for executing precise straight-line movements (as seen in our `drive_straight_pid` function) and accurate 90-degree turns (in our `turn_in_place_pid` function).

Design Changes:
The primary change from our initial concept was the placement of the ultrasonic sensor. It was originally mounted in the middle of the chassis's left side. We moved it to the extreme front of the robot.

Reason for Change:
Early testing revealed that a mid-mounted sensor had a delayed reaction to wall curvature (especially inward bends). By moving the sensor to the front, it could detect changes in the wall's distance much sooner. This was critical for our control algorithm (`follow_wall_diagnostic`), which implemented data smoothing (`ALPHA = 0.35`) and outlier rejection (`continue_far` logic). This change ensured our controller received the most immediate data possible to make correct steering decisions.

---

### Task 2: Performance vs. Expectations

Expectations:
We expected our robot to robustly complete all phases of the task: detect the initial wall using its bumpers, back up, execute a precise 90-degree right turn, and then smoothly follow the wall using a P-controller for the required distance.

Actual Performance:
The robot's mechanical and algorithmic performance was almost entirely aligned with our expectations. Its movements were smooth, and the PID-assisted straight-line drive and gyro-assisted turns were highly accurate and stable.

Identified Shortcomings:
Our sole and most significant shortcoming was a critical misinterpretation of the lab requirements.
* The requirement stated that the robot's measuring point should "never be more than 30 cm from the wall", which we now understand as a maximum permissible distance.
* We incorrectly interpreted this as a target distance of 30 cm, setting our code's `target_distance_mm = 300`.
* Due to the nature of a P-controller (which oscillates around the setpoint) and sensor noise, our robot would occasionally and briefly exceed the 30 cm maximum. This resulted in point deductions for the "Following the wall" objective and was our only source of error.

---

### Task 3: Future Improvements

Design:
We believe our physical design, particularly the front-mounted ultrasonic sensor, is robust and well-suited for this task. No significant physical design changes would be made.

Implementation:
1.  Correct Target Distance: The most immediate and vital change would be to correct our misinterpretation of the rules. We would change the `target_distance_mm` from 300 to a safer, more conservative value (e.g., 200mm or 250mm). This would provide a sufficient buffer zone, ensuring that even with controller oscillation, the robot would not breach the 30 cm maximum boundary.
2.  Controller Enhancement (P to PID): Our `follow_wall_diagnostic` function currently implements a Proportional (P) controller (using `CORRECTION_GAIN = 1.4`). To improve performance, we would upgrade this to a full PID (Proportional-Integral-Derivative) controller.
    * An Integral (I) term would eliminate steady-state error (e.g., preventing the robot from settling at 28cm when the target is 25cm).
    * A Derivative (D) term would dampen oscillations, allowing the robot to stabilize much more quickly and smoothly after encountering curves.
3.  Refine Odometry Calibration: Our current method for tracking the 2.2m distance relies on averaging motor encoder rotations (`avg_motor_angle`). We calibrated this through extensive testing to account for wheel slip. We would refine this odometry by making the "distance per rotation" value dynamic. For instance, when the robot is turning sharply (i.e., a large difference between left and right motor speeds), wheel slip increases. Our implementation could be improved to account for this, providing a more accurate distance measurement.