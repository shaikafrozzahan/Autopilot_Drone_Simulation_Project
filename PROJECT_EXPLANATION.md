# Detailed project explanation

## 1. Project goal
This project demonstrates the logic behind an autonomous drone in a **2D computer simulation**. A virtual vehicle follows a sequence of waypoints while a controller adjusts its velocity and a simple local rule steers away from circular keep-out zones.

It is intentionally not connected to real flight hardware. Real drone flight requires validated flight-control software, calibrated sensors, robust state estimation, failsafes, and controlled flight testing.

## 2. System blocks
1. **Mission planner:** `WAYPOINTS` defines the desired route.
2. **Position error:** the controller subtracts the simulated position from the active target.
3. **PID-style controller:** converts position error into a desired velocity.
4. **Obstacle avoidance:** adds steering when the vehicle approaches a keep-out circle.
5. **Vehicle model:** simulated velocity approaches commanded velocity, then position is updated.
6. **Telemetry:** each time step records position, velocity, target, waypoint index, and a toy battery estimate.
7. **Visualization:** Matplotlib plots the trajectory, waypoint route, and keep-out zones.

## 3. PID concept
PID means Proportional, Integral, and Derivative:
- **P** responds to current error. A larger distance to the waypoint creates a larger command.
- **I** accumulates error over time. This example uses a small integral gain and clamps the accumulated value to prevent runaway windup.
- **D** reacts to how quickly error changes, which can help reduce overshoot in a properly tuned system.

For each axis, the simplified command is:
`command = Kp * error + Ki * accumulated_error + Kd * rate_of_change_of_error`

The result is clamped to the maximum simulated speed. These values are demonstration values, not real-flight tuning values.

## 4. Waypoint navigation
The simulation targets one waypoint at a time. When the virtual position enters `WAYPOINT_RADIUS`, the index advances to the next point. This is a simplified form of waypoint following.

## 5. Obstacle avoidance
Each obstacle is a circle with a radius. If the vehicle is near the circle's expanded keep-out radius, the code adds a repulsive vector and a small tangential vector. This is easy to visualize and understand, but it is not a globally optimal path planner and may fail in narrow passages or complex obstacle layouts.

## 6. Simulated vehicle dynamics
The simulation does not model aerodynamics, attitude, thrust, motor response, GPS noise, IMU drift, wind, or battery physics. Instead, it makes velocity approach the commanded velocity by a fixed fraction each time step. This keeps the example understandable while making it more realistic than teleporting directly to each waypoint.

## 7. Telemetry
The CSV records:
- `time_s`: simulation time
- `x`, `y`: virtual position
- `vx`, `vy`: virtual velocity
- `target_x`, `target_y`: active waypoint
- `waypoint_index`: current mission progress
- `battery_percent_simulated`: illustrative battery estimate only

## 8. How to demonstrate it
1. Install dependencies using `python -m pip install -r requirements.txt`.
2. Run `python src/autopilot_sim.py`.
3. Show the route plot and `telemetry.csv`.
4. Change one waypoint and run again.
5. Change an obstacle location and compare the path.
6. Run the unit tests with `python -m unittest discover -s tests -v`.

## 9. Honest limitations
This is a 2D educational simulation, not a real autopilot and not a validated flight simulator. It cannot control a physical drone. Obstacle avoidance is a simple steering heuristic, and the battery percentage is a toy estimate. Clearly state these limitations in an interview.

## 10. Future work
- Use a recognized simulator such as PX4 SITL with Gazebo after learning its setup.
- Compare waypoint error with different controller gains.
- Add sensor noise and wind disturbances to the simulation.
- Add a proper path planner such as A* on a grid.
- Add a dashboard for telemetry visualization.
