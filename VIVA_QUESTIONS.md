# Interview / viva preparation

## 1. Explain your project in 45–60 seconds
“My project is an educational 2D autopilot drone simulation written in Python. It follows a series of waypoints, uses a PID-style controller to convert position error into velocity commands, and applies a simple obstacle-avoidance rule near circular keep-out zones. It logs position and velocity telemetry to CSV and plots the trajectory using Matplotlib. It is simulation-only; it does not control a physical drone. I designed it to learn navigation and feedback-control concepts before moving to a recognized flight simulator.”

## 2. What is an autopilot?
An autopilot is a control system that executes a flight or navigation task using sensor/state estimates and control commands. In this project, the term describes the simulated navigation logic only.

## 3. What is a waypoint?
A waypoint is a target position that the vehicle attempts to reach. When the simulation enters a small radius around a target, it switches to the next waypoint.

## 4. Why use PID?
PID is a widely taught feedback-control method. The proportional term responds to current error, the integral term accounts for accumulated error, and the derivative term responds to the error's rate of change. This example uses a simplified position-to-velocity controller.

## 5. Is this a real drone?
No. It is a 2D Python simulation. It does not model a real quadcopter's roll, pitch, yaw, thrust, motors, IMU, GPS, wind, or flight dynamics.

## 6. How does obstacle avoidance work?
When the virtual drone approaches an expanded circular keep-out zone, the program adds a vector that pushes it away and a small sideways component. This is a simple heuristic, not a guaranteed safe path planner.

## 7. What is telemetry?
Telemetry is recorded information about the system over time. Here it includes time, position, velocity, active waypoint, and an illustrative battery estimate.

## 8. What are the weaknesses?
The model is 2D, uses idealized dynamics, has no sensor noise or wind, and the obstacle rule may fail in complex environments. The battery value is only illustrative.

## 9. How did you test it?
Use the included unit tests for helper functions and speed limiting, then run the simulation and inspect the generated route and CSV. Do not claim these tests or results until you run them yourself.

## 10. What would you improve next?
I would add sensor noise and wind, compare controller gains quantitatively, implement grid-based A* path planning, and later explore PX4 SITL/Gazebo. I would not connect the educational code directly to real flight hardware.

## 11. What does the integral limit do?
It limits the accumulated error so the integral term cannot grow without bound. This is a basic anti-windup measure.

## 12. What is the difference between simulation and hardware testing?
Simulation executes a mathematical model on a computer. Hardware testing uses physical components and must include calibration, failsafe validation, risk controls, and appropriate legal permissions.
