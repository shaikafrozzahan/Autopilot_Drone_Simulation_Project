# Educational Autopilot Drone — 2D Simulation

A portfolio-ready **simulation-first** project demonstrating waypoint navigation, a simple PID controller, obstacle avoidance, and live telemetry for a virtual quadcopter.

> **Scope and safety:** This repository simulates a drone in 2D. It does not control real flight hardware, motors, or a real aircraft. Do not connect it to a physical drone or use it for real-world flight. Real autopilots require flight-controller firmware, calibrated sensors, failsafes, controlled testing, and applicable aviation rules.

## Features
- 2D virtual drone and configurable waypoints
- PID-style velocity control toward each waypoint
- Circular keep-out zones with basic steering around obstacles
- Simulated battery indicator and telemetry log
- Plots the flight path and telemetry using Matplotlib
- Beginner-readable Python with comments and a viva/interview guide

## Requirements
- Python 3.10+
- `matplotlib`

Install:
```bash
python -m pip install -r requirements.txt
```

Run:
```bash
python src/autopilot_sim.py
```

The program opens a 2D route plot and saves:
- `flight_path.png`
- `telemetry.csv`

Run the tests:
```bash
python -m unittest discover -s tests -v
```

## Repository structure
```text
Autopilot_Drone_Simulation_Project/
├── README.md
├── requirements.txt
├── src/
│   └── autopilot_sim.py
├── tests/
│   └── test_controller.py
├── docs/
│   ├── PROJECT_EXPLANATION.md
│   ├── VIVA_QUESTIONS.md
│   └── DESIGN.md
└── images/
    ├── system_architecture.png
    ├── flight_path_example.png
    └── project_cover.png
```

## Suggested development workflow
1. Run the simulator and inspect the path.
2. Change waypoint coordinates in `WAYPOINTS`.
3. Add a new keep-out circle in `OBSTACLES`.
4. Compare runs with obstacle avoidance enabled and disabled.
5. Explain the controller, limitations, and tests in your own words.

## Resume description (edit to match what you actually ran)
**Educational Autopilot Drone Simulation | Python**
- Built a 2D quadcopter simulation with waypoint-based navigation and a PID-style controller.
- Implemented basic obstacle keep-out behavior and logged simulated position, velocity, waypoint status, and battery level.
- Visualized the planned route and simulated trajectory using Matplotlib.
- Tools: Python, control systems fundamentals, numerical simulation, Matplotlib, CSV telemetry.

**Be accurate:** This is a simulation project, not a real drone build or real flight test. If you modify it, document the changes you personally made and verified.
