# Design and implementation notes

## Data flow
```text
Waypoint list
     |
     v
Position error <----- Simulated position
     |
     v
PID-style controller
     |
     v
Obstacle steering rule
     |
     v
Velocity command -> simplified dynamics -> new position
     |                                  |
     +---------- telemetry log <--------+
                        |
                        v
                 CSV + Matplotlib plot
```

## Main parameters
- `DT`: simulation time step in seconds.
- `MAX_SPEED`: upper bound on simulated velocity magnitude.
- `WAYPOINT_RADIUS`: distance threshold to advance to the next waypoint.
- `OBSTACLES`: circles represented by `(x, y, radius)`.
- `Kp`, `Ki`, `Kd`: controller gains for each axis.

## Suggested experiments
1. Increase `Kp` and observe whether the trajectory overshoots more.
2. Reduce `MAX_SPEED` and compare travel time.
3. Move an obstacle closer to the route.
4. Change `WAYPOINT_RADIUS` and inspect when the waypoint index advances.
5. Add random measurement noise as a future experiment.
