"""
Educational 2D autopilot simulation.
This is not real flight-control firmware and must not be connected to a real drone.
"""
from __future__ import annotations
import csv
import math
from pathlib import Path
import matplotlib.pyplot as plt

# Route points in arbitrary 2D simulation units.
WAYPOINTS = [(1, 1), (8, 2), (11, 8), (17, 9), (19, 3)]
# Circular keep-out zones: (x, y, radius)
OBSTACLES = [(6.0, 4.2, 1.25), (13.8, 5.8, 1.35)]
DT = 0.1
MAX_SPEED = 1.6
WAYPOINT_RADIUS = 0.45
OBSTACLE_MARGIN = 0.85
SIM_SECONDS = 80


def clamp(value: float, low: float, high: float) -> float:
    """Limit a value to a defined range."""
    return max(low, min(high, value))


class PID:
    """Small PID controller with integral clamping and derivative smoothing."""
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float = 2.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.integral_limit = integral_limit

    def update(self, error: float, dt: float) -> float:
        self.integral = clamp(self.integral + error * dt,
                              -self.integral_limit, self.integral_limit)
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


def distance(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def avoid_obstacles(position, desired_velocity):
    """
    Simple local steering rule for demonstration.
    If the predicted next point approaches a keep-out circle, add a sideways
    velocity component. This is not a certified planner and has limitations.
    """
    vx, vy = desired_velocity
    px, py = position
    for ox, oy, radius in OBSTACLES:
        dx, dy = px - ox, py - oy
        dist = math.hypot(dx, dy)
        safe_radius = radius + OBSTACLE_MARGIN
        if dist < safe_radius + 1.4:
            if dist < 1e-6:
                dx, dy, dist = 1.0, 0.0, 1.0
            # Repulsion grows as the simulated vehicle nears the keep-out zone.
            strength = clamp((safe_radius + 1.4 - dist) / (safe_radius + 1.4), 0, 1)
            # A radial push plus a small tangential component helps it pass around.
            vx += 1.7 * strength * dx / dist + 0.45 * strength * (-dy / dist)
            vy += 1.7 * strength * dy / dist + 0.45 * strength * (dx / dist)
    mag = math.hypot(vx, vy)
    if mag > MAX_SPEED:
        vx, vy = vx * MAX_SPEED / mag, vy * MAX_SPEED / mag
    return vx, vy


def simulate():
    x, y = WAYPOINTS[0]
    vx, vy = 0.0, 0.0
    waypoint_index = 1
    pid_x = PID(1.25, 0.015, 0.18)
    pid_y = PID(1.25, 0.015, 0.18)
    history = []
    battery = 100.0
    total_steps = int(SIM_SECONDS / DT)

    for step in range(total_steps):
        time_s = step * DT
        target = WAYPOINTS[waypoint_index] if waypoint_index < len(WAYPOINTS) else WAYPOINTS[-1]
        err_x, err_y = target[0] - x, target[1] - y
        if math.hypot(err_x, err_y) < WAYPOINT_RADIUS and waypoint_index < len(WAYPOINTS) - 1:
            waypoint_index += 1
            target = WAYPOINTS[waypoint_index]
            err_x, err_y = target[0] - x, target[1] - y

        desired_vx = clamp(pid_x.update(err_x, DT), -MAX_SPEED, MAX_SPEED)
        desired_vy = clamp(pid_y.update(err_y, DT), -MAX_SPEED, MAX_SPEED)
        desired_vx, desired_vy = avoid_obstacles((x, y), (desired_vx, desired_vy))

        # Simplified vehicle dynamics: velocity approaches the commanded velocity.
        vx += (desired_vx - vx) * 0.35
        vy += (desired_vy - vy) * 0.35
        x += vx * DT
        y += vy * DT

        # Battery is a toy estimate, not a physical battery model.
        battery = max(0.0, battery - (0.0012 + 0.00018 * math.hypot(vx, vy)) * DT)
        history.append({
            "time_s": round(time_s, 2), "x": x, "y": y, "vx": vx, "vy": vy,
            "target_x": target[0], "target_y": target[1],
            "waypoint_index": waypoint_index, "battery_percent_simulated": battery
        })
        if waypoint_index == len(WAYPOINTS) - 1 and distance((x, y), WAYPOINTS[-1]) < WAYPOINT_RADIUS:
            # Record a few seconds of stable arrival, then end.
            if time_s > 2:
                break

    return history


def export_csv(history, output_path: Path):
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(history[0].keys()))
        writer.writeheader()
        writer.writerows(history)


def plot_history(history, output_path: Path):
    xs = [r["x"] for r in history]
    ys = [r["y"] for r in history]
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(xs, ys, label="Simulated trajectory", linewidth=2.2)
    wx, wy = zip(*WAYPOINTS)
    ax.plot(wx, wy, "--o", label="Waypoint route")
    for i, (ox, oy, radius) in enumerate(OBSTACLES, 1):
        circle = plt.Circle((ox, oy), radius + OBSTACLE_MARGIN,
                            color="crimson", alpha=0.16)
        ax.add_patch(circle)
        ax.text(ox, oy, f"Keep-out {i}", ha="center", va="center", fontsize=8)
    ax.scatter([xs[0]], [ys[0]], marker="s", s=75, label="Start")
    ax.scatter([xs[-1]], [ys[-1]], marker="*", s=150, label="Last position")
    ax.set_title("Educational Autopilot Drone — 2D Simulation")
    ax.set_xlabel("X position (simulation units)")
    ax.set_ylabel("Y position (simulation units)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.show()


def main():
    history = simulate()
    root = Path(__file__).resolve().parents[1]
    csv_path = root / "telemetry.csv"
    plot_path = root / "flight_path.png"
    export_csv(history, csv_path)
    plot_history(history, plot_path)
    final = history[-1]
    print(f"Simulation steps: {len(history)}")
    print(f"Final position: x={final['x']:.2f}, y={final['y']:.2f}")
    print(f"Simulated battery estimate: {final['battery_percent_simulated']:.1f}%")
    print(f"Saved telemetry: {csv_path}")
    print(f"Saved plot: {plot_path}")


if __name__ == "__main__":
    main()
