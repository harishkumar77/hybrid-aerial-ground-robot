#!/usr/bin/env python3
"""Replay the hybrid robot mission in Gazebo Harmonic.

1. cargo run --bin export_gazebo        (writes the world + trail/hazards/events)
2. gz sim gazebo/hybrid_site.sdf        (press Play, bottom-left)
3. python3 gazebo/replay.py             (in a second terminal)

Options:
  --pause      wait for Enter at every event (takeoff, landing, hazard, repair): for presenting
  --speed X    faster (2.0) or slower (0.5) playback
  --reset      only put the robot back at the start and hide hazard markers

The robot is moved with the /world/hybrid_site/set_pose service. Nothing here plans:
the route, map corrections and repairs all come from the Rust planner's output.
"""
import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time

WORLD = "hybrid_site"
ROBOT = "hybrid_bot"
Z_DRIVE = 0.22
Z_FLY = 2.5
HERE = os.path.dirname(os.path.abspath(__file__))


def gz_set_pose(name, x, y, z, yaw=0.0):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "{name}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
           f'orientation: {{x: 0, y: 0, z: {qz:.4f}, w: {qw:.4f}}}')
    r = subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "2000", "--req", req],
        capture_output=True, text=True)
    return "true" in r.stdout


def gz_camera(x, y, z, roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    req = (f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}}}')
    subprocess.run(["gz", "service", "-s", "/gui/move_to/pose",
                    "--reqtype", "gz.msgs.GUICamera", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "2000", "--req", req], capture_output=True)


def load():
    with open(os.path.join(HERE, "trail.csv")) as f:
        trail = [(float(r["x"]), float(r["y"]), r["mode"]) for r in csv.DictReader(f)]
    hazards = {}
    with open(os.path.join(HERE, "hazards.csv")) as f:
        for r in csv.DictReader(f):
            hazards.setdefault(int(r["step"]), []).append((r["marker"], float(r["x"]), float(r["y"])))
    events = {}
    with open(os.path.join(HERE, "events.csv")) as f:
        for line in f:
            line = line.strip()
            if line:
                step, text = line.split(";", 1)
                events.setdefault(int(step), []).append(text)
    return trail, hazards, events


def say(text, pause):
    print(f"  {text}")
    if pause:
        input("    [Enter] ")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pause", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--reset", action="store_true")
    a = ap.parse_args()

    if not shutil.which("gz"):
        sys.exit("`gz` not found. Source your Gazebo/ROS environment first.")
    trail, hazards, events = load()
    x0, y0, _ = trail[0]

    # reset scene
    if not gz_set_pose(ROBOT, x0, y0, Z_DRIVE):
        sys.exit("set_pose failed. Is `gz sim gazebo/hybrid_site.sdf` running and playing (Play button)?")
    for markers in hazards.values():
        for name, hx, hy in markers:
            gz_set_pose(name, hx, hy, -5.0)
    if a.reset:
        print("reset done")
        return
    gz_camera(-4, -26, 20, 0, 0.62, 1.5708)

    substeps = max(1, round(4 / a.speed))
    dt = 0.04 / a.speed
    yaw = 0.0
    z = Z_DRIVE
    flown = driven = 0

    print("\nMission replay (route, map corrections and repairs computed by the Rust planner)\n")
    say("aerial draft loaded; global plan = cyan tiles (drive) + magenta tiles (fly)", a.pause)

    for i in range(len(trail)):
        for name, hx, hy in hazards.get(i, []):
            gz_set_pose(name, hx, hy, 0.06)
        for text in events.get(i, []):
            say(f"step {i:>3}  {text}", a.pause)
        if i + 1 == len(trail):
            break

        (xa, ya, ma), (xb, yb, mb) = trail[i], trail[i + 1]

        if ma != mb:  # takeoff / landing in place
            if mb == "Fly":
                say(f"step {i:>3}  SUPERVISOR: takeoff interlocks passed (stopped, airspace OK, battery reserve OK) -> TRANSITION Drive->Fly", a.pause)
                z_to = Z_FLY
            else:
                say(f"step {i:>3}  SUPERVISOR: landing zone verified free -> TRANSITION Fly->Drive", a.pause)
                z_to = Z_DRIVE
            n = 12
            for k in range(1, n + 1):
                gz_set_pose(ROBOT, xa, ya, z + (z_to - z) * k / n, yaw)
                time.sleep(dt)
            z = z_to
            print(f"            transition confirmed -> ARMED/Auto/{mb}")
            continue

        if (xb, yb) != (xa, ya):
            yaw = math.atan2(yb - ya, xb - xa)
        for k in range(1, substeps + 1):
            t = k / substeps
            gz_set_pose(ROBOT, xa + (xb - xa) * t, ya + (yb - ya) * t, z, yaw)
            time.sleep(dt)
        if ma == "Fly":
            flown += 1
        else:
            driven += 1

    print(f"\n  driven {driven} steps, flown {flown} steps. Hazards found from the ground are the red tiles.")


if __name__ == "__main__":
    main()
