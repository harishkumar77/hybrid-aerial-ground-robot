#!/usr/bin/env python3
"""Play a fault scenario in Gazebo Harmonic.

  cargo run                                   (writes the world + scenario files)
  gz sim -r gazebo/fault_site.sdf             (terminal 1)
  python3 gazebo/fault_replay.py --list       (terminal 2)
  python3 gazebo/fault_replay.py planner_dies_flying
  python3 gazebo/fault_replay.py estop_flying --pause --speed 0.7

Robot color = supervisor state:
  green = AUTO   amber = TAKEOFF/LANDING   red = FAIL-SAFE   blue = MANUAL   grey = DISARMED

Every state change and every reaction in the narration comes from the real
hybrid_safety Supervisor, run in a 50 ms loop by `cargo run`. This script only
moves the robot models and prints the narration.
"""
import argparse
import math
import os
import shutil
import subprocess
import sys
import time

WORLD = "fault_site"
HERE = os.path.dirname(os.path.abspath(__file__))
LOOKS = ["disarmed", "manual", "auto", "transition", "failsafe"]
LABEL = {"disarmed": "DISARMED", "manual": "MANUAL", "auto": "AUTO",
         "transition": "TAKEOFF/LANDING", "failsafe": "FAIL-SAFE"}
COLOR = {"disarmed": "\033[90m", "manual": "\033[94m", "auto": "\033[92m",
         "transition": "\033[93m", "failsafe": "\033[91m"}
RESET = "\033[0m"


def set_pose(name, x, y, z, yaw=0.0):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "{name}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
           f'orientation: {{x: 0, y: 0, z: {qz:.4f}, w: {qw:.4f}}}')
    r = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose",
                        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "2000", "--req", req], capture_output=True, text=True)
    return "true" in r.stdout


def camera():
    # looking north-east at the river crossing
    yaw, pitch = 1.5708, 0.62
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    req = (f"pose: {{position: {{x: -2, y: -20, z: 15}}, "
           f"orientation: {{x: {-sp * sy:.4f}, y: {sp * cy:.4f}, z: {cp * sy:.4f}, w: {cp * cy:.4f}}}}}")
    subprocess.run(["gz", "service", "-s", "/gui/move_to/pose", "--reqtype", "gz.msgs.GUICamera",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req],
                   capture_output=True)


def scenarios():
    d = os.path.join(HERE, "scenarios")
    return sorted(f[:-4] for f in os.listdir(d) if f.endswith(".tsv")) if os.path.isdir(d) else []


def load(name):
    frames = []
    with open(os.path.join(HERE, "scenarios", f"{name}.tsv")) as f:
        next(f)
        for line in f:
            p = line.rstrip("\n").split("\t")
            frames.append(dict(t=int(p[0]), x=float(p[1]), y=float(p[2]), z=float(p[3]),
                               yaw=float(p[4]), look=p[5],
                               raise_=[m for m in p[6].split(",") if m],
                               notes=[n for n in p[7].split(" || ") if n]))
    markers = {}
    with open(os.path.join(HERE, "hazards.tsv")) as f:
        next(f)
        for line in f:
            m, x, y = line.rstrip("\n").split("\t")
            markers[m] = (float(x), float(y))
    return frames, markers


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", nargs="?")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--pause", action="store_true", help="wait for Enter at faults and supervisor decisions")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--quiet", action="store_true", help="hide planner (map) notes")
    a = ap.parse_args()

    names = scenarios()
    if a.list or not a.scenario:
        if not names:
            sys.exit("no scenario files. Run `cargo run` in this folder first.")
        print("scenarios:\n  " + "\n  ".join(names))
        return
    if a.scenario not in names:
        sys.exit(f"unknown scenario '{a.scenario}'. Use --list.")
    if not shutil.which("gz"):
        sys.exit("`gz` not found. Source your Gazebo environment first.")

    frames, markers = load(a.scenario)

    # reset scene
    if not set_pose("bot_" + frames[0]["look"], frames[0]["x"], frames[0]["y"], frames[0]["z"]):
        sys.exit("set_pose failed. Is `gz sim -r gazebo/fault_site.sdf` running (and playing)?")
    for lk in LOOKS:
        if lk != frames[0]["look"]:
            set_pose("bot_" + lk, 0, 0, -20)
    for m, (x, y) in markers.items():
        set_pose(m, x, y, -5)
    set_pose("hybrid_bot", 0, 0, -20)
    camera()

    print(f"\n=== {a.scenario} ===")
    print("robot color:  " + "  ".join(f"{COLOR[k]}{LABEL[k]}{RESET}" for k in ["auto", "transition", "failsafe", "manual", "disarmed"]) + "\n")

    current = frames[0]["look"]
    t0 = time.monotonic()
    for fr in frames:
        if fr["look"] != current:
            set_pose("bot_" + current, 0, 0, -20)
            current = fr["look"]
        set_pose("bot_" + current, fr["x"], fr["y"], fr["z"], fr["yaw"])
        for m in fr["raise_"]:
            x, y = markers[m]
            set_pose(m, x, y, 0.06)

        stop = False
        for n in fr["notes"]:
            if a.quiet and n.startswith("planner:"):
                continue
            print(f"  t={fr['t'] / 1000:5.2f}s  {COLOR[fr['look']]}[{LABEL[fr['look']]:<15}]{RESET} {n}")
            if n.startswith("FAULT") or "FAILSAFE" in n or "REJECTED" in n or "DISARMED" in n.split("->")[-1]:
                stop = True
        if a.pause and stop:
            input("      [Enter] ")
            t0 = time.monotonic() - fr["t"] / 1000 / a.speed

        wait = t0 + fr["t"] / 1000 / a.speed - time.monotonic()
        if wait > 0:
            time.sleep(wait)

    print(f"\n  final state: {COLOR[current]}{LABEL[current]}{RESET}\n")


if __name__ == "__main__":
    main()
