#!/usr/bin/env python3
"""leg_kinematics.py - 2-DOF leg (hip + knee) tools for the PBL-i2 quadruped.

What it does
    workspace : plot every foot position the leg can reach
    ik X Y    : joint angles needed to put the foot at (X, Y) mm from the hip
    stairs    : check that the foot can lift over and land on a step, plot it
    torque    : rough static servo-torque estimate for the stair poses

How to run (from software/tools/)
    pip install -r requirements.txt
    python leg_kinematics.py workspace
    python leg_kinematics.py ik 40 -40
    python leg_kinematics.py stairs --save stairs.png
    python leg_kinematics.py torque

Dimensions come from robot_config.yaml (placeholders until the design is final).

Frame: hip axis at the origin, x forward, y up (ground is y = -stand_height).
Angles: hip = thigh angle from straight down (+ forward);
        knee = shank angle relative to the thigh (0 = straight leg).
"""
import argparse
import math
from pathlib import Path

import numpy as np
import yaml

CONFIG_PATH = Path(__file__).with_name("robot_config.yaml")


def load_config(path=CONFIG_PATH):
    with open(path) as f:
        return yaml.safe_load(f)


class Leg2DOF:
    def __init__(self, cfg):
        leg = cfg["leg"]
        self.l1 = leg["l1_thigh_mm"]
        self.l2 = leg["l2_shank_mm"]
        self.hip_min, self.hip_max = leg["hip_limits_deg"]
        self.knee_max = leg["knee_max_deg"]
        # knee behind the leg line -> knee angle is positive
        self.sign = 1 if leg.get("knee_bend", "back") == "back" else -1

    def fk(self, hip_deg, knee_deg):
        """Return (knee_xy, foot_xy) in mm."""
        a1 = math.radians(hip_deg)
        a2 = a1 + math.radians(knee_deg)
        knee = (self.l1 * math.sin(a1), -self.l1 * math.cos(a1))
        foot = (knee[0] + self.l2 * math.sin(a2), knee[1] - self.l2 * math.cos(a2))
        return knee, foot

    def ik(self, x, y):
        """Return (hip_deg, knee_deg) or None if the point is out of reach."""
        d2 = x * x + y * y
        c2 = (d2 - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        if abs(c2) > 1:
            return None
        th2 = self.sign * math.acos(c2)
        phi = math.atan2(x, -y)
        th1 = phi - math.atan2(self.l2 * math.sin(th2), self.l1 + self.l2 * math.cos(th2))
        return math.degrees(th1), math.degrees(th2)

    def in_limits(self, hip_deg, knee_deg):
        k = self.sign * knee_deg
        return self.hip_min <= hip_deg <= self.hip_max and -1e-6 <= k <= self.knee_max


def stair_poses(cfg):
    """Foot targets for one front-leg step: liftoff, apex, touchdown."""
    stand = cfg["robot"]["stand_height_mm"]
    h = cfg["stairs"]["step_height_mm"]
    fwd = cfg["gait"]["swing_forward_mm"]
    clr = cfg["gait"]["step_clearance_mm"]
    return [
        ("liftoff", 0.0, -stand),
        ("apex", fwd / 2, -stand + h + clr),
        ("touchdown", fwd, -stand + h),
    ]


def cmd_ik(leg, cfg, args):
    sol = leg.ik(args.x, args.y)
    if sol is None:
        print(f"({args.x}, {args.y}) is out of reach "
              f"(reach {abs(leg.l1 - leg.l2):.0f}..{leg.l1 + leg.l2:.0f} mm)")
        return
    ok = "OK" if leg.in_limits(*sol) else "OUTSIDE joint limits"
    print(f"hip = {sol[0]:.1f} deg, knee = {sol[1]:.1f} deg  [{ok}]")


def cmd_workspace(leg, cfg, args):
    import matplotlib.pyplot as plt
    pts = []
    for hip in np.linspace(leg.hip_min, leg.hip_max, 120):
        for k in np.linspace(0, leg.knee_max, 120):
            pts.append(leg.fk(hip, leg.sign * k)[1])
    pts = np.array(pts)
    stand = cfg["robot"]["stand_height_mm"]
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(pts[:, 0], pts[:, 1], s=2, alpha=0.4, label="reachable foot positions")
    ax.axhline(-stand, color="gray", ls="--", label="ground (standing)")
    ax.plot(0, 0, "ko", label="hip")
    ax.set_aspect("equal")
    ax.set_xlabel("x forward (mm)")
    ax.set_ylabel("y up (mm)")
    ax.set_title("Leg workspace")
    ax.legend(loc="lower left")
    finish(fig, args)


def cmd_stairs(leg, cfg, args):
    import matplotlib.pyplot as plt
    stand = cfg["robot"]["stand_height_mm"]
    h = cfg["stairs"]["step_height_mm"]
    fwd = cfg["gait"]["swing_forward_mm"]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot([-60, fwd * 0.6, fwd * 0.6, fwd + 60],
            [-stand, -stand, -stand + h, -stand + h], "k-", lw=2, label="stairs")
    ax.plot(0, 0, "ko")
    all_ok = True
    colors = {"liftoff": "tab:blue", "apex": "tab:orange", "touchdown": "tab:green"}
    for name, x, y in stair_poses(cfg):
        sol = leg.ik(x, y)
        if sol is None:
            print(f"{name:10s} ({x:5.1f},{y:6.1f})  OUT OF REACH")
            all_ok = False
            continue
        ok = leg.in_limits(*sol)
        all_ok &= ok
        print(f"{name:10s} ({x:5.1f},{y:6.1f})  hip {sol[0]:6.1f}  knee {sol[1]:6.1f}  "
              f"{'OK' if ok else 'OUTSIDE joint limits'}")
        knee, foot = leg.fk(*sol)
        ax.plot([0, knee[0], foot[0]], [0, knee[1], foot[1]], "o-",
                color=colors[name], label=name)
    print("RESULT:", "step is feasible" if all_ok else "step is NOT feasible with these settings")
    ax.set_aspect("equal")
    ax.set_xlabel("x forward (mm)")
    ax.set_ylabel("y up (mm)")
    ax.set_title(f"Front-leg step over {h} mm")
    ax.legend(loc="lower left")
    finish(fig, args)


def cmd_torque(leg, cfg, args):
    """Worst case: one leg carries mass/3 x dynamic factor with a vertical foot force."""
    s = cfg["servo"]
    load_kg = cfg["robot"]["mass_kg"] / 3 * s["dynamic_factor"]
    usable = s["stall_torque_kgcm"] * s["derate"]
    print(f"servo: {s.get('model', '?')}")
    print(f"load per leg {load_kg:.2f} kgf, usable servo torque {usable:.2f} kg*cm")
    worst = 0.0
    for name, x, y in stair_poses(cfg):
        sol = leg.ik(x, y)
        if sol is None:
            continue
        knee, foot = leg.fk(*sol)
        hip_t = load_kg * abs(foot[0]) / 10          # mm -> cm
        knee_t = load_kg * abs(foot[0] - knee[0]) / 10
        worst = max(worst, hip_t, knee_t)
        flag = "OK" if max(hip_t, knee_t) <= usable else "TOO HIGH"
        print(f"{name:10s} hip {hip_t:5.2f}  knee {knee_t:5.2f} kg*cm  [{flag}]")
    if worst > 0:
        max_mass = usable / (worst / load_kg) * 3 / s["dynamic_factor"]
        print(f"worst joint {worst:.2f} kg*cm -> max robot mass about {max_mass:.2f} kg "
              f"with this servo and leg geometry")
    print("Rough estimate only: ignores servo/link mass, friction and shock.")


def finish(fig, args):
    import matplotlib.pyplot as plt
    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print("saved", args.save)
    else:
        plt.show()


def main():
    p = argparse.ArgumentParser(description="2-DOF leg tools")
    p.add_argument("--config", default=str(CONFIG_PATH))
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("workspace", "stairs"):
        sp = sub.add_parser(name)
        sp.add_argument("--save", help="write PNG instead of opening a window")
    sub.add_parser("torque")
    sp = sub.add_parser("ik")
    sp.add_argument("x", type=float)
    sp.add_argument("y", type=float)
    args = p.parse_args()
    if getattr(args, "save", None):
        import matplotlib
        matplotlib.use("Agg")
    cfg = load_config(args.config)
    leg = Leg2DOF(cfg)
    {"workspace": cmd_workspace, "stairs": cmd_stairs,
     "torque": cmd_torque, "ik": cmd_ik}[args.cmd](leg, cfg, args)


if __name__ == "__main__":
    main()