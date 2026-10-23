#!/usr/bin/env python3
"""gait_sim.py - crawl-gait stair-climbing simulator for the PBL-i2 quadruped.

What it does
    Plays a "climb one step" cycle on a staircase, one leg at a time:
        shift COG -> lift leg -> land   (first pair of legs)
        push body forward and up
        shift COG -> lift leg -> land   (second pair of legs)
    gait.leg_order in robot_config.yaml picks the pairs:
        front_rear : FR, FL | push | RR, RL   (the original flowchart)
        diagonal   : FR, RL | push | FL, RR   (classic crawl, default)
    and repeats until all four feet are on the top platform. For every frame it
    checks that
      1. every leg can reach its foot position (IK) inside the joint limits, and
      2. the COG stays inside the support polygon (static stability, plan view).
    It prints a report and can plot / animate the result.

How to run (from software/tools/)
    pip install -r requirements.txt
    python gait_sim.py                       # report + plot window
    python gait_sim.py --save result.png     # report + PNG (side view + stability)
    python gait_sim.py --gif climb.gif       # animation (needs: pip install pillow)
    python gait_sim.py --cycles 6

All dimensions come from robot_config.yaml (placeholders until the design is final).

Model and simplifications (read before trusting the result)
    * Side view uses the same hip/knee model as leg_kinematics.py. Left and right
      legs are assumed to do the same thing; body pitch and roll are ignored.
    * Legs have no sideways joint, so feet stay at +/- body_width/2. The COG is
      therefore fixed laterally (cog_lateral_mm). The stability check is done in
      plan view with that fixed lateral position.
    * Only static stability is checked (slow gait, no dynamics, no torque).
      Use `python leg_kinematics.py torque` for servo load.
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from leg_kinematics import CONFIG_PATH, Leg2DOF, load_config  # noqa: E402

# (front=+1 / rear=-1, left=+1 / right=-1)
LEGS = {"FL": (1, 1), "FR": (1, -1), "RL": (-1, 1), "RR": (-1, -1)}
COLORS = {"FL": "tab:blue", "FR": "tab:cyan", "RL": "tab:red", "RR": "tab:orange"}


class Stairs:
    def __init__(self, cfg):
        s = cfg["stairs"]
        self.h = s["step_height_mm"]
        self.d = s.get("step_depth_mm", 80)
        self.n = s.get("num_steps", 3)
        self.gap = s.get("start_gap_mm", 20)
        self.edge_margin = s.get("foot_edge_margin_mm", 10)

    def surface(self, x):
        if x < 0:
            return 0.0
        return min(math.floor(x / self.d) + 1, self.n) * self.h

    def edges(self):
        return [k * self.d for k in range(self.n)]

    @property
    def top(self):
        return self.n * self.h


def convex_hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def polygon_margin(points, p):
    """Signed distance (mm) from p to the nearest edge of the convex hull; > 0 = inside."""
    hull = convex_hull(points)
    if len(hull) < 3:
        return -999.0
    m = float("inf")
    for i, a in enumerate(hull):
        b = hull[(i + 1) % len(hull)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        m = min(m, (ex * (p[1] - a[1]) - ey * (p[0] - a[0])) / math.hypot(ex, ey))
    return m


class Sim:
    def __init__(self, cfg):
        self.cfg = cfg
        self.leg = Leg2DOF(cfg)
        self.st = Stairs(cfg)
        r, g = cfg["robot"], cfg["gait"]
        self.stand = r["stand_height_mm"]
        self.L = r.get("body_length_mm", 120)
        self.W = r.get("body_width_mm", 90)
        self.cog_dx = r.get("cog_offset_x_mm", 0)
        self.cog_z = r.get("cog_lateral_mm", 0)
        self.fwd = g["swing_forward_mm"]
        self.clr = g["step_clearance_mm"]
        self.advance = g.get("body_advance_mm", 60)
        self.max_shift = int(g.get("max_cog_shift_mm", 40))
        self.req = g.get("stability_margin_mm", 5)
        self.n = int(g.get("frames_per_phase", 10))
        order = g.get("leg_order", "diagonal")
        self.groups = {
            "front_rear": (("FR", "FL"), ("RR", "RL")),   # flowchart order
            "diagonal": (("FR", "RL"), ("FL", "RR")),     # classic crawl
        }[order]
        self.frames = []
        self.max_shift_used = 0.0

    # ---- geometry helpers -------------------------------------------------
    def hip_x(self, leg, bx):
        return bx + LEGS[leg][0] * self.L / 2

    def foot_z(self, leg):
        return LEGS[leg][1] * self.W / 2

    def problems(self, bx, by, feet):
        out = []
        for name, (fx, fy) in feet.items():
            sol = self.leg.ik(fx - self.hip_x(name, bx), fy - by)
            if sol is None:
                out.append(f"{name} out of reach")
            elif not self.leg.in_limits(*sol):
                out.append(f"{name} joint limit (hip {sol[0]:.0f}, knee {sol[1]:.0f})")
        return out

    def margin(self, bx, feet, exclude=None):
        pts = [(fx, self.foot_z(n)) for n, (fx, fy) in feet.items() if n != exclude]
        return polygon_margin(pts, (bx + self.cog_dx, self.cog_z))

    def land_x(self, hip_x):
        x = hip_x + self.fwd
        for e in self.st.edges():
            if abs(x - e) < self.st.edge_margin:
                x = e + self.st.edge_margin
        return x

    def add(self, label, cycle, bx, by, feet, exclude=None):
        self.frames.append({
            "label": label, "cycle": cycle, "bx": bx, "by": by,
            "feet": dict(feet), "swing": exclude,
            "problems": self.problems(bx, by, feet),
            "margin": self.margin(bx, feet, exclude),
        })

    # ---- gait phases ------------------------------------------------------
    def best_shift(self, leg, bx, by, feet):
        best = None
        for d in range(-self.max_shift, self.max_shift + 1):
            cx = bx + d
            if self.problems(cx, by, feet):
                continue
            key = (min(self.margin(cx, feet, leg), 2 * self.req), -abs(d))
            if best is None or key > best[0]:
                best = (key, cx)
        return bx if best is None else best[1]

    def shift(self, leg, cycle, bx, by, feet):
        cx = self.best_shift(leg, bx, by, feet)
        self.max_shift_used = max(self.max_shift_used, abs(cx - bx))
        steps = self.n if cx != bx else 1
        for i in range(1, steps + 1):
            # all four feet are still down while sliding; only the last frame
            # (just before the lift) must be stable on the 3 remaining legs
            ex = leg if i == steps else None
            self.add(f"shift for {leg}", cycle, bx + (cx - bx) * i / steps, by, feet, ex)
        return cx

    def swing(self, leg, cycle, bx, by, feet):
        x0, y0 = feet[leg]
        x1 = self.land_x(self.hip_x(leg, bx))
        y1 = self.st.surface(x1)
        for i in range(1, self.n + 1):
            s = i / self.n
            x = x0 + (x1 - x0) * s
            ahead = max(self.st.surface(x), self.st.surface(x + 10))
            y = max(y0 + (y1 - y0) * s, ahead) + self.clr * math.sin(math.pi * s)
            moving = dict(feet)
            moving[leg] = (x, y)
            self.add(f"swing {leg}", cycle, bx, by, moving, leg)
        feet[leg] = (x1, y1)
        self.add(f"land {leg}", cycle, bx, by, feet)

    def push(self, cycle, bx, by, feet):
        by1 = self.stand + sum(f[1] for f in feet.values()) / 4
        bx1 = bx + self.advance
        for i in range(1, self.n + 1):
            s = i / self.n
            self.add("push body", cycle, bx + (bx1 - bx) * s, by + (by1 - by) * s, feet)
        return bx1, by1

    def run(self, max_cycles):
        fx = -self.st.gap
        bx = fx - self.L / 2
        by = float(self.stand)
        feet = {n: (fx - (0 if LEGS[n][0] == 1 else self.L), 0.0) for n in LEGS}
        self.add("start", 0, bx, by, feet)
        for cycle in range(1, max_cycles + 1):
            for gi, group in enumerate(self.groups):
                for leg in group:
                    bx = self.shift(leg, cycle, bx, by, feet)
                    self.swing(leg, cycle, bx, by, feet)
                if gi == 0:
                    bx, by = self.push(cycle, bx, by, feet)
            if all(f[1] >= self.st.top - 1e-6 for f in feet.values()):
                break
        return self.frames


def report(sim):
    fr = sim.frames
    cycles = fr[-1]["cycle"]
    done = all(f[1] >= sim.st.top - 1e-6 for f in fr[-1]["feet"].values())
    worst = min(fr, key=lambda f: f["margin"])
    print(f"cycles: {cycles}, frames: {len(fr)}, all feet on top: {'yes' if done else 'NO'}")
    print(f"largest body shift over planted feet: {sim.max_shift_used:.0f} mm "
          f"(limit {sim.max_shift})")
    print(f"min stability margin: {worst['margin']:.1f} mm in '{worst['label']}' "
          f"cycle {worst['cycle']} (required {sim.req})")
    bad = [f for f in fr if f["problems"]]
    unstable = [f for f in fr if f["margin"] < sim.req]
    counts = {}
    for f in bad:
        for p in f["problems"]:
            counts[p.split(" (")[0]] = counts.get(p.split(" (")[0], 0) + 1
    if counts:
        print("leg problems (frames affected):")
        for k, v in sorted(counts.items(), key=lambda kv: -kv[1])[:8]:
            print(f"   {k}: {v}")
    print(f"frames below required stability margin: {len(unstable)}")
    ok = done and not bad and not unstable
    print("RESULT:", "PASS" if ok else "FAIL - see problems above")
    return ok


def draw_scene(ax, sim, frame=None, trails=False):
    st = sim.st
    xs, ys = [-150.0], [0.0]
    for k in range(st.n):
        xs += [k * st.d, k * st.d]
        ys += [k * st.h, (k + 1) * st.h]
    xs.append(st.n * st.d + 200)
    ys.append(st.top)
    ax.plot(xs, ys, "k-", lw=2)
    if trails:
        for leg in LEGS:
            pts = [f["feet"][leg] for f in sim.frames]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], "-", lw=0.8,
                    color=COLORS[leg], alpha=0.7, label=leg)
        ax.legend(loc="upper left", fontsize=8)
    if frame is not None:
        bx, by = frame["bx"], frame["by"]
        ax.plot([sim.hip_x("RL", bx), sim.hip_x("FL", bx)], [by, by], "k-", lw=4)
        for leg in ("FL", "RL"):
            hx = sim.hip_x(leg, bx)
            fx, fy = frame["feet"][leg]
            sol = sim.leg.ik(fx - hx, fy - by)
            if sol is None:
                ax.plot([hx, fx], [by, fy], "r:", lw=1)
                continue
            knee, _ = sim.leg.fk(*sol)
            ax.plot([hx, hx + knee[0], fx], [by, by + knee[1], fy], "o-",
                    color=COLORS[leg], lw=2)
        ax.plot(bx + sim.cog_dx, by, "k*", ms=10)
    ax.set_aspect("equal")
    ax.set_xlabel("x forward (mm)")
    ax.set_ylabel("y up (mm)")


def plot(sim, save=None):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 9),
                                 gridspec_kw={"height_ratios": [3, 2]})
    draw_scene(a1, sim, sim.frames[-1], trails=True)
    a1.set_title("Foot paths (all cycles) and final pose")
    m = [f["margin"] for f in sim.frames]
    a2.plot(m, lw=1.2)
    a2.axhline(sim.req, color="r", ls="--", label=f"required margin {sim.req} mm")
    bad = [i for i, f in enumerate(sim.frames) if f["problems"]]
    if bad:
        a2.plot(bad, [m[i] for i in bad], "rx", label="leg out of reach / limit")
    a2.set_xlabel("frame")
    a2.set_ylabel("COG margin (mm)")
    a2.set_title("Static stability margin (plan view)")
    a2.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=150)
        print("saved", save)
    else:
        plt.show()


def animate(sim, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    fig, ax = plt.subplots(figsize=(8, 4.5))

    def update(i):
        ax.clear()
        f = sim.frames[i]
        draw_scene(ax, sim, f)
        ax.set_xlim(-150, sim.st.n * sim.st.d + 200)
        ax.set_ylim(-10, sim.stand + sim.st.top + 60)
        ax.set_title(f"cycle {f['cycle']}: {f['label']}  (margin {f['margin']:.0f} mm)")

    FuncAnimation(fig, update, frames=len(sim.frames)).save(
        path, writer=PillowWriter(fps=8))
    print("saved", path)


def main():
    p = argparse.ArgumentParser(description="Crawl-gait stair simulator")
    p.add_argument("--config", default=str(CONFIG_PATH))
    p.add_argument("--cycles", type=int, help="max climb cycles (default: steps + 6)")
    p.add_argument("--save", help="write PNG instead of opening a window")
    p.add_argument("--gif", help="write an animated GIF (needs pillow)")
    p.add_argument("--no-plot", action="store_true")
    args = p.parse_args()
    cfg = load_config(args.config)
    sim = Sim(cfg)
    sim.run(args.cycles or cfg["stairs"].get("num_steps", 3) + 6)
    report(sim)
    if args.gif:
        try:
            animate(sim, args.gif)
        except ImportError:
            print("GIF needs pillow: pip install pillow")
    if not args.no_plot:
        plot(sim, args.save)


if __name__ == "__main__":
    main()