import json
import time
import numpy as np
import track
import mpc

V_TARGETS = [10.0, 14.0, 18.0]
OBSTACLE_SCENARIOS = {
    "none": None,
    "bottom_straight": {"s": 20.0, "ey": 1.5, "radius": 1.4},
    "top_straight": {"s": 105.0, "ey": -1.5, "radius": 1.4},
}

cl = track.Centerline()


def to_world_series(s_list, ey_list):
    xs, ys, heads = [], [], []
    for s, ey in zip(s_list, ey_list):
        x, y, h = cl.to_world(s, ey)
        xs.append(float(x)); ys.append(float(y)); heads.append(float(h))
    return xs, ys, heads


scenarios = {}
t0 = time.time()
for v_target in V_TARGETS:
    for obs_name, obs in OBSTACLE_SCENARIOS.items():
        key = f"{int(v_target)}|{obs_name}"
        print("solving", key)
        log = mpc.simulate_lap(v_target=v_target, obstacle=obs, max_steps=260)
        xs, ys, heads = to_world_series(log["s"], log["ey"])
        scenarios[key] = {
            "t": [round(v, 3) for v in log["t"]],
            "s": [round(v, 3) for v in log["s"]],
            "ey": [round(v, 3) for v in log["ey"]],
            "epsi": [round(v, 4) for v in log["epsi"]],
            "v": [round(v, 3) for v in log["v"]],
            "delta": [round(v, 4) for v in log["delta"]],
            "a": [round(v, 3) for v in log["a"]],
            "x": [round(v, 3) for v in xs],
            "y": [round(v, 3) for v in ys],
            "heading": [round(v, 4) for v in heads],
            "lap_time": round(log["t"][-1], 2),
        }

print(f"solved {len(scenarios)} scenarios in {time.time()-t0:.1f}s")

# track geometry for rendering: centerline + corridor edges + obstacle markers
cx, cy = cl.polyline(0.0, n=600)
lx, ly = cl.polyline(track.TRACK_HALF_WIDTH, n=600)
rx, ry = cl.polyline(-track.TRACK_HALF_WIDTH, n=600)

obstacles_world = {}
for name, obs in OBSTACLE_SCENARIOS.items():
    if obs is None:
        continue
    ox, oy, _ = cl.to_world(obs["s"], obs["ey"])
    obstacles_world[name] = {"x": round(float(ox), 3), "y": round(float(oy), 3), "radius": obs["radius"]}

out = {
    "meta": {
        "v_targets": V_TARGETS,
        "obstacle_names": list(OBSTACLE_SCENARIOS.keys()),
        "lap_length": track.LAP_LENGTH,
        "delta_max": mpc.DELTA_MAX,
        "v_max": mpc.V_MAX,
        "a_max": mpc.A_MAX,
        "a_min": mpc.A_MIN,
        "ey_lim": mpc.EY_LIM,
        "wheelbase": mpc.WHEELBASE,
    },
    "track": {
        "centerline": {"x": [round(v, 3) for v in cx], "y": [round(v, 3) for v in cy]},
        "left_edge": {"x": [round(v, 3) for v in lx], "y": [round(v, 3) for v in ly]},
        "right_edge": {"x": [round(v, 3) for v in rx], "y": [round(v, 3) for v in ry]},
        "obstacles": obstacles_world,
    },
    "scenarios": scenarios,
}

with open("scenarios.json", "w") as f:
    json.dump(out, f, separators=(",", ":"))

import os
print("file size KB:", os.path.getsize("scenarios.json") / 1024)
