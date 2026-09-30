"""
Oval race track defined by curvature as a function of arc length, kappa(s).

Two straights connected by two constant-curvature turns (both the same
rotational sense, like a running track), with the curvature smoothed across
each transition using a tanh blend -- i.e. a short clothoid-like transition
curve, the same device real tracks use so a car doesn't have to snap its
steering angle instantly at a corner entry.

kappa(s) is expressed purely with CasADi-compatible ops so it can be dropped
directly into a symbolic MPC dynamics model. Converting a simulated
(s, e_y) trajectory back to world (x, y) coordinates for plotting is handled
separately, by numerically integrating heading(s) = integral of kappa once,
offline, in plain numpy.
"""
import numpy as np
import casadi as ca

STRAIGHT_LEN = 40.0     # m
TURN_RADIUS = 15.0      # m
TRANSITION_W = 3.0      # m, width of the smoothed curvature transition
TRACK_HALF_WIDTH = 4.0  # m, drivable corridor half-width from centerline

S1 = STRAIGHT_LEN                          # end of bottom straight
S2 = S1 + np.pi * TURN_RADIUS              # end of right-hand turn
S3 = S2 + STRAIGHT_LEN                     # end of top straight
S4 = S3 + np.pi * TURN_RADIUS              # end of left-hand turn = lap length
LAP_LENGTH = S4


def _smooth_pulse(s, a, b, w):
    """~1 for s in [a+w, b-w], smoothly to 0 outside, transition width w."""
    return 0.5 * (ca.tanh((s - a) / w) - ca.tanh((s - b) / w))


def kappa(s):
    """Signed curvature at arc length s (CasADi-compatible, periodic)."""
    s_mod = ca.fmod(s, LAP_LENGTH) + LAP_LENGTH
    s_mod = ca.fmod(s_mod, LAP_LENGTH)
    turn1 = _smooth_pulse(s_mod, S1, S2, TRANSITION_W)
    turn2 = _smooth_pulse(s_mod, S3, S4, TRANSITION_W)
    return (turn1 + turn2) / TURN_RADIUS


def build_centerline_table(ds=0.05):
    """Numerically integrate heading(s) once to get a lookup table for
    converting (s, e_y) Frenet coordinates back to world (x, y, heading)."""
    s_sym = ca.MX.sym('s')
    kappa_fn = ca.Function('kappa_num', [s_sym], [kappa(s_sym)])

    n = int(np.ceil(LAP_LENGTH / ds)) + 1
    s_grid = np.linspace(0, LAP_LENGTH, n)
    heading = np.zeros(n)
    x = np.zeros(n)
    y = np.zeros(n)
    for i in range(1, n):
        k = float(kappa_fn(s_grid[i - 1]))
        heading[i] = heading[i - 1] + k * ds
        x[i] = x[i - 1] + np.cos(heading[i - 1]) * ds
        y[i] = y[i - 1] + np.sin(heading[i - 1]) * ds
    return {"s": s_grid, "x": x, "y": y, "heading": heading}


class Centerline:
    def __init__(self):
        t = build_centerline_table()
        self.s, self.x, self.y, self.heading = t["s"], t["x"], t["y"], t["heading"]

    def position(self, s):
        s_mod = np.mod(s, LAP_LENGTH)
        x = np.interp(s_mod, self.s, self.x)
        y = np.interp(s_mod, self.s, self.y)
        # unwrap heading interpolation across the 0/2pi seam by interpolating
        # sin/cos separately
        c = np.interp(s_mod, self.s, np.cos(self.heading))
        si = np.interp(s_mod, self.s, np.sin(self.heading))
        heading = np.arctan2(si, c)
        return x, y, heading

    def to_world(self, s, ey):
        """Convert Frenet (s, e_y) to world (x, y), offsetting along the
        left-hand normal of the centerline heading."""
        x, y, heading = self.position(s)
        nx, ny = -np.sin(heading), np.cos(heading)
        return x + ey * nx, y + ey * ny, heading

    def polyline(self, offset=0.0, n=800):
        s = np.linspace(0, LAP_LENGTH, n)
        xs, ys = [], []
        for si in s:
            x, y, _ = self.to_world(si, offset)
            xs.append(x); ys.append(y)
        return np.array(xs), np.array(ys)
