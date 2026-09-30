"""
Nonlinear MPC path-following controller for a kinematic bicycle model,
racing around the oval track defined in track.py.

State is expressed in Frenet (path) coordinates relative to the track
centerline -- x = [s, ey, epsi, v] -- progress along the track, lateral
deviation from centerline, heading error relative to the track tangent, and
speed. This is the same formulation used in real racing-MPC work (e.g.
Liniger, Domahidi & Morari, "Optimization-based Autonomous Racing of 1:43
Scale RC Cars" -- closely related to Berkeley's own BARC platform and MPC
Lab): expressing the dynamics relative to the path turns "stay on track" and
"go as fast as possible" into simple bounds and a linear cost, instead of
needing a moving reference trajectory.

At every control step the controller re-solves a short-horizon optimal
control problem (receding-horizon control) using the vehicle's *current*
state as the initial condition, applies only the first control input, then
re-solves again one step later -- the defining feature of MPC, as opposed to
the single open-loop solve used in the powered-descent-guidance project.
"""
import numpy as np
import casadi as ca
import track

# ---- vehicle parameters ----
WHEELBASE = 2.7       # m
A_MAX = 3.0           # m/s^2, max acceleration
A_MIN = -6.0          # m/s^2, max braking
DELTA_MAX = 0.5       # rad, max steering angle (~28.6 deg)
V_MAX = 22.0          # m/s
CAR_HALF_WIDTH = 0.9  # m

N = 15          # horizon steps
DT = 0.15       # s per step

# corridor half-width available to the car's centerline point
EY_LIM = track.TRACK_HALF_WIDTH - CAR_HALF_WIDTH


def dynamics(state, u):
    s, ey, epsi, v = state[0], state[1], state[2], state[3]
    delta, a = u[0], u[1]
    k = track.kappa(s)
    s_dot = v * ca.cos(epsi) / (1 - ey * k)
    ey_dot = v * ca.sin(epsi)
    epsi_dot = v * ca.tan(delta) / WHEELBASE - k * s_dot
    v_dot = a
    return ca.vertcat(s_dot, ey_dot, epsi_dot, v_dot)


class RacingMPC:
    def __init__(self, v_target, obstacle=None):
        """
        v_target: desired cruising speed (m/s) the cost tries to hold
        obstacle: optional dict {s: float, ey: float, radius: float} that the
                  corridor constraint routes around on the inside (ey side)
        """
        self.v_target = v_target
        self.obstacle = obstacle
        self._build()

    def _build(self):
        opti = ca.Opti()
        X = opti.variable(4, N + 1)
        U = opti.variable(2, N)
        x0 = opti.parameter(4)

        opti.subject_to(X[:, 0] == x0)
        for k in range(N):
            s_k = X[:, k]
            u_k = U[:, k]
            k1 = dynamics(s_k, u_k)
            k2 = dynamics(s_k + DT / 2 * k1, u_k)
            k3 = dynamics(s_k + DT / 2 * k2, u_k)
            k4 = dynamics(s_k + DT * k3, u_k)
            s_next = s_k + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            opti.subject_to(X[:, k + 1] == s_next)

        for k in range(N):
            opti.subject_to(opti.bounded(-DELTA_MAX, U[0, k], DELTA_MAX))
            opti.subject_to(opti.bounded(A_MIN, U[1, k], A_MAX))
        for k in range(N + 1):
            opti.subject_to(opti.bounded(0, X[3, k], V_MAX))
            lo, hi = -EY_LIM, EY_LIM
            if self.obstacle is not None:
                # narrow the free corridor near the obstacle's s-location,
                # forcing the car to pass on the side away from the obstacle
                s_obs, ey_obs, r = self.obstacle["s"], self.obstacle["ey"], self.obstacle["radius"]
                closeness = ca.exp(-((X[0, k] - s_obs) / (2 * r)) ** 2)
                if ey_obs >= 0:
                    hi = hi - closeness * (hi - (ey_obs - r))
                else:
                    lo = lo + closeness * ((ey_obs + r) - lo)
            opti.subject_to(opti.bounded(lo, X[1, k], hi))

        # cost: hold target speed, stay centered, small heading error,
        # penalize control effort and its rate of change
        cost = 0
        for k in range(N + 1):
            cost += 1.0 * (X[3, k] - self.v_target) ** 2
            cost += 4.0 * X[1, k] ** 2
            cost += 2.0 * X[2, k] ** 2
        for k in range(N):
            cost += 0.5 * U[0, k] ** 2
            cost += 0.05 * U[1, k] ** 2
        for k in range(N - 1):
            cost += 8.0 * (U[0, k + 1] - U[0, k]) ** 2
            cost += 0.5 * (U[1, k + 1] - U[1, k]) ** 2
        opti.minimize(cost)

        p_opts = {"expand": True, "print_time": False}
        s_opts = {"print_level": 0, "max_iter": 300, "tol": 1e-4}
        opti.solver("ipopt", p_opts, s_opts)

        self.opti, self.X, self.U, self.x0_param = opti, X, U, x0
        self._warm = None

    def step(self, state):
        self.opti.set_value(self.x0_param, state)
        if self._warm is not None:
            self.opti.set_initial(self.X, self._warm["X"])
            self.opti.set_initial(self.U, self._warm["U"])
        else:
            self.opti.set_initial(self.X, np.tile(np.array(state).reshape(4, 1), N + 1))
        sol = self.opti.solve()
        X_sol = sol.value(self.X)
        U_sol = sol.value(self.U)
        # shift solution one step for next warm start
        self._warm = {
            "X": np.hstack([X_sol[:, 1:], X_sol[:, -1:]]),
            "U": np.hstack([U_sol[:, 1:], U_sol[:, -1:]]),
        }
        return U_sol[:, 0]


def simulate_lap(v_target, obstacle=None, max_steps=500):
    controller = RacingMPC(v_target, obstacle)
    state = np.array([0.0, 0.0, 0.0, v_target * 0.3])
    log = {"t": [], "s": [], "ey": [], "epsi": [], "v": [], "delta": [], "a": []}
    t = 0.0
    for _ in range(max_steps):
        u = controller.step(state)
        log["t"].append(t)
        log["s"].append(float(state[0]))
        log["ey"].append(float(state[1]))
        log["epsi"].append(float(state[2]))
        log["v"].append(float(state[3]))
        log["delta"].append(float(u[0]))
        log["a"].append(float(u[1]))

        k1 = dynamics(ca.DM(state), ca.DM(u))
        k2 = dynamics(ca.DM(state) + DT / 2 * k1, ca.DM(u))
        k3 = dynamics(ca.DM(state) + DT / 2 * k2, ca.DM(u))
        k4 = dynamics(ca.DM(state) + DT * k3, ca.DM(u))
        state = np.array((ca.DM(state) + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)).full()).flatten()
        t += DT

        if state[0] >= track.LAP_LENGTH:
            break
    return log


if __name__ == "__main__":
    import time
    t0 = time.time()
    log = simulate_lap(v_target=14.0, max_steps=200)
    print("steps", len(log["t"]), "elapsed", time.time() - t0, "final s", log["s"][-1])
