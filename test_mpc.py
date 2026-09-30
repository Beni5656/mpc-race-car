"""Quick sanity checks for the racing MPC controller."""
import numpy as np
import track
import mpc


def test_completes_a_lap_within_track_limits():
    log = mpc.simulate_lap(v_target=14.0, max_steps=200)
    assert log["s"][-1] >= 0.98 * track.LAP_LENGTH
    assert max(abs(e) for e in log["ey"]) <= mpc.EY_LIM + 1e-2


def test_respects_actuator_limits():
    log = mpc.simulate_lap(v_target=14.0, max_steps=120)
    assert max(abs(d) for d in log["delta"]) <= mpc.DELTA_MAX + 1e-3
    assert all(mpc.A_MIN - 1e-3 <= a <= mpc.A_MAX + 1e-3 for a in log["a"])


def test_higher_target_speed_completes_lap_faster():
    slow = mpc.simulate_lap(v_target=10.0, max_steps=250)
    fast = mpc.simulate_lap(v_target=18.0, max_steps=250)
    assert fast["t"][-1] < slow["t"][-1]


if __name__ == "__main__":
    test_completes_a_lap_within_track_limits()
    test_respects_actuator_limits()
    test_higher_target_speed_completes_lap_faster()
    print("all tests passed")
