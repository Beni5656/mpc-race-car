# MPC Race Car

An interactive simulator for **nonlinear model-predictive control (MPC)**
racing a vehicle around a closed track — the same control technique behind
Berkeley's own **BARC (Berkeley Autonomous Race Car)** platform and MPC Lab.

Unlike a single open-loop trajectory solve, MPC re-solves a short-horizon
optimal control problem at *every* control step using the vehicle's current
state, applies only the first control input, and re-solves again one step
later. That receding-horizon loop is what lets it react to disturbances and
obstacles in real time instead of committing to one plan in advance.

## What it solves

The vehicle dynamics are expressed in **Frenet (path) coordinates** relative
to the track centerline — progress `s`, lateral deviation `e_y`, heading
error `e_psi`, and speed `v` — the same formulation used in real racing-MPC
research (e.g. Liniger, Domahidi & Morari, *"Optimization-based Autonomous
Racing of 1:43 Scale RC Cars"*). Expressing the state relative to the path
turns "stay on track" into a simple bound on `e_y` and "go fast" into a
speed-tracking cost, instead of needing a moving reference trajectory.

At every step, a 15-step (2.25 s) horizon nonlinear program is solved with
CasADi/IPOPT, warm-started from the previous step's shifted solution, subject
to:

- Steering-angle and acceleration/braking limits
- A speed limit
- A track-corridor constraint on lateral deviation
- Optionally, a static obstacle that narrows the free corridor, forcing the
  controller to find a way around it

## Files

- `track.py` — the oval track's curvature as a function of arc length
  (`kappa(s)`), with clothoid-like smoothed transitions between the straights
  and corners, plus the lookup table for converting Frenet `(s, e_y)` back to
  world `(x, y)` coordinates for plotting
- `mpc.py` — the path-following bicycle model and the `RacingMPC` receding-
  horizon controller (`RacingMPC.step`), plus `simulate_lap` which runs the
  closed-loop control loop for a full lap
- `test_mpc.py` — sanity checks (stays on track, respects actuator limits,
  faster target speed finishes sooner)
- `generate.py` — sweeps target speed and obstacle placement, solves each
  full lap, and writes `scenarios.json`
- `data.js` — `scenarios.json` wrapped as a `window.RACE_DATA` global
- `index.html` / `app.js` — the interactive console: a live top-down view of
  the car on the track with the obstacle and corridor boundaries, telemetry,
  and speed/steering strip charts

## Running it

```
pip install -r requirements.txt
python generate.py
python -c "import json; open('data.js','w').write('window.RACE_DATA = ' + open('scenarios.json').read() + ';')"
```

Then open `index.html` in a browser.

## Extending it

- Add tire slip (a dynamic bicycle model) instead of the kinematic one, so
  the controller has to manage understeer/oversteer near the friction limit
- Replace the static obstacle with a moving one and re-plan online
- Add lap-to-lap learning (Rosolia & Borrelli's *Learning MPC*): reuse the
  previous lap's trajectory to build a value function that lets the
  controller push closer to the limit each lap — the actual approach BARC
  uses
