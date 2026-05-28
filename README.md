# Non-Prehensile Cube Pushing with Force-Based MPC

A MuJoCo simulation of a non-prehensile manipulation task: a kinematic spherical pusher steers a free-floating cube along an L-shaped reference path using a force-level Model Predictive Controller (MPC) with friction-cone constraints.

Course project for **EEE 587 – Optimal Control of Dynamic Systems**, Arizona State University, Spring 2025.

## What this does

- Tracks an L-shaped reference (forward 0.2 m → 90° turn → 0.2 m) with a free-floating cube on a friction plane.
- Solves a force-level MPC at each control tick (50-step horizon, `cvxpy` + `clarabel`) over normal and tangential contact forces `(F_n, F_t)`.
- Enforces a friction-cone constraint `|F_t| ≤ μ_p · F_n` and a unilateral push constraint `F_n ≥ 0`.
- Heuristically selects one of six contact sites on the cube's back / left faces based on the current path segment and orientation error, to either push straight or steer.
- Realizes the optimal force as a pusher velocity command on a `mocap` body in MuJoCo, with an inward-velocity correction term proportional to `F_n`.

## Method

The controller uses a linearized kinematic dynamics model

```
x_{k+1} = A_d x_k + B_d u_k,   u_k = [F_n, F_t]
```

where `x = [px, py, θ, vx, vy, ω]` and `B_d` is swapped between two variants — one calibrated for back-face pushes (forward motion) and one for side-face pushes (the turn segment). A Goyal–Mason-style limit-surface matrix is computed from the geometry and friction coefficients and reported as a sanity check on the contact mechanics.

State and terminal costs are quadratic (`Q`, `Q_N`), with the optimization decoupling angular and translational error terms to keep the QP convex. The QP is rebuilt as a parameterized `cvxpy.Problem` once at startup and re-solved every control step.

A site-selection layer picks the contact point each tick:

- If `|θ_err| < 40°`: push the back-center (or side-center) site straight.
- If `|θ_err| ≥ 40°` on the forward segment: switch to a back-left or back-right offset site to apply a corrective moment.

## Project structure

```
mujoco-non-prehensile-mpc/
├── README.md
├── requirements.txt
├── .gitignore
├── cube.xml             # MuJoCo model (cube + pusher + contact sites)
├── mpc_pushing.py       # MPC + simulation loop + plotting
└── docs/
    └── report.pdf       # Full project report
```

## Installation

```bash
git clone https://github.com/Aman-Chandak/mujoco-non-prehensile-mpc.git
cd mujoco-non-prehensile-mpc
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Running

```bash
python mpc_pushing.py
```

A MuJoCo viewer window opens and the cube is pushed along the L-path. After the run (or on viewer close), two plots pop up:

1. Desired vs. actual XY trajectory of the cube.
2. Position-tracking error vs. time.

### Tweakable parameters (top of `mpc_pushing.py`)

| Parameter | Meaning |
|---|---|
| `SIM_DURATION` | Total simulation time in seconds |
| `RELATIVE_PATH_VELOCITY` | Reference speed along the path |
| `FORWARD_DISTANCE`, `TURN_DISTANCE`, `TURN_DIRECTION` | Path geometry |
| `MU_P`, `MU_G` | Pusher–cube and ground friction coefficients |
| `MPC_HORIZON`, `Q`, `R_cost`, `Q_N` | MPC horizon and cost matrices |
| `max_fn` | Upper bound on normal contact force |
| `ORIENTATION_ERROR_THRESHOLD` | Threshold for switching to corrective-site pushing |

## Limitations / honest notes

- The dynamics are **kinematic-linearized**, not the full Newton–Euler rigid-body dynamics. The `FN_VX_GAIN`, `FT_VTH_GAIN`, etc. are hand-tuned gains relating contact force commands to body velocities, calibrated against MuJoCo's response. The computed limit-surface matrix is reported but not directly used inside the MPC; the QP runs on the kinematic model.
- The pusher is a `mocap` body — it is positioned kinematically rather than being a torque-controlled robot arm. The MPC's force output is converted into a velocity command, with the contact-normal direction biased by `F_n`.
- Contact-site selection is a heuristic (segment + orientation-error threshold). Replacing it with a hybrid/MIQP formulation that picks the site inside the optimizer was left as future work — discussed in the report.

## Author

Aman Chandak — M.S. Robotics and Autonomous Systems, Arizona State University.
[Portfolio](https://aman-chandak.github.io/portfolio/) · [LinkedIn](https://www.linkedin.com/in/aman-chandak-9094b5131/)
