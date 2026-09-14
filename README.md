# Missile Guidance & Intercept Simulator

A Python simulation framework for studying and comparing classical missile guidance
laws — Pure Pursuit (PP), Proportional Navigation (PN), and Augmented Proportional
Navigation (APN) — against a maneuvering 3D target, with a Monte Carlo framework
for statistically evaluating guidance performance under seeker noise.

This is an ongoing personal project built to develop a working understanding of
Guidance, Navigation, and Control (GNC) from first principles, structured the way
an engineering codebase should be rather than as a one-off script.

## Motivation

Most treatments of PN/APN present the guidance laws as closed-form equations.
The goal here was to actually build them — implement the guidance geometry,
integrate pursuer dynamics, and then use Monte Carlo trials to see the textbook
behavior (PP failing against a maneuvering target, PN degrading under noise, APN
outperforming PN by compensating for target acceleration) fall out of the
simulation rather than just quoting it.

## Guidance laws implemented

- **Pure Pursuit (PP)** — commands acceleration directly along the instantaneous
  line-of-sight (LOS) to the target. Simple, but always "chases" where the target
  currently is rather than leading it, which makes it fundamentally unsuited to
  maneuvering or fast targets.
- **Proportional Navigation (PN)** — commands acceleration proportional to the
  LOS rotation rate and closing velocity (`a_c = N · Vc · λ̇`), driving the LOS
  rate toward zero and putting the pursuer on a collision-course trajectory. This
  implementation uses a navigation constant `N = 3`.
- **Augmented Proportional Navigation (APN)** — PN plus a correction term based
  on the target's estimated lateral acceleration, compensating for target
  maneuvers that pure PN can't anticipate.

Guidance law formulations follow the standard treatment given in Zarchan's
*Tactical and Strategic Missile Guidance*.

## Project evolution: V1 → V2

**Version 1** (`Version 1/`) was a single monolithic script written to understand
the guidance laws themselves. Guidance laws are standalone functions operating on
raw NumPy arrays and global variables, the guidance law used per run is a
hardcoded string swapped by hand, and a live matplotlib animation is built
directly into the simulation loop. It runs exactly one trial per execution. This
version did its job: it forced me to get the underlying physics and geometry
right end-to-end before worrying about anything else.

**Version 2** (`Version 2/`) is a rewrite focused on system architecture — making
the codebase modular, parameterized, and readable enough that guidance laws,
trajectories, and simulation configuration can be changed independently without
touching unrelated code, and so that running hundreds/thousands of trials for
statistical analysis is actually straightforward. This is the active version of
the project.

## Repository structure (V2)

```
Version 2/
├── InitialConditions.py      # Central simulation config
├── GuidanceLaws.py           # State dataclass + GuidanceLaw ABC + PP/PN/APN
├── Trajectories.py           # Stateless target trajectory generators
├── Simulator.py               # run_trial(): single end-to-end simulation
├── Monte_Carlo_analysis.py   # Sweeps laws x noise levels x trials
└── Visualize_path.py          # Static 3D plot of a single trial
```

**`InitialConditions.py`** — the single source of truth for simulation
parameters: timestep (`dt = 0.01s`), target speed (1000 m/s), pursuer speed
multiplier (1.7x target speed), navigation constant (`N = 3`), acceleration
saturation limit (`max_gs = 40`), kill zone radius (25 m), and seeker noise
magnitude (`e`). Also defines the default single-run guidance law selection
(`law_name`, `law_params`) used by `Visualize_path.py`.

**`GuidanceLaws.py`** — the core guidance law implementations, structured as an
object-oriented hierarchy:
- `State`: a small dataclass holding position (`r`) and velocity (`v`).
- `GuidanceLaw`: an abstract base class defining the `compute_command(p_state,
  t_state)` interface every guidance law must implement.
- `PurePursuit`, `PropNav`, `AugPropNav`: concrete implementations. `AugPropNav`
  is built via composition — it owns a `PropNav` instance internally and adds
  the target-acceleration correction term on top of it, rather than
  reimplementing the PN math. All three saturate commanded acceleration at
  `max_gs`.

This structure means adding a new guidance law (e.g. a future optimal-control-based
law) is a matter of writing a new subclass, not modifying existing code.

**`Trajectories.py`** — stateless functions that generate a target's position and
velocity history given a time/position array. Currently implements `Helix` (a 3D
spiral maneuver, amplitude ~89 m over a 6000 m wavelength — tuned so the target's
centripetal acceleration is a physically realistic ~10g rather than an earlier,
unrealistic ~559g) and `Straight` (a non-maneuvering baseline).

**`Simulator.py`** — `run_trial(e, law_name, law_params)` runs one complete
simulation: it generates the clean target trajectory, corrupts the seeker's
*position* estimate of the target with uniform noise `±e` (target velocity is
passed to the guidance law uncorrupted), then steps the pursuer forward in time.
At each step, the chosen guidance law computes a commanded acceleration from the
current pursuer/target `State`, which is converted into a new pursuer velocity
*direction* (speed magnitude is preserved — acceleration changes heading, not
speed) and integrated into a new position. The trial ends either at intercept
(pursuer-target distance falls below the kill zone radius) or when the target
reaches the end of its trajectory. Guidance law selection is handled through a
`law_registry` dict that maps a law name to its class and required parameter
keys, so `run_trial` never hardcodes which law it's running — this is what makes
it possible to sweep laws programmatically in the Monte Carlo analysis instead of
editing the script by hand.

**`Monte_Carlo_analysis.py`** — the analytical core of the project. It sweeps
every combination of `law_names = [PP, PN, APN]`, 11 seeker noise levels
(`e` from 0 to 30 m), and `n_trials` repeats per combination, calling
`run_trial` fresh each time. Results are stored in pre-allocated 3D NumPy arrays
(`shape = (n_laws, n_e, n_trials)`) for miss distance, hit/miss, and intercept
time, then reduced to per-law, per-noise-level means — miss distance, hit rate,
and average intercept time.

**`Visualize_path.py`** — runs a single trial with default parameters and plots
the pursuer and target trajectories in 3D for a quick visual sanity check.

## Current results

Monte Carlo trials against the maneuvering (Helix) target, sweeping seeker
position noise from 0 to 30 m:

- **PP** fails to intercept at every noise level, including zero noise. This is
  the expected, physically correct result — PP has no mechanism to lead a
  maneuvering target — not a simulator bug.
- **PN** and **APN** both start near 100% hit rate at zero noise and degrade
  toward 0% as noise increases, as expected: noise corrupts the LOS rate
  computation that both laws depend on.
- **APN outperforms PN** through the middle of the noise sweep, consistent with
  theory — the target-acceleration compensation term gives it more margin
  against a maneuvering target before noise dominates.

## Known limitations / open questions

- Seeker noise is currently applied only to the target's *position*, not
  velocity. Since APN's target-acceleration estimate is derived from the (clean)
  velocity history, this may partially explain APN's noise robustness — it's an
  open question how much of APN's advantage in these results is genuine
  guidance-law robustness vs. an artifact of what the noise model corrupts.
- Noise is time-invariant (constant `±e` for the whole trial) — noise that scales
  with timestep or closing range is a deliberately deferred extension.
- No sensor/state estimation layer yet — the pursuer currently guides directly
  off the (noisy) raw target measurement rather than a filtered estimate.

## Roadmap

- Extended Kalman Filter (EKF) phase: Gaussian measurement noise + state
  estimation feeding the guidance loop, replacing the current raw-noise model.
- Additional target maneuver profiles: a bang-bang/step maneuver (highest
  analytical value for showing APN's advantage over PN in a clean, interpretable
  way) and, longer-term, a reactive/evading target.
- Time-varying noise models.
- C++ port, focused on real-time-style performance and explicit memory/ownership
  design.
- Longer-term: 6DOF dynamics, interactive frontend.

**Note on scope:** only guidance laws and features that are actually implemented
and validated are described above as complete. EKF, 6DOF, and the C++ port are
roadmap items, not current capabilities.

## Requirements & running it

```
pip install numpy matplotlib
```

- Run `Version 2/Monte_Carlo_analysis.py` for the statistical comparison across
  guidance laws and noise levels (prints summary tables to console).
- Run `Version 2/Visualize_path.py` for a single-trial 3D trajectory plot using
  the default configuration in `InitialConditions.py`.
- To change simulation parameters (target trajectory, pursuer speed, guidance
  law constants, kill zone size), edit `Version 2/InitialConditions.py`.

## References

- Zarchan, P. *Tactical and Strategic Missile Guidance*, AIAA.
