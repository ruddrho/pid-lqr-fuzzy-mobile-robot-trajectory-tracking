# Enhanced PID, LQR, and Fuzzy Mobile Robot Navigation

This MATLAB project compares PID, toolbox-free LQR, and toolbox-free Fuzzy Logic controllers on the same enlarged mobile robot, smooth cubic-Bezier route, and fixed obstacle environment.

## Improvements

- Robot body enlarged to 1.20 m × 0.74 m
- Four proportionally enlarged wheels
- Smooth C1-continuous cubic-Bezier centerline
- Broad left/right turns without sharp waypoint corners
- Curvature-aware speed reduction
- Tangent-heading tracking
- Smooth acceleration and angular-rate limits
- Shared APF-style obstacle avoidance
- Exact oriented-body collision guard
- Enlarged three-controller animation panels
- Larger fonts, indicators, status text, and command readouts
- H.264-safe fixed video dimensions

## Run

1. Extract the ZIP.
2. Open the extracted folder as the MATLAB Current Folder.
3. Run:

```matlab
main
```

All output files are saved inside `results/`.

## Required MATLAB version

MATLAB R2019b or newer is recommended. No Control System Toolbox or Fuzzy Logic Toolbox is required.

## Main files

- `main.m`
- `projectConfig.m`
- `projectEnvironment.m`
- `generateReferencePath.m`
- `simulateController.m`
- `pidController.m`
- `lqrController.m`
- `fuzzyController.m`
- `obstacleAvoidance.m`
- `robotCollides.m`
- `drawMobileRobot.m`
- `animateControllers.m`
- `visualizeComparison.m`

## MATLAB structure-assignment compatibility fix

The simulation results are collected in cell arrays and combined only after all controllers finish. This avoids the MATLAB error:

```text
Subscripted assignment between dissimilar structures.
```

The main script also places the project folder at the beginning of the MATLAB search path to prevent older duplicate controller files from being used.

## MATLAB graphics compatibility fix
This release avoids the unsupported `DisplayName` property on `rectangle`
objects, uses an explicit legend handle for obstacles, and replaces `yline`
with a backward-compatible horizontal reference plot.

## Dashboard layout fix
Controller status, speed, turn rate, center error, clearance, and progress are displayed in dedicated panels below the three navigation plots. The information no longer covers the planned path or robot.
