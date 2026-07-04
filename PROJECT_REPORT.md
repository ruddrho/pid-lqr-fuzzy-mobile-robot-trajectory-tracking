# Project Report: Enhanced Comparative Robot Navigation

## Objective

The project compares PID, LQR, and Fuzzy Logic control for autonomous mobile robot trajectory tracking while preserving a common robot model, route, obstacle map, start position, and destination.

## Path design

A chain of cubic Bezier segments creates a C1-continuous route. The path is resampled by arc length, and tangent heading and curvature are calculated numerically. Reference speed stays nearly constant and reduces only where curvature increases.

## Enlarged robot

The robot body is 1.20 m long and 0.74 m wide. Four enlarged wheels, a heading indicator, center marker, and front marker are rendered. The collision model uses the same enlarged dimensions plus a safety margin.

## Controllers

- PID: lateral and heading PID feedback with filtered derivative terms and anti-windup.
- LQR: discrete Frenet-frame LQR using an internal iterative Riccati solver.
- Fuzzy: toolbox-free Sugeno-style steering correction using lateral and heading error memberships.

## Safety

All controllers share a low-authority artificial-potential-field safety layer. The reference path is validated against obstacle clearance before simulation. An oriented-body collision guard prevents translation into circular or rectangular obstacles.

## Visualization

The animation contains three enlarged controller panels with large titles, fonts, status indicators, speed, turn rate, centerline error, obstacle clearance, and route progress.

## Outputs

- Controller metrics CSV
- Trajectory comparison PNG
- Tracking-error and command PNG
- Metric comparison PNG
- Final animation frame PNG
- Controller comparison MP4
- Complete MAT results file
