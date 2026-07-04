# Algorithms Summary

## Cubic-Bezier route

Adjacent cubic Bezier segments use shared endpoint tangents, producing a continuous and natural path without angular waypoint jumps.

## Centerline tracking

Each controller receives Frenet-frame lateral error, longitudinal error, tangent-heading error, reference curvature, and reference speed from the nearest forward path sample.

## Smooth movement

Linear and angular commands pass through acceleration/rate limits. Curvature-aware speed control slightly reduces speed during turns while maintaining near-constant speed on gentle sections.

## Obstacle avoidance

A shared APF-style correction activates only near obstacles. Because the planned path already has safe clearance, the avoidance layer remains low authority and does not unnecessarily pull the robot away from the path.

## Collision guard

The enlarged oriented rectangular robot body is checked against circles and axis-aligned rectangles. A predicted collision stops translation and commands a smooth safety rotation.
