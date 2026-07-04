function env = projectEnvironment()
%PROJECTENVIRONMENT Fixed start, goal, and obstacle arrangement.
% Start and goal are preserved from the original comparison project.

env.start = [0.0; 0.0];
env.goal = [24.6; 0.8];

% Circular obstacles: [centerX centerY radius]
env.circleObstacles = [
     3.60  3.00  0.45
     6.40  2.40  0.50
     8.90  3.25  0.48
    11.20  1.65  0.46
    14.30  2.00  0.50
    17.10  3.20  0.48
    20.30  2.55  0.52
    23.00  1.05  0.42
];

% Rectangular obstacles: [x y width height]
env.rectangleObstacles = [
     1.30  1.60  0.90  0.55
     5.20  5.35  1.00  0.55
    10.10  5.35  1.00  0.55
    12.70  3.55  0.90  0.60
    15.20  3.10  0.95  0.60
    18.00  0.15  1.00  0.55
    21.40  5.15  0.90  0.55
];
end
