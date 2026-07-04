function cfg = projectConfig()
%PROJECTCONFIG Central settings for the enhanced comparison project.

%% Simulation
cfg.dt = 0.03;
cfg.maxTime = 90;
cfg.maxLinearVelocity = 0.90;
cfg.maxAngularVelocity = 2.40;
cfg.maxLinearAcceleration = 0.75;
cfg.maxAngularAcceleration = 3.20;
cfg.goalTolerance = 0.12;
cfg.searchWindow = 180;

% Start exactly on the centerline and tangent to the path.
cfg.initialLateralOffset = 0.0;
cfg.initialHeadingOffset = 0.0;

% Disturbances may be enabled for robustness tests. They are disabled by
% default so the robot remains centered on the requested reference path.
cfg.enableDisturbance = false;
cfg.lateralDisturbance1 = 0.010;
cfg.lateralDisturbance2 = 0.006;
cfg.yawDisturbance = 0.012;

%% Enlarged robot geometry (approximately 2.2x the original visual body)
cfg.robot.length = 1.20;
cfg.robot.width = 0.74;
cfg.robot.wheelLength = 0.36;
cfg.robot.wheelWidth = 0.13;
cfg.robot.wheelOffset = 0.075;
cfg.robot.safetyMargin = 0.10;

%% Smooth path generation
cfg.pathSamplesPerSegment = 100;
cfg.pathPointCount = 1800;
cfg.bezierTension = 0.65;
cfg.nominalSpeed = 0.78;
cfg.minimumTurnSpeed = 0.38;
cfg.curvatureSpeedGain = 2.40;
cfg.pathSafetyBuffer = 0.15;

%% Obstacle avoidance and collision protection
cfg.avoidanceInfluenceDistance = 0.95;
cfg.emergencyClearance = 0.42;
cfg.avoidanceSteeringGain = 1.10;
cfg.minimumAvoidanceSpeedScale = 0.35;
cfg.collisionTurnRate = 1.20;

%% World and graphics
cfg.worldLimits = [-0.8 25.4 -0.8 6.2];
cfg.ui.figurePosition = [20 40 1900 760];
cfg.ui.axisFontSize = 14;
cfg.ui.titleFontSize = 19;
cfg.ui.infoFontSize = 13;
cfg.ui.lineWidth = 2.6;
cfg.ui.pathLineWidth = 3.0;
cfg.ui.indicatorSize = 14;

%% Output options
cfg.outputDir = 'results';
cfg.makeVideo = true;
cfg.animationStride = 5;
cfg.videoFrameRate = 20;
end
