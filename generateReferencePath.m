function path = generateReferencePath(env,cfg)
%GENERATEREFERENCEPATH Generate a smooth, continuous cubic-Bezier route.
% The route contains broad left and right bends, no sharp corners, and is
% arc-length resampled so the path-tracking controllers advance smoothly.

anchors = [
     env.start(1) env.start(2)
     1.60  0.20
     3.20  1.00
     4.80  2.50
     6.40  4.10
     8.20  4.90
    10.00  4.50
    11.50  3.20
    12.80  1.60
    14.20  0.60
    15.80  0.50
    17.40  1.60
    18.90  3.20
    20.50  4.30
    22.10  4.00
    23.50  2.60
     env.goal(1) env.goal(2)
];

rawPath = cubicBezierChain(anchors,cfg.bezierTension,cfg.pathSamplesPerSegment);
rawS = [0; cumsum(vecnorm(diff(rawPath),2,2))];

% Remove any accidental duplicate arc-length samples.
[rawS,uniqueIndex] = unique(rawS,'stable');
rawPath = rawPath(uniqueIndex,:);

path.s = linspace(0,rawS(end),cfg.pathPointCount)';
path.x = interp1(rawS,rawPath(:,1),path.s,'linear');
path.y = interp1(rawS,rawPath(:,2),path.s,'linear');
path.x(1) = env.start(1); path.y(1) = env.start(2);
path.x(end) = env.goal(1); path.y(end) = env.goal(2);

% Tangent heading and curvature.
dx = gradient(path.x,path.s);
dy = gradient(path.y,path.s);
ddx = gradient(dx,path.s);
ddy = gradient(dy,path.s);
path.theta = unwrap(atan2(dy,dx));
denominator = max((dx.^2 + dy.^2).^(3/2),1e-9);
path.curvature = (dx.*ddy - dy.*ddx)./denominator;

% Nearly constant velocity with gentle reduction at tighter curves.
path.vRef = cfg.nominalSpeed./(1 + cfg.curvatureSpeedGain*abs(path.curvature));
path.vRef = min(max(path.vRef,cfg.minimumTurnSpeed),cfg.nominalSpeed);
path.wRef = path.vRef.*path.curvature;

path.waypoints = anchors;
path.goal = env.goal;
path.start = env.start;

% Confirm that the enlarged robot has a safe centerline corridor.
clearance = zeros(size(path.s));
for k = 1:numel(path.s)
    clearance(k) = pointObstacleClearance([path.x(k);path.y(k)],env);
end
path.clearance = clearance;
path.minimumClearance = min(clearance);
requiredClearance = cfg.robot.width/2 + cfg.robot.safetyMargin + cfg.pathSafetyBuffer;
if path.minimumClearance < requiredClearance
    error(['Generated path clearance %.3f m is below the required %.3f m. ' ...
           'Adjust only the path anchors; do not move the obstacles.'], ...
           path.minimumClearance,requiredClearance);
end
end

function curve = cubicBezierChain(points,tension,samplesPerSegment)
%CUBICBEZIERCHAIN Build a C1-continuous chain of cubic Bezier segments.

numberOfPoints = size(points,1);
tangents = zeros(size(points));
tangents(1,:) = tension*(points(2,:)-points(1,:));
tangents(end,:) = tension*(points(end,:)-points(end-1,:));
for i = 2:numberOfPoints-1
    tangents(i,:) = 0.5*tension*(points(i+1,:)-points(i-1,:));
end

curve = zeros(0,2);
for i = 1:numberOfPoints-1
    B0 = points(i,:);
    B1 = points(i,:) + tangents(i,:)/3;
    B2 = points(i+1,:) - tangents(i+1,:)/3;
    B3 = points(i+1,:);

    if i < numberOfPoints-1
        tau = linspace(0,1,samplesPerSegment+1)';
        tau(end) = [];
    else
        tau = linspace(0,1,samplesPerSegment+1)';
    end
    oneMinusTau = 1-tau;
    segment = oneMinusTau.^3.*B0 + ...
        3*oneMinusTau.^2.*tau.*B1 + ...
        3*oneMinusTau.*tau.^2.*B2 + tau.^3.*B3;
    curve = [curve; segment]; %#ok<AGROW>
end
end
