function collision = robotCollides(pose,env,cfg)
%ROBOTCOLLIDES Exact oriented-body checks against circles and rectangles.

hx = cfg.robot.length/2 + cfg.robot.safetyMargin;
hy = cfg.robot.width/2 + cfg.robot.safetyMargin;
theta = pose(3);
u = [cos(theta);sin(theta)];
v = [-sin(theta);cos(theta)];
center = pose(1:2);
collision = false;

% World boundary check using all enlarged body corners.
localCorners = [ hx  hy; hx -hy; -hx -hy; -hx hy]';
worldCorners = [u v]*localCorners + center;
limits = cfg.worldLimits;
if any(worldCorners(1,:) < limits(1)) || any(worldCorners(1,:) > limits(2)) || ...
   any(worldCorners(2,:) < limits(3)) || any(worldCorners(2,:) > limits(4))
    collision = true;
    return;
end

% Oriented rectangle versus circle.
rotationTranspose = [u v]';
for i = 1:size(env.circleObstacles,1)
    c = env.circleObstacles(i,:);
    localCenter = rotationTranspose*([c(1);c(2)]-center);
    closest = [min(max(localCenter(1),-hx),hx); ...
               min(max(localCenter(2),-hy),hy)];
    if norm(localCenter-closest) <= c(3)
        collision = true;
        return;
    end
end

% Oriented rectangle versus axis-aligned rectangle using SAT.
axesToTest = [u v [1;0] [0;1]];
for i = 1:size(env.rectangleObstacles,1)
    r = env.rectangleObstacles(i,:);
    rectangleCenter = [r(1)+r(3)/2;r(2)+r(4)/2];
    centerDifference = rectangleCenter-center;
    separated = false;
    for a = 1:size(axesToTest,2)
        axisVector = axesToTest(:,a);
        robotRadius = hx*abs(dot(u,axisVector)) + ...
            hy*abs(dot(v,axisVector));
        rectangleRadius = r(3)/2*abs(axisVector(1)) + ...
            r(4)/2*abs(axisVector(2));
        if abs(dot(centerDifference,axisVector)) > ...
                robotRadius + rectangleRadius
            separated = true;
            break;
        end
    end
    if ~separated
        collision = true;
        return;
    end
end
end
