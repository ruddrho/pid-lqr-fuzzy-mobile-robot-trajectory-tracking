function [vOut,wOut,avoidance] = obstacleAvoidance(pose,vIn,wIn,env,cfg)
%OBSTACLEAVOIDANCE Smooth APF-style correction shared by all controllers.
% The planned path already provides safe clearance, so this module acts as
% a gentle safety layer rather than pulling the robot away from centerline.

px = pose(1); py = pose(2); theta = pose(3);
repulsion = [0;0];
minimumClearance = inf;
influence = cfg.avoidanceInfluenceDistance;

for i = 1:size(env.circleObstacles,1)
    c = env.circleObstacles(i,:);
    fromObstacle = [px-c(1);py-c(2)];
    centerDistance = norm(fromObstacle);
    if centerDistance < 1e-9
        away = [1;0];
    else
        away = fromObstacle/centerDistance;
    end
    clearance = centerDistance-c(3);
    minimumClearance = min(minimumClearance,clearance);
    repulsion = repulsion + repulsiveContribution( ...
        away,clearance,theta,influence);
end

for i = 1:size(env.rectangleObstacles,1)
    r = env.rectangleObstacles(i,:);
    qx = min(max(px,r(1)),r(1)+r(3));
    qy = min(max(py,r(2)),r(2)+r(4));
    fromObstacle = [px-qx;py-qy];
    clearance = norm(fromObstacle);

    if clearance < 1e-9
        rectangleCenter = [r(1)+r(3)/2;r(2)+r(4)/2];
        fromObstacle = [px;py]-rectangleCenter;
        if norm(fromObstacle) < 1e-9
            fromObstacle = [1;0];
        end
        away = fromObstacle/norm(fromObstacle);
        clearance = -min([px-r(1),r(1)+r(3)-px, ...
                          py-r(2),r(2)+r(4)-py]);
    else
        away = fromObstacle/clearance;
    end

    minimumClearance = min(minimumClearance,clearance);
    repulsion = repulsion + repulsiveContribution( ...
        away,clearance,theta,influence);
end

if isinf(minimumClearance)
    minimumClearance = realmax;
end

danger = min(max((influence-minimumClearance)/ ...
    max(influence-cfg.emergencyClearance,eps),0),1);

if norm(repulsion) > 1e-10
    repulsiveAngle = wrapToPiLocal(atan2(repulsion(2),repulsion(1))-theta);
else
    repulsiveAngle = 0;
end

wOut = wIn + cfg.avoidanceSteeringGain*danger*repulsiveAngle;
vOut = vIn*max(cfg.minimumAvoidanceSpeedScale,1-0.45*danger);

vOut = min(max(vOut,0),cfg.maxLinearVelocity);
wOut = min(max(wOut,-cfg.maxAngularVelocity),cfg.maxAngularVelocity);

avoidance.minimumClearance = minimumClearance;
avoidance.danger = danger;
avoidance.active = danger > 0;
avoidance.repulsiveAngle = repulsiveAngle;
end

function contribution = repulsiveContribution(away,clearance,theta,influence)
if clearance >= influence
    contribution = [0;0];
    return;
end

% Reduce the effect of obstacles that are behind the robot.
obstacleDirection = -away;
relativeObstacleAngle = wrapToPiLocal( ...
    atan2(obstacleDirection(2),obstacleDirection(1))-theta);
frontFactor = 0.15 + 0.85*max(cos(relativeObstacleAngle),0)^2;
weight = frontFactor*(1/max(clearance,0.05)-1/influence);
contribution = weight*away;
end
