function result = simulateController(controllerName,path,env,cfg)
%SIMULATECONTROLLER Run one controller with smooth actuation and avoidance.

maxSteps = ceil(cfg.maxTime/cfg.dt)+1;
pose = [path.x(1);path.y(1);path.theta(1)];
pose(1:2) = pose(1:2) + cfg.initialLateralOffset* ...
    [-sin(path.theta(1));cos(path.theta(1))];
pose(3) = wrapToPiLocal(pose(3)+cfg.initialHeadingOffset);

result.time = zeros(maxSteps,1);
result.pose = zeros(maxSteps,3);
result.referenceIndex = zeros(maxSteps,1);
result.crossTrackError = zeros(maxSteps,1);
result.headingError = zeros(maxSteps,1);
result.longitudinalError = zeros(maxSteps,1);
result.v = zeros(maxSteps,1);
result.w = zeros(maxSteps,1);
result.clearance = zeros(maxSteps,1);
result.avoidanceDanger = zeros(maxSteps,1);

controllerState = [];
progressIndex = 1;
previousV = 0;
previousW = 0;
goalReached = false;
collisionDetected = robotCollides(pose,env,cfg);
collisionGuardActivations = 0;
lastStep = 1;

for k = 1:maxSteps
    t = (k-1)*cfg.dt;
    ref = nearestPathState(pose,path,progressIndex,cfg.searchWindow);
    progressIndex = max(progressIndex,ref.index);

    switch upper(controllerName)
        case 'PID'
            [rawV,rawW,controllerState] = pidController(ref,controllerState,cfg);
        case 'LQR'
            [rawV,rawW,controllerState] = lqrController(ref,controllerState,cfg);
        case 'FUZZY'
            [rawV,rawW,controllerState] = fuzzyController(ref,controllerState,cfg);
        otherwise
            error('Unknown controller: %s',controllerName);
    end

    distanceToGoal = norm(pose(1:2)-path.goal);
    if ref.remainingDistance < 1.50
        rawV = min(rawV,0.55*distanceToGoal+0.04);
    end

    [safeV,safeW,avoidance] = obstacleAvoidance( ...
        pose,rawV,rawW,env,cfg);

    % Rate limits make rotation and speed changes physically smooth.
    v = rateLimit(safeV,previousV,cfg.maxLinearAcceleration*cfg.dt);
    w = rateLimit(safeW,previousW,cfg.maxAngularAcceleration*cfg.dt);

    result.time(k) = t;
    result.pose(k,:) = pose';
    result.referenceIndex(k) = ref.index;
    result.crossTrackError(k) = ref.lateralError;
    result.headingError(k) = ref.headingError;
    result.longitudinalError(k) = ref.longitudinalError;
    result.v(k) = v;
    result.w(k) = w;
    result.clearance(k) = avoidance.minimumClearance;
    result.avoidanceDanger(k) = avoidance.danger;

    if cfg.enableDisturbance
        lateralDisturbance = cfg.lateralDisturbance1*sin(0.55*t) + ...
            cfg.lateralDisturbance2*sin(1.70*t);
        yawDisturbance = cfg.yawDisturbance*sin(0.80*t);
    else
        lateralDisturbance = 0;
        yawDisturbance = 0;
    end

    candidatePose = robotKinematics(pose,v,w,cfg.dt, ...
        lateralDisturbance,yawDisturbance);

    % Final collision guard. Translation is stopped before contact and the
    % robot rotates smoothly toward the path tangent / free direction.
    if robotCollides(candidatePose,env,cfg)
        collisionGuardActivations = collisionGuardActivations + 1;
        turnDirection = sign(-ref.lateralError-ref.headingError);
        if turnDirection == 0
            turnDirection = sign(w);
        end
        if turnDirection == 0
            turnDirection = 1;
        end
        v = 0;
        w = rateLimit(turnDirection*cfg.collisionTurnRate, ...
            previousW,cfg.maxAngularAcceleration*cfg.dt);
        candidatePose = robotKinematics(pose,0,w,cfg.dt,0,0);
        if robotCollides(candidatePose,env,cfg)
            candidatePose = pose;
            w = 0;
        end
        if robotCollides(candidatePose,env,cfg)
            collisionDetected = true;
        end
        result.v(k) = v;
        result.w(k) = w;
    end

    pose = candidatePose;
    previousV = v;
    previousW = w;
    lastStep = k;

    if progressIndex >= numel(path.s)-8 && ...
            norm(pose(1:2)-path.goal) < cfg.goalTolerance
        goalReached = true;
        break;
    end
end

fields = {'time','pose','referenceIndex','crossTrackError','headingError', ...
          'longitudinalError','v','w','clearance','avoidanceDanger'};
for i = 1:numel(fields)
    value = result.(fields{i});
    result.(fields{i}) = value(1:lastStep,:);
end

result.name = char(controllerName);
result.goalReached = goalReached;
result.collisionDetected = collisionDetected;
result.collisionGuardActivations = collisionGuardActivations;
result.finalPose = pose;
end

function limited = rateLimit(command,previousCommand,maximumStep)
limited = previousCommand + min(max(command-previousCommand, ...
    -maximumStep),maximumStep);
end
