function metrics = calculateMetrics(result,path,cfg)
%CALCULATEMETRICS Objective tracking, safety, and command indicators.

cte = result.crossTrackError;
heading = result.headingError;

metrics.Controller = string(result.name);
metrics.GoalReached = result.goalReached;
metrics.CollisionDetected = result.collisionDetected;
metrics.CollisionGuardActivations = result.collisionGuardActivations;
metrics.RMSE_CrossTrack_m = sqrt(mean(cte.^2));
metrics.MAE_CrossTrack_m = mean(abs(cte));
metrics.MaxCrossTrack_m = max(abs(cte));
metrics.RMSE_Heading_deg = sqrt(mean(heading.^2))*180/pi;
metrics.FinalPositionError_m = norm(result.finalPose(1:2)-path.goal);
metrics.MinimumClearance_m = min(result.clearance);
metrics.RecoveryTime_s = recoveryTime(result.time,cte,heading,cfg.dt);
metrics.ControlEffort = trapz(result.time,result.v.^2 + 0.25*result.w.^2);
metrics.CompletionTime_s = result.time(end);
metrics.TravelDistance_m = sum(vecnorm(diff(result.pose(:,1:2)),2,2));
end

function value = recoveryTime(time,cte,heading,dt)
holdSteps = max(1,round(3/dt));
value = NaN;
for k = 1:max(1,numel(time)-holdSteps+1)
    stopIndex = min(k+holdSteps-1,numel(time));
    if all(abs(cte(k:stopIndex)) < 0.08) && ...
       all(abs(heading(k:stopIndex)) < 4*pi/180)
        value = time(k);
        return;
    end
end
end
