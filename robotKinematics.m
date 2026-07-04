function poseNext = robotKinematics(pose,v,w,dt,lateralDisturbance,yawDisturbance)
%ROBOTKINEMATICS Differential-drive/unicycle kinematic update.

theta = pose(3);
xdot = v*cos(theta) - lateralDisturbance*sin(theta);
ydot = v*sin(theta) + lateralDisturbance*cos(theta);
thetadot = w + yawDisturbance;

poseNext = pose + dt*[xdot;ydot;thetadot];
poseNext(3) = wrapToPiLocal(poseNext(3));
end
