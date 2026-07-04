function ref = nearestPathState(pose,path,previousIndex,searchWindow)
%NEARESTPATHSTATE Find the nearest forward path state without shortcuts.
% Search begins at the previous progress index, so progress cannot jump
% backward or skip directly to a distant section of the path.

N = numel(path.s);
firstIndex = min(max(previousIndex,1),N);
lastIndex = min(firstIndex + searchWindow,N);
candidate = firstIndex:lastIndex;

d2 = (path.x(candidate)-pose(1)).^2 + (path.y(candidate)-pose(2)).^2;
[~,localIndex] = min(d2);
index = candidate(localIndex);

xRef = path.x(index);
yRef = path.y(index);
thetaRef = path.theta(index);
dx = pose(1)-xRef;
dy = pose(2)-yRef;

% Frenet-frame tracking errors.
ref.lateralError = -sin(thetaRef)*dx + cos(thetaRef)*dy;
ref.longitudinalError = cos(thetaRef)*(xRef-pose(1)) + ...
    sin(thetaRef)*(yRef-pose(2));
ref.headingError = wrapToPiLocal(pose(3)-thetaRef);

ref.index = index;
ref.x = xRef;
ref.y = yRef;
ref.theta = thetaRef;
ref.curvature = path.curvature(index);
ref.v = path.vRef(index);
ref.w = path.wRef(index);
ref.s = path.s(index);
ref.remainingDistance = path.s(end)-path.s(index);
ref.progress = path.s(index)/max(path.s(end),eps);
end
