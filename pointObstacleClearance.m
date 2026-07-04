function minimumClearance = pointObstacleClearance(point,env)
%POINTOBSTACLECLEARANCE Signed center-to-obstacle boundary clearance.

px = point(1); py = point(2);
minimumClearance = inf;

for i = 1:size(env.circleObstacles,1)
    c = env.circleObstacles(i,:);
    clearance = hypot(px-c(1),py-c(2))-c(3);
    minimumClearance = min(minimumClearance,clearance);
end

for i = 1:size(env.rectangleObstacles,1)
    r = env.rectangleObstacles(i,:);
    qx = min(max(px,r(1)),r(1)+r(3));
    qy = min(max(py,r(2)),r(2)+r(4));
    clearance = hypot(px-qx,py-qy);
    if px >= r(1) && px <= r(1)+r(3) && ...
       py >= r(2) && py <= r(2)+r(4)
        clearance = -min([px-r(1),r(1)+r(3)-px, ...
                          py-r(2),r(2)+r(4)-py]);
    end
    minimumClearance = min(minimumClearance,clearance);
end

if isinf(minimumClearance)
    minimumClearance = realmax;
end
end
