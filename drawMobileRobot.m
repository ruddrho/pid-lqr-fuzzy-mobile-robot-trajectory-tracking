function handles = drawMobileRobot(ax,pose,color,handles,cfg)
%DRAWMOBILEROBOT Draw/update an enlarged four-wheel mobile robot.

if nargin < 4 || isempty(handles)
    handles = struct();
end

L = cfg.robot.length;
W = cfg.robot.width;
wheelL = cfg.robot.wheelLength;
wheelW = cfg.robot.wheelWidth;
wheelOffset = cfg.robot.wheelOffset;

R = [cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
bodyLocal = [ L/2  W/2; L/2 -W/2; -L/2 -W/2; -L/2 W/2]';
bodyWorld = R*bodyLocal + pose(1:2);

wheelCenters = [
     0.28*L  W/2+wheelOffset
     0.28*L -W/2-wheelOffset
    -0.28*L  W/2+wheelOffset
    -0.28*L -W/2-wheelOffset
];
wheelLocal = [ wheelL/2  wheelW/2; wheelL/2 -wheelW/2; ...
              -wheelL/2 -wheelW/2; -wheelL/2 wheelW/2]';

arrowEnd = pose(1:2) + 0.72*L*[cos(pose(3));sin(pose(3))];
noseLocal = [L/2;0];
noseWorld = R*noseLocal + pose(1:2);

if ~isfield(handles,'body') || ~isgraphics(handles.body)
    handles.body = patch(ax,bodyWorld(1,:),bodyWorld(2,:),color, ...
        'EdgeColor','k','LineWidth',1.6,'FaceAlpha',0.94);
    handles.wheels = gobjects(4,1);
    for i = 1:4
        centerLocal = wheelCenters(i,:)';
        wheelWorld = R*(wheelLocal+centerLocal) + pose(1:2);
        handles.wheels(i) = patch(ax,wheelWorld(1,:),wheelWorld(2,:), ...
            [0.08 0.08 0.09],'EdgeColor','k','LineWidth',1.0);
    end
    handles.heading = plot(ax,[pose(1) arrowEnd(1)], ...
        [pose(2) arrowEnd(2)],'k-','LineWidth',2.6);
    handles.center = plot(ax,pose(1),pose(2),'wo','MarkerFaceColor','w', ...
        'MarkerEdgeColor','k','MarkerSize',5);
    handles.nose = plot(ax,noseWorld(1),noseWorld(2),'o', ...
        'MarkerFaceColor',[1 0.25 0.15],'MarkerEdgeColor','k','MarkerSize',6);
else
    set(handles.body,'XData',bodyWorld(1,:),'YData',bodyWorld(2,:));
    for i = 1:4
        centerLocal = wheelCenters(i,:)';
        wheelWorld = R*(wheelLocal+centerLocal) + pose(1:2);
        set(handles.wheels(i),'XData',wheelWorld(1,:), ...
            'YData',wheelWorld(2,:));
    end
    set(handles.heading,'XData',[pose(1) arrowEnd(1)], ...
        'YData',[pose(2) arrowEnd(2)]);
    set(handles.center,'XData',pose(1),'YData',pose(2));
    set(handles.nose,'XData',noseWorld(1),'YData',noseWorld(2));
end
end
