function animateControllers(path,env,results,cfg)
%ANIMATECONTROLLERS Three-panel animation with separate information panels.
% Controller information is intentionally placed below each navigation
% axes so that status text never covers the reference path or robot.
%
% This implementation uses ordinary axes rather than tiledlayout/uigridlayout
% so it remains compatible with older MATLAB releases.

colors = lines(numel(results));
numberOfControllers = numel(results);

% Give the separated track and information regions enough vertical room.
figurePosition = cfg.ui.figurePosition;
figurePosition(4) = max(figurePosition(4),860);
fig = figure('Color','w','Position',figurePosition, ...
    'Name','Enhanced PID vs LQR vs Fuzzy Navigation');

% Manual normalized layout: one navigation axes and one information axes
% for each controller. This prevents all dashboard text from overlapping
% the route, obstacles, or moving robot.
outerLeft = 0.025;
outerRight = 0.018;
panelGap = 0.025;
panelWidth = (1-outerLeft-outerRight-(numberOfControllers-1)*panelGap) ...
    / numberOfControllers;
trackBottom = 0.39;
trackHeight = 0.53;
infoBottom = 0.055;
infoHeight = 0.255;

ax = gobjects(1,numberOfControllers);
infoAx = gobjects(1,numberOfControllers);
trail = gobjects(1,numberOfControllers);
statusIndicator = gobjects(1,numberOfControllers);
statusText = gobjects(1,numberOfControllers);
progressBackground = gobjects(1,numberOfControllers);
progressForeground = gobjects(1,numberOfControllers);
robot = cell(1,numberOfControllers);
metricText = cell(numberOfControllers,5);

for i = 1:numberOfControllers
    left = outerLeft + (i-1)*(panelWidth+panelGap);

    %% Navigation area
    ax(i) = axes('Parent',fig,'Units','normalized', ...
        'Position',[left trackBottom panelWidth trackHeight]);
    hold(ax(i),'on');
    grid(ax(i),'on');
    axis(ax(i),'equal');
    axis(ax(i),cfg.worldLimits);

    drawObstacles(ax(i),env);
    plot(ax(i),path.x,path.y,'k--','LineWidth',cfg.ui.pathLineWidth);
    plot(ax(i),env.start(1),env.start(2),'gs', ...
        'MarkerFaceColor','g','MarkerSize',10);
    plot(ax(i),env.goal(1),env.goal(2),'rp', ...
        'MarkerFaceColor','r','MarkerSize',14);

    trail(i) = plot(ax(i),NaN,NaN,'LineWidth',cfg.ui.lineWidth, ...
        'Color',colors(i,:));
    robot{i} = drawMobileRobot(ax(i),results(i).pose(1,:)', ...
        colors(i,:),[],cfg);

    title(ax(i),sprintf('%s CONTROLLER',upper(results(i).name)), ...
        'FontSize',cfg.ui.titleFontSize,'FontWeight','bold');
    xlabel(ax(i),'X (m)','FontSize',cfg.ui.axisFontSize);
    ylabel(ax(i),'Y (m)','FontSize',cfg.ui.axisFontSize);
    set(ax(i),'FontSize',cfg.ui.axisFontSize,'LineWidth',1.2, ...
        'Box','on');

    %% Dedicated information area below the navigation track
    infoAx(i) = axes('Parent',fig,'Units','normalized', ...
        'Position',[left infoBottom panelWidth infoHeight]);
    hold(infoAx(i),'on');
    axis(infoAx(i),[0 1 0 1]);
    axis(infoAx(i),'off');

    % Readable dashboard background and border.
    rectangle(infoAx(i),'Position',[0.01 0.02 0.98 0.95], ...
        'FaceColor',[0.965 0.975 0.990], ...
        'EdgeColor',[0.50 0.55 0.64],'LineWidth',1.2);

    statusIndicator(i) = plot(infoAx(i),0.055,0.82,'o', ...
        'MarkerSize',cfg.ui.indicatorSize, ...
        'MarkerFaceColor',[1.0 0.72 0.10], ...
        'MarkerEdgeColor','k','LineWidth',1.0);
    statusText(i) = text(infoAx(i),0.105,0.82,'TRACKING', ...
        'FontSize',cfg.ui.infoFontSize+1,'FontWeight','bold', ...
        'VerticalAlignment','middle','Interpreter','none');

    metricText{i,1} = text(infoAx(i),0.045,0.58,'Speed: -- m/s', ...
        'FontSize',cfg.ui.infoFontSize,'FontWeight','bold');
    metricText{i,2} = text(infoAx(i),0.045,0.38,'Turn rate: -- rad/s', ...
        'FontSize',cfg.ui.infoFontSize,'FontWeight','bold');
    metricText{i,3} = text(infoAx(i),0.51,0.58,'Center error: -- m', ...
        'FontSize',cfg.ui.infoFontSize,'FontWeight','bold');
    metricText{i,4} = text(infoAx(i),0.51,0.38,'Clearance: -- m', ...
        'FontSize',cfg.ui.infoFontSize,'FontWeight','bold');
    metricText{i,5} = text(infoAx(i),0.045,0.17,'Progress: 0%%', ...
        'FontSize',cfg.ui.infoFontSize,'FontWeight','bold');

    % Progress bar displayed in the information panel, not over the path.
    progressBackground(i) = plot(infoAx(i),[0.32 0.94],[0.20 0.20], ...
        '-','Color',[0.78 0.80 0.84],'LineWidth',8);
    progressForeground(i) = plot(infoAx(i),[0.32 0.32],[0.20 0.20], ...
        '-','Color',colors(i,:),'LineWidth',8);
end

videoOpen = false;
if cfg.makeVideo
    try
        videoObject = VideoWriter( ...
            fullfile(cfg.outputDir,'controller_comparison.mp4'),'MPEG-4');
        videoObject.FrameRate = cfg.videoFrameRate;
        open(videoObject);
        videoOpen = true;
    catch videoError
        warning('Video recording disabled: %s',videoError.message);
    end
end

targetSize = [];
maximumLength = max(arrayfun(@(r)size(r.pose,1),results));
for k = 1:cfg.animationStride:maximumLength
    for i = 1:numberOfControllers
        currentIndex = min(k,size(results(i).pose,1));

        set(trail(i),'XData',results(i).pose(1:currentIndex,1), ...
            'YData',results(i).pose(1:currentIndex,2));
        robot{i} = drawMobileRobot(ax(i), ...
            results(i).pose(currentIndex,:)',colors(i,:),robot{i},cfg);

        vNow = results(i).v(currentIndex);
        wNow = results(i).w(currentIndex);
        cteNow = results(i).crossTrackError(currentIndex);
        clearNow = results(i).clearance(currentIndex);
        progressNow = 100*results(i).referenceIndex(currentIndex)/numel(path.s);
        progressNow = min(max(progressNow,0),100);

        set(metricText{i,1},'String',sprintf('Speed: %.2f m/s',vNow));
        set(metricText{i,2},'String',sprintf('Turn rate: %.2f rad/s',wNow));
        set(metricText{i,3},'String',sprintf('Center error: %.3f m',cteNow));
        set(metricText{i,4},'String',sprintf('Clearance: %.2f m',clearNow));
        set(metricText{i,5},'String',sprintf('Progress: %.0f%%',progressNow));

        progressEnd = 0.32 + (0.94-0.32)*(progressNow/100);
        set(progressForeground(i),'XData',[0.32 progressEnd]);

        if currentIndex >= size(results(i).pose,1) && results(i).goalReached
            set(statusIndicator(i),'MarkerFaceColor',[0.15 0.80 0.25]);
            set(statusText(i),'String','GOAL REACHED');
        elseif results(i).avoidanceDanger(currentIndex) > 0.20
            set(statusIndicator(i),'MarkerFaceColor',[1.0 0.35 0.15]);
            set(statusText(i),'String','AVOIDING');
        else
            set(statusIndicator(i),'MarkerFaceColor',[1.0 0.72 0.10]);
            set(statusText(i),'String','TRACKING');
        end
    end

    drawnow;
    if videoOpen
        [videoFrame,targetSize] = prepareVideoFrame(fig,targetSize);
        writeVideo(videoObject,videoFrame);
    end
end

if videoOpen
    close(videoObject);
end
saveFigurePNG(fig,fullfile(cfg.outputDir,'final_animation_frame.png'));
end
