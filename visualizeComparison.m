function visualizeComparison(path,env,results,metricsTable,cfg)
%VISUALIZECOMPARISON Create enlarged publication-style comparison figures.
% Uses graphics commands compatible with MATLAB releases where Rectangle
% does not support DisplayName and Legend does not support NumColumns.

names = {results.name};
colors = lines(numel(results));
fontSize = cfg.ui.axisFontSize;

%% Figure 1: trajectories, obstacles, and enlarged final robots
fig1 = figure('Color','w','Position',[60 60 1550 820]);
ax = axes(fig1);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
axis(ax,cfg.worldLimits);
drawObstacles(ax,env);

% Dummy line marker used only for a compatible obstacle legend entry.
hLegend = gobjects(0);
legendLabels = {};
hObstacle = plot(ax,NaN,NaN,'s', ...
    'LineStyle','none','MarkerSize',10, ...
    'MarkerFaceColor',[0.24 0.24 0.28], ...
    'MarkerEdgeColor',[0.08 0.08 0.10]);
hLegend(end+1) = hObstacle;
legendLabels{end+1} = 'Obstacles';

hPath = plot(ax,path.x,path.y,'k--','LineWidth',cfg.ui.pathLineWidth);
hLegend(end+1) = hPath;
legendLabels{end+1} = 'Cubic-Bezier centerline';

for i = 1:numel(results)
    hTrajectory = plot(ax,results(i).pose(:,1),results(i).pose(:,2), ...
        'LineWidth',cfg.ui.lineWidth,'Color',colors(i,:));
    hLegend(end+1) = hTrajectory;
    legendLabels{end+1} = names{i};
    drawMobileRobot(ax,results(i).pose(end,:)',colors(i,:),[],cfg);
end

hStart = plot(ax,env.start(1),env.start(2),'gs', ...
    'MarkerFaceColor','g','MarkerSize',11);
hGoal = plot(ax,env.goal(1),env.goal(2),'rp', ...
    'MarkerFaceColor','r','MarkerSize',15);
hLegend(end+1) = hStart;
legendLabels{end+1} = 'Start';
hLegend(end+1) = hGoal;
legendLabels{end+1} = 'Goal';

xlabel(ax,'X position (m)','FontSize',fontSize+1);
ylabel(ax,'Y position (m)','FontSize',fontSize+1);
title(ax,'Enhanced PID, LQR, and Fuzzy Robot Navigation', ...
    'FontSize',cfg.ui.titleFontSize,'FontWeight','bold');

legendHandle = legend(ax,hLegend,legendLabels,'Location','southoutside');
set(legendHandle,'FontSize',fontSize-1);
set(ax,'FontSize',fontSize,'LineWidth',1.2);
saveFigurePNG(fig1,fullfile(cfg.outputDir,'trajectory_comparison.png'));

%% Figure 2: tracking errors, commands, and safety
fig2 = figure('Color','w','Position',[60 40 1550 980]);

ax1 = subplot(3,2,1); hold(ax1,'on'); grid(ax1,'on');
for i = 1:numel(results)
    plot(ax1,results(i).time,results(i).crossTrackError, ...
        'LineWidth',2.0,'Color',colors(i,:));
end
drawHorizontalReference(ax1,0,'k:',1.2);
xlabel(ax1,'Time (s)'); ylabel(ax1,'Cross-track error (m)');
title(ax1,'Centerline Tracking Error');
legend(ax1,names,'Location','best');
set(ax1,'FontSize',fontSize);

ax2 = subplot(3,2,2); hold(ax2,'on'); grid(ax2,'on');
for i = 1:numel(results)
    plot(ax2,results(i).time,results(i).headingError*180/pi, ...
        'LineWidth',2.0,'Color',colors(i,:));
end
drawHorizontalReference(ax2,0,'k:',1.2);
xlabel(ax2,'Time (s)'); ylabel(ax2,'Heading error (deg)');
title(ax2,'Tangent-Heading Tracking'); set(ax2,'FontSize',fontSize);

ax3 = subplot(3,2,3); hold(ax3,'on'); grid(ax3,'on');
for i = 1:numel(results)
    plot(ax3,results(i).time,results(i).v, ...
        'LineWidth',1.8,'Color',colors(i,:));
end
xlabel(ax3,'Time (s)'); ylabel(ax3,'v (m/s)');
title(ax3,'Smooth Linear Velocity'); set(ax3,'FontSize',fontSize);

ax4 = subplot(3,2,4); hold(ax4,'on'); grid(ax4,'on');
for i = 1:numel(results)
    plot(ax4,results(i).time,results(i).w, ...
        'LineWidth',1.8,'Color',colors(i,:));
end
xlabel(ax4,'Time (s)'); ylabel(ax4,'\omega (rad/s)');
title(ax4,'Smooth Angular Velocity'); set(ax4,'FontSize',fontSize);

ax5 = subplot(3,2,5); hold(ax5,'on'); grid(ax5,'on');
for i = 1:numel(results)
    plot(ax5,results(i).time,results(i).clearance, ...
        'LineWidth',1.8,'Color',colors(i,:));
end
safetyRequirement = cfg.robot.width/2 + cfg.robot.safetyMargin;
drawHorizontalReference(ax5,safetyRequirement,'r--',1.4);
xLimits = xlim(ax5);
text(ax5,xLimits(1)+0.02*(xLimits(2)-xLimits(1)), ...
    safetyRequirement,' Body safety requirement', ...
    'Color','r','VerticalAlignment','bottom','FontSize',fontSize-2);
xlabel(ax5,'Time (s)'); ylabel(ax5,'Clearance (m)');
title(ax5,'Obstacle Clearance'); set(ax5,'FontSize',fontSize);

ax6 = subplot(3,2,6); hold(ax6,'on'); grid(ax6,'on');
for i = 1:numel(results)
    plot(ax6,results(i).time,results(i).avoidanceDanger, ...
        'LineWidth',1.8,'Color',colors(i,:));
end
xlabel(ax6,'Time (s)'); ylabel(ax6,'Danger level');
title(ax6,'Obstacle-Avoidance Activity'); ylim(ax6,[0 1.05]);
set(ax6,'FontSize',fontSize);

saveFigurePNG(fig2,fullfile(cfg.outputDir,'errors_and_commands.png'));

%% Figure 3: key metrics
fig3 = figure('Color','w','Position',[80 80 1450 760]);
ax7 = subplot(1,2,1);
errorData = [metricsTable.RMSE_CrossTrack_m, ...
             metricsTable.MAE_CrossTrack_m, ...
             metricsTable.FinalPositionError_m];
bar(ax7,errorData); grid(ax7,'on');
set(ax7,'XTickLabel',cellstr(metricsTable.Controller), ...
    'FontSize',fontSize);
ylabel(ax7,'Error (m)'); title(ax7,'Position Tracking Metrics');
legend(ax7,{'RMSE','MAE','Final error'}, ...
    'Location','best','FontSize',fontSize-1);

ax8 = subplot(1,2,2);
operationData = [metricsTable.CompletionTime_s, ...
                 metricsTable.ControlEffort, ...
                 metricsTable.MinimumClearance_m];
bar(ax8,operationData); grid(ax8,'on');
set(ax8,'XTickLabel',cellstr(metricsTable.Controller), ...
    'FontSize',fontSize);
title(ax8,'Time, Control Effort, and Clearance');
legend(ax8,{'Completion time','Control effort','Minimum clearance'}, ...
    'Location','best','FontSize',fontSize-1);
saveFigurePNG(fig3,fullfile(cfg.outputDir,'metric_comparison.png'));
end

function drawHorizontalReference(ax,yValue,lineStyle,lineWidth)
%DRAWHORIZONTALREFERENCE Backward-compatible replacement for yline.
xLimits = xlim(ax);
plot(ax,xLimits,[yValue yValue],lineStyle, ...
    'LineWidth',lineWidth,'HandleVisibility','off');
end
