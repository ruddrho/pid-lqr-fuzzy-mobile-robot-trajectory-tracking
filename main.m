%% Enhanced PID, LQR, and Fuzzy Mobile Robot Navigation Comparison
% Smooth cubic-Bezier path, enlarged robot, obstacle avoidance,
% centerline tracking, large controller panels, and HD video output.

clear; clc; close all;
clear functions;
rng(1);

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot)
    projectRoot = pwd;
end
addpath(projectRoot,'-begin');
cd(projectRoot);
rehash path;

cfg = projectConfig();
cfg.outputDir = fullfile(projectRoot,'results');
if ~exist(cfg.outputDir,'dir')
    mkdir(cfg.outputDir);
end

env = projectEnvironment();
path = generateReferencePath(env,cfg);
controllerNames = {'PID','LQR','Fuzzy'};

fprintf('\n============================================================\n');
fprintf(' ENHANCED PID, LQR, AND FUZZY ROBOT NAVIGATION COMPARISON\n');
fprintf('============================================================\n');
fprintf('Robot size: %.2f m x %.2f m\n',cfg.robot.length,cfg.robot.width);
fprintf('Path samples: %d\n',numel(path.s));
fprintf('Minimum planned obstacle clearance: %.3f m\n',path.minimumClearance);

% Store structures in cells first. Direct assignment into struct([]) can
% produce "Subscripted assignment between dissimilar structures" in
% some MATLAB releases because the empty structure has no defined fields.
numberOfControllers = numel(controllerNames);
resultCells = cell(1,numberOfControllers);
metricCells = cell(1,numberOfControllers);

for i = 1:numberOfControllers
    fprintf('Simulating %s controller...\n',controllerNames{i});
    oneResult = simulateController(controllerNames{i},path,env,cfg);
    resultCells{i} = oneResult;
    metricCells{i} = calculateMetrics(oneResult,path,cfg);
end

% Every controller returns the same fields, so these cell contents can now
% be combined safely into ordinary structure arrays.
results = [resultCells{:}];
metricStruct = [metricCells{:}];
metricsTable = struct2table(metricStruct);
disp(metricsTable);
writetable(metricsTable,fullfile(cfg.outputDir,'controller_metrics.csv'));

visualizeComparison(path,env,results,metricsTable,cfg);
animateControllers(path,env,results,cfg);

save(fullfile(cfg.outputDir,'simulation_results.mat'), ...
    'cfg','env','path','results','metricsTable');

fprintf('\nGenerated outputs in: %s\n',cfg.outputDir);
fprintf(' - controller_metrics.csv\n');
fprintf(' - trajectory_comparison.png\n');
fprintf(' - errors_and_commands.png\n');
fprintf(' - metric_comparison.png\n');
fprintf(' - final_animation_frame.png\n');
fprintf(' - controller_comparison.mp4 (when video is supported)\n');
fprintf(' - simulation_results.mat\n\n');
