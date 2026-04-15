%% FILE SUMMARY
% Purpose: Compare safety metrics (stop distance/time) across real and virtual conditions.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Signal Processing Toolbox; Statistics and Machine Learning Toolbox; Curve Fitting Toolbox; local package dependencies/dabarplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: CSV logs in data/magnetic_data/safety_experiments.
% Outputs: Aggregated bar/scatter summaries and nonparametric statistical test outputs.
% Run Notes: Designed to produce consolidated quantitative panels for publication.


%% NOTE for CODE
% This code has the basic structure to segment trials in collision
% avoidance experiments.
% Change the variable "analysisType" from 1 to 6 to view different EE
% trajectories visualized with the avoidance region, waypoints and the
% magnet.
%
% This code generates panels b,c and d of First Results Figure related to
% collision avoidance. In detail:
% - analysisType = 1 generates panel b
% - analysisType = 2 generates panel c
% - analysisType = 3 generates panel d
%
% analysisType = 4,5,6 show the same plots when the tracking algorithm was
% not active and the robot was controlled knowing the exact magnet
% location.



warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath(genpath("data/session_01"))
    end
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % operations over path
    currentPath = pwd;
    cd ..
    addpath(genpath("data/session_01"))
    cd(currentPath)
end

addpath("dependencies")

%% Data Loading

stopSpeed = 5e-4;

% Target points in the workspace for the robot end-effector.
waypoints = [0.466457,   0.3,  0.40
    0.466457,  -0.3,  0.40];

% Number to switch the file analysed
for j = 1:3

    switch j
        case 1 % Magnetic Tracking Active - Magnet on the Left side
            filename = "SAFETY_REAL1_exp_09_07_15_36_24.csv";
            magGT = [0.466 -0.205 0.3265 0 1 0];
            tStop = 300;
        case 2 % Magnetic Tracking Active - Magnet on the Center
            filename = "safety_real2_exp_09_07_17_26_05";
            magGT = [0.466 -0.005 0.3265 0 1 0];
            tStop = 300;
        case 3 % Magnetic Tracking Active - Magnet on the Right side
            filename = "SAFETY_REAL_3_exp_09_07_18_29_45.csv";
            magGT = [0.466 0.2 0.3265 0 1 0];
            tStop = 300;
    end

    % DATA EXTRACTIONS FROM FILE

    % custom function to read/import csv data exported in c++ (see at the end of the file)
    dataTable = import_CSV_Table(filename);

    time = dataTable.time;

    time        = time-time(1); % reset time start
    magPose     = dataTable{:,2:7};
    magField    = dataTable{:,17:end};
    EE_position = dataTable{:,8:10};
    EE_velocity = dataTable{:,11:13};

    % remove bad final data
    data2remove = time > tStop;
    time(data2remove)          = [];
    magPose(data2remove,:)     = [];
    magField(data2remove,:)    = [];
    EE_position(data2remove,:) = [];
    EE_velocity(data2remove,:) = [];

    [pks,locs]   = findpeaks(EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);
    [pks2,locs2] = findpeaks(-EE_position(:,2),"MinPeakHeight",-min(EE_position(:,2))/2);

    motionEndsIdx = sort([locs; locs2]);
    speedIdx = find(abs(EE_velocity(:,2)) < stopSpeed & abs(EE_position(:,2)) < 0.2);

    % TRIALS SEGMENTATION
    % Different Trials can be roughly segmented as in the avoidance case,
    % considering the end-points of the end-effector positions along y (peaks
    % in the y end-effector coordinates).
    % This time, the Trials stop when the derivative of the position (velocity)
    % and the derivative of the velocity (acceleration) are zero.

    numSamplesInterval = diff(motionEndsIdx);
    maxNumSamplesInterval = max(numSamplesInterval);
    normalizedTime = linspace(0,1,maxNumSamplesInterval);
    nTrials = numel(motionEndsIdx)-1;
    nSamplFiltering = 100;

    resampledTrials_x = zeros(maxNumSamplesInterval,nTrials);
    resampledTrials_y = resampledTrials_x;
    resampledTrials_z = resampledTrials_x;
    xq = normalizedTime';
    k = 1;
    nTrials2 = nTrials;

    figure
    subplot(211)
    plot(time,EE_position(:,2))
    subplot(212)
    plot(time,EE_velocity(:,2))

    minDist = struct();


    for i = 1:nTrials

        if k == length(motionEndsIdx)
            break
        end
        startP = motionEndsIdx(k);
        endP = min(speedIdx(speedIdx > motionEndsIdx(k) & speedIdx < motionEndsIdx(k+1)));
        k = k+1;
        x = time(startP:endP);

        if ~isempty(x)
            x = x-x(1);
            x2 = x;
            x = x/max(x);

            minDist(j).val(i) = norm(EE_position(endP,:)-magGT(1:3));

            % for EEx
            y = EE_position(startP:endP,1);
            resampledTrials_x(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

            % for EEy
            y = EE_position(startP:endP,2);
            resampledTrials_y(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

            % for EEz
            y = EE_position(startP:endP,3);
            resampledTrials_z(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

            y = smooth(EE_velocity(startP:endP,2));

        else
            nTrials2 = nTrials2-1;
        end
    end

    minDist(j).val(minDist(j).val == 0) = [];

end

%%

for i = 1:3
    switch i
        case 1 % Magnetic Tracking NOT Active - Magnet on the Left side
            filename = "SAFETY_VIRTUAL1_exp_09_07_16_19_16.csv";
            magGT = [0.466 -0.205 0.3265 0 1 0];
        case 2 % Magnetic Tracking NOT Active - Magnet on the Center
            filename = "safety_virtual_2_exp_09_07_17_32_29.csv";
            magGT = [0.466 -0.005 0.3265 0 1 0];
        case 3 % Magnetic Tracking NOT Active - Magnet on the Right side
            filename = "SAFETY_VIRTUAL_3_exp_09_07_18_27_01.csv";
            magGT = [0.466 0.188 0.3265 0 1 0];
    end

    % DATA EXTRACTIONS FROM FILE

    % custom function to read/import csv data exported in c++ (see at the end of the file)
    dataTable = import_CSV_Table(filename);

    time = dataTable.time;

    time        = time-time(1); % reset time start
    magPose     = dataTable{:,2:7};
    magField    = dataTable{:,17:end};
    EE_position = dataTable{:,8:10};
    EE_velocity = dataTable{:,11:13};

    % remove bad final data
    data2remove = time > tStop;
    time(data2remove)          = [];
    magPose(data2remove,:)     = [];
    magField(data2remove,:)    = [];
    EE_position(data2remove,:) = [];
    EE_velocity(data2remove,:) = [];

    groundTruthStop(i) = norm(EE_position(end,:)-magGT(1:3));
end

figure
plot(time, EE_velocity(:,2))

%% t slowdown and stop extracted from preprocessed and segmented data

Times(1).tSD = [0.5570    0.5410    0.5540    0.5590    0.5470    0.5570    0.5420    0.5400    0.5160    0.5210    0.5410    0.5570    0.5200    0.5440    0.5570];
Times(1).tST = [0.4840    0.4790    0.4790    0.4920    0.5200    0.5220    0.4830    0.4830    0.5240    0.5220    0.5210    0.5240    0.4880    0.5230    0.5330];

Times(2).tSD = [0.5000    0.4800    0.5200    0.4960    0.5320    0.5200    0.4360    0.4680    0.4630    0.4800    0.5200    0.4770    0.5250    0.5020    0.5050];
Times(2).tST = [0.5530    0.5250    0.5390    0.5250    0.5340    0.4770    0.5210    0.5010    0.5250    0.5170    0.5610    0.5210    0.5290    0.5250    0.4810];

Times(3).tSD = [0.5820    0.5370    0.5610    0.5780    0.4910    0.5610    0.5330    0.5260    0.5290    0.5510    0.5470    0.5550    0.5140    0.5060    0.5650];
Times(3).tST = [0.5480    0.6060    0.4350    0.4880    0.5220    0.4940    0.4840    0.5160    0.4790    0.4700    0.5250    0.5230    0.5060    0.4810    0.4790];

%% STOP DISTANCE ANALYSIS

% stop distances extracted from preprocessed data in minDist structure

StopDistArray = [0.1245    0.1385    0.1371
    0.1261    0.1372    0.1376
    0.1280    0.1353    0.1373
    0.1303    0.1355    0.1335
    0.1400    0.1345    0.1331
    0.1218    0.1335    0.1373
    0.1280    0.1351    0.1373
    0.1266    0.1408    0.1332
    0.1398    0.1306    0.1341
    0.1258    0.1357    0.1335
    0.1422    0.1348    0.1382
    0.1283    0.1347    0.1347
    0.1286    0.1340    0.1350
    0.1435    0.1365    0.1350
    0.1371    0.1371    0.1335];

figure("Units","normalized","Position",[0.3 0.3 0.15 0.3])
subplot(211)
hold on
h = dabarplot(StopDistArray,'errorbars','SD',...
    'barspacing',0.8);
scatter(1:3,groundTruthStop,"filled","r","Marker","square")
hold off
subplot(212)
hold on
h = dabarplot(StopDistArray(:),'errorbars','SD',...
    'barspacing',0.8);
scatter(1,mean(groundTruthStop),"filled","r","Marker","square")
hold off


try
    if(test)
        [p,tbl,stats] = friedman(StopDistArray);

        [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni");

        [p,h,stats] = signrank(StopDistArray(:),mean(groundTruthStop));
    end
catch exception
    disp("STOP DISTANCES")

    [p,tbl,stats] = friedman(StopDistArray)

    [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni")

    [p,h,stats] = signrank(StopDistArray(:),mean(groundTruthStop))
end


TimesArray = [Times(1).tSD(:) Times(1).tST(:)...
    Times(2).tSD(:) Times(2).tST(:)...
    Times(3).tSD(:) Times(3).tST(:)];

try
    if(test)
        %% SLOW DOWN TIME ANALYSIS
        figure
        subplot(211)
        h = dabarplot(TimesArray,'errorbars','SD',...
            'barspacing',0.8);

        [p,tbl,stats] = friedman(TimesArray(:,[1:2:end]));

        [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni");

        [p,h,stats] = signrank(reshape(TimesArray(:,1:2:end),[],1),0.522);

        %% STOP TIME ANALYSIS

        [p,tbl,stats] = friedman(TimesArray(:,[2:2:end]));

        subplot(212)
        h = dabarplot([reshape(TimesArray(:,1:2:end),[],1) reshape(TimesArray(:,2:2:end),[],1)],'errorbars','SD',...
            'barspacing',0.8);

        [p,h,stats] = signrank(reshape(TimesArray(:,2:2:end),[],1),0.502);
    end
catch exception
    %% SLOW DOWN TIME ANALYSIS
    figure
    subplot(211)
    h = dabarplot(TimesArray,'errorbars','SD',...
        'barspacing',0.8);

    disp("SLOW DOWN TIMES")
    [p,tbl,stats] = friedman(TimesArray(:,[1:2:end]))

    [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni")

    [p,h,stats] = signrank(reshape(TimesArray(:,1:2:end),[],1),0.522)

    %% STOP TIME ANALYSIS

    disp("STOP TIMES")
    [p,tbl,stats] = friedman(TimesArray(:,[2:2:end]))

    subplot(212)
    h = dabarplot([reshape(TimesArray(:,1:2:end),[],1) reshape(TimesArray(:,2:2:end),[],1)],'errorbars','SD',...
        'barspacing',0.8);

    [p,h,stats] = signrank(reshape(TimesArray(:,2:2:end),[],1),0.502)
end


%% Auxiliary Function

function data = import_CSV_Table(filename, dataLines)
% Auto-generated by MATLAB on 31-Oct-2024 16:52:49

% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 28);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "mag_pos_x", "mag_pos_y", "mag_pos_z", "mag_orien_x", "mag_orien_y", "mag_orien_z", "ee_x", "ee_y", "ee_z", "v_ee_x", "v_ee_y", "v_ee_z", "F_ee_x", "F_ee_y", "F_ee_z", "s1_x", "s1_y", "s1_z", "s2_x", "s2_y", "s2_z", "s3_x", "s3_y", "s3_z", "s4_x", "s4_y", "s4_z"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);

end