%% FILE SUMMARY
% Purpose: Analyze avoidance behavior versus obstacle distance/radius using segmented trials.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Signal Processing Toolbox; Curve Fitting Toolbox; local package dependencies/dabarplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: Session CSV logs in data/session_fede_2_novembre.
% Outputs: Resampled trajectories and distance/error summaries by condition.
% Run Notes: Set direction and analysisType parameters to choose trajectory subsets.


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
        addpath(genpath("data/session_02"))
    end
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % Operations over path
    currentPath = pwd;
    cd ..
    addpath(genpath("data/session_02"))
    cd(currentPath)
end

addpath("dependencies")

%% Data Loading

waypoints = [0.54788,   0.45,  0.40
             0.54788,  -0.45,  0.40];

magnetPosition = [];
direction = 1; % options available: 1:left, 2:right

Color = [0      0.4470 0.7410
         0.8500 0.3250 0.0980
         0.9290 0.6940 0.1250
         0.4940 0.1840 0.5560
         0.4660 0.6740 0.1880];

plotType = "2D"; % options available: "2D", "3D"

figure
hold on

for j = 1:5

    switch j
        case 1
            filename = "Avoidance_exp_11_03_13_47_49.csv"; % 30mm
        case 2
            filename = "Avoidance_exp_11_03_13_59_42.csv"; % 25mm
        case 3
            filename = "Avoidance_exp_11_03_14_15_14.csv"; % 20mm
        case 4
            filename = "Avoidance_exp_11_03_14_26_19.csv"; % 15mm
        case 5
            filename = "Avoidance_exp_11_03_14_33_23.csv"; % 10mm
    end

    magGT = [0.54788 0.00006 0.32819];

    dataTable = import_CSV_Table(filename);

    time = dataTable.time;
    %
    time = time-time(1);
    magPose = dataTable{:,2:7};
    magField = dataTable{:,17:end};
    EE_position = dataTable{:,8:10};
    EE_velocity = dataTable{:,11:13};

    %% Data Display
    isOutlier = magPose(:,end) == -1;
    magPose(isOutlier,:) = nan;

    isZero = EE_position(:,1) == 0;
    EE_position(isZero,:) = nan;
    EE_velocity(isZero,:) = nan;

    [pks,locs] = findpeaks(EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);
    [pks2,locs2] = findpeaks(-EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);

    numSamplesInterval = diff(locs);
    maxNumSamplesInterval = max(numSamplesInterval);
    normalizedTime = linspace(0,1,maxNumSamplesInterval);
    nTrials = numel(locs)-1;
    nSamplFiltering = 100;

    resampledTrials_x = zeros(maxNumSamplesInterval,nTrials-1);
    resampledTrials_y = resampledTrials_x;
    resampledTrials_z = resampledTrials_x;
    xq = normalizedTime';

    for i = 1:nTrials
        switch direction
            case 1
                if locs(1) < locs2(1)
                    timeMask = locs(i):locs2(i);
                else
                    timeMask = locs(i):locs2(i+1);
                end
            case 2
                if locs(1) < locs2(1)
                    timeMask = locs2(i):locs(i+1);
                else
                    timeMask = locs2(i):locs(i);
                end
        end
       
        x = time(timeMask);
        x = x-x(1);
        x = x/max(x);

        % for EEx
        y = EE_position(timeMask,1);
        resampledTrials_x(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEy
        y = EE_position(timeMask,2);
        resampledTrials_y(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEz
        y = EE_position(timeMask,3);
        resampledTrials_z(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        resampledNorm = vecnorm([resampledTrials_x(:,i) - magGT(1) ...
                                 resampledTrials_y(:,i) - magGT(2) ...
                                 resampledTrials_z(:,i) - magGT(3)],2,2);

        minDistances(j).values(i) = min(resampledNorm);
    end

    % to repeat analyses on the other side
    if direction == 1
        direction = 2;
    else
        direction = 1;
    end

    resampledTrials_x_nomem = zeros(maxNumSamplesInterval,nTrials-1);
    resampledTrials_y_nomem = resampledTrials_x;
    resampledTrials_z_nomem = resampledTrials_x;

    for i = 1:nTrials
        switch direction
            case 1
                if locs(1) < locs2(1)
                    timeMask = locs(i):locs2(i);
                else
                    timeMask = locs(i):locs2(i+1);
                end
            case 2
                if locs(1) < locs2(1)
                    timeMask = locs2(i):locs(i+1);
                else
                    timeMask = locs2(i):locs(i);
                end
        end
       
        x = time(timeMask);
        x = x-x(1);
        x = x/max(x);

        % for EEx
        y = EE_position(timeMask,1);
        resampledTrials_x_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEy
        y = EE_position(timeMask,2);
        resampledTrials_y_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEz
        y = EE_position(timeMask,3);
        resampledTrials_z_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        resampledNorm = vecnorm([resampledTrials_x_nomem(:,i) - magGT(1) ...
                                 resampledTrials_y_nomem(:,i) - magGT(2) ...
                                 resampledTrials_z_nomem(:,i) - magGT(3)],2,2);

        minDistances(j).values(nTrials+i) = min(resampledNorm);
    end

    % re flip direction
    if direction == 1
        direction = 2;
    else
        direction = 1;
    end

    R_avoid = 0.18;
    angles = linspace(0,2*pi,100);

    [X,Y,Z] = sphere(100);
    X = X*R_avoid + magGT(:,1);
    Y = Y*R_avoid + magGT(:,2);
    Z = Z*R_avoid + magGT(:,3);

    switch plotType
        case "2D"
            for i = 1:nTrials-1
                plot(resampledTrials_y(:,i),resampledTrials_z(:,i),"Color",Color(j,:))
            end
        case "3D"
            for i = 1:nTrials-1
                plot3(resampledTrials_x(:,i),resampledTrials_y(:,i),resampledTrials_z(:,i),"Color",Color(j,:))
            end
        otherwise
    end

end

switch plotType
    case "2D"
        scatter(magGT(2),magGT(3),8,'filled','k')
        scatter(waypoints(:,2),waypoints(:,3),35,'filled','k')
        hold off
        axis([-0.4 0.4 0 0.75])
        axis equal
    case "3D"
        scatter3(magGT(1),magGT(2),magGT(3),8,'filled','k')

        scatter3(waypoints(:,1),waypoints(:,2),waypoints(:,3),35,'filled','k')
        hold off
        axis([0.4 0.5 -0.5 0.5 0 1])
        axis equal
        axis off
        view(40,25)
    otherwise
end

%%

minDistArray = [minDistances(1).values(1:18)
                minDistances(2).values(1:18)
                minDistances(3).values([1:9 12:20])
                minDistances(4).values([1:9 13:21])
                minDistances(5).values(1:18)]';

figure("Units","normalized","Position",[0.3 0.3 0.15 0.3])
h = dabarplot(minDistArray,'errorbars','SD',...
    'barspacing',0.8);
ylim([0 0.2])
yticks(0:0.05:0.2)


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