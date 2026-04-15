%% FILE SUMMARY
% Purpose: Extend radius-based avoidance analysis with velocity and timing features.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Signal Processing Toolbox; Curve Fitting Toolbox; local package dependencies/dabarplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: Session CSV logs in data/session_fede_2_novembre.
% Outputs: Trajectory/velocity visualizations and condition-wise summary plots.
% Run Notes: Set direction to isolate left-to-right or right-to-left movements.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath(genpath("data/session_01"))
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
    addpath(genpath("data/session_01"))
    addpath(genpath("data/session_02"))
    cd(currentPath)
end

addpath(genpath("dependencies"))

%% Data Loading

waypoints = [0.54788,   0.45,  0.40
             0.54788,  -0.45,  0.40];

magnetPosition = [];
direction = 2; % options: 1:left, 2:right

Color = [0      0.4470 0.7410
         0.8500 0.3250 0.0980
         0.9290 0.6940 0.1250
         0.4940 0.1840 0.5560
         0.4660 0.6740 0.1880];

plotType = "2D"; % options: "2D", "3D"

chosen = 2;

figure(Units="normalized",Position=[0.3 0.3 0.25 0.3])
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

    % 18 -> pos (3), vel (3), mag data (12)
    numData = 24;
    resampledTrials = zeros(maxNumSamplesInterval,numData,nTrials-1);
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
        y = [EE_position EE_velocity magField magPose];
        y = y(timeMask,:);
        resampledTrials(:,:,i) = smoothANDresample(x,y,xq,nSamplFiltering);

        resampledNorm = vecnorm([resampledTrials(:,1,i) - magGT(1) ...
                                 resampledTrials(:,2,i) - magGT(2) ...
                                 resampledTrials(:,3,i) - magGT(3)],2,2);

        minDistances(j).values(i) = min(resampledNorm);
    end

    xq2plot(j).xq = xq;
    resampledTrials2Plot(j).dataX = median(squeeze(resampledTrials(:,1,:)),2);
    resampledTrials2Plot(j).dataY = median(squeeze(resampledTrials(:,2,:)),2);
    resampledTrials2Plot(j).dataZ = median(squeeze(resampledTrials(:,3,:)),2);
    for k = 1:12
        resampledTrials2Plot(j).B(k,:) = median(squeeze(resampledTrials(:,6+k,:)),2);
    end
    resampledTrials2Plot(j).Mag(:,1) = median(squeeze(resampledTrials(:,19,:)),2,"omitnan");
    resampledTrials2Plot(j).Mag(:,2) = median(squeeze(resampledTrials(:,20,:)),2,"omitnan");
    resampledTrials2Plot(j).Mag(:,3) = median(squeeze(resampledTrials(:,21,:)),2,"omitnan");
    resampledTrials2Plot(j).MagDist = vecnorm([resampledTrials2Plot(j).dataX - magGT(1) ...
                                               resampledTrials2Plot(j).dataY - magGT(2) ...
                                               resampledTrials2Plot(j).dataZ - magGT(3)],2,2);


    % to repeat analyses on the other side
    if direction == 1
        direction = 2;
    else
        direction = 1;
    end

    resampledTrials_nomem = zeros(maxNumSamplesInterval,numData,nTrials-1);

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
        y = [EE_position EE_velocity magField magPose];
        y = y(timeMask,:);
        resampledTrials_nomem(:,:,i) = smoothANDresample(x,y,xq,nSamplFiltering);

        resampledNorm = vecnorm([resampledTrials_nomem(:,1,i) - magGT(1) ...
                                 resampledTrials_nomem(:,2,i) - magGT(2) ...
                                 resampledTrials_nomem(:,3,i) - magGT(3)],2,2);

        minDistances(j).values(nTrials+i) = min(resampledNorm);
    end

    for k = 1:12
        resampledTrials2Plot(j).B2(k,:) = median(squeeze(resampledTrials_nomem(:,6+k,:)),2);
    end
    resampledTrials2Plot(j).Mag2(:,1) = median(squeeze(resampledTrials_nomem(:,19,:)),2,"omitnan");
    resampledTrials2Plot(j).Mag2(:,2) = median(squeeze(resampledTrials_nomem(:,20,:)),2,"omitnan");
    resampledTrials2Plot(j).Mag2(:,3) = median(squeeze(resampledTrials_nomem(:,21,:)),2,"omitnan");

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
            plot(resampledTrials2Plot(j).dataY,resampledTrials2Plot(j).dataZ,"Color",Color(j,:))
        case "3D"
            for i = 1:nTrials-1
                plot3(resampledTrials(:,1,i),resampledTrials(:,2,i),resampledTrials(:,3,i),"Color",Color(j,:))
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

%%

chosen = 3;

sensColors = ["#45579D"; "#1A8CBD"; "#20A4B5"
              "#6DB844"; "#C5D62E"; "#FCF218"
              "#F9CF5E"; "#F7AA15"; "#F17B3A"
              "#EC1B3B"; "#B3509A"; "#9568A7"];

figure
subplot(221)
hold on
for k = 1:12
    maskTrack = resampledTrials2Plot(chosen).MagDist <= 0.25;
    plot(xq2plot(chosen).xq(maskTrack),smooth(resampledTrials2Plot(chosen).B(k,maskTrack),1001),"Color",sensColors(k))
end
xlim([min(xq2plot(chosen).xq(maskTrack)) max(xq2plot(chosen).xq(maskTrack))])
ylim([-100 150])
hold off

subplot(222)
maskTrack = resampledTrials2Plot(chosen).MagDist <= 0.25;
errorArray = [smooth(resampledTrials2Plot(chosen).Mag(maskTrack,1),1001)-magGT(1), ...
              smooth(resampledTrials2Plot(chosen).Mag(maskTrack,2),1001)-magGT(2), ...
              smooth(resampledTrials2Plot(chosen).Mag(maskTrack,3),1001)-magGT(3), ...
              vecnorm([smooth(resampledTrials2Plot(chosen).Mag(maskTrack,1),1001)-magGT(1),...
                       smooth(resampledTrials2Plot(chosen).Mag(maskTrack,2),1001)-magGT(2),...
                       smooth(resampledTrials2Plot(chosen).Mag(maskTrack,3),1001)-magGT(3)],2,2)];
h = dabarplot(errorArray,'errorbars','SD',...
    'barspacing',0.8);
ylim([-0.04 0.04])

subplot(223)
hold on
for k = 1:12
    maskTrack = resampledTrials2Plot(chosen).MagDist <= 0.25;
    plot(xq2plot(chosen).xq(maskTrack),smooth(resampledTrials2Plot(chosen).B2(k,maskTrack),1001),"Color",sensColors(k))
end
xlim([min(xq2plot(chosen).xq(maskTrack)) max(xq2plot(chosen).xq(maskTrack))])
ylim([-150 100])
hold off

subplot(224)
maskTrack = resampledTrials2Plot(chosen).MagDist <= 0.25;
errorArray = [smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,1),1001)-magGT(1), ...
              smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,2),1001)-magGT(2), ...
              smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,3),1001)-magGT(3), ...
              vecnorm([smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,1),1001)-magGT(1),...
                       smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,2),1001)-magGT(2),...
                       smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,3),1001)-magGT(3)],2,2)];
h = dabarplot(errorArray,'errorbars','SD',...
    'barspacing',0.8);
ylim([-0.04 0.04])

%%

figure
subplot(221)
hold on
for k = 1:12
    maskTrack = resampledTrials2Plot(chosen).MagDist <= 0.25;
    plot(xq2plot(chosen).xq(maskTrack),smooth(resampledTrials2Plot(chosen).B(k,maskTrack),1001),"Color",sensColors(k))
end
xlim([min(xq2plot(chosen).xq(maskTrack)) max(xq2plot(chosen).xq(maskTrack))])
ylim([-100 150])
hold off

EE_posResampled = [resampledTrials2Plot(chosen).dataX(maskTrack) ...
                   resampledTrials2Plot(chosen).dataY(maskTrack) ...
                   resampledTrials2Plot(chosen).dataZ(maskTrack)];
distance = vecnorm(EE_posResampled-magGT,2,2);

subplot(222)
maskTrack = resampledTrials2Plot(chosen).MagDist <= 0.25;
errorArray = [(smooth(resampledTrials2Plot(chosen).Mag(maskTrack,1),1001)-magGT(1))./distance*100, ...
              (smooth(resampledTrials2Plot(chosen).Mag(maskTrack,2),1001)-magGT(2))./distance*100, ...
              (smooth(resampledTrials2Plot(chosen).Mag(maskTrack,3),1001)-magGT(3))./distance*100, ...
              vecnorm([(smooth(resampledTrials2Plot(chosen).Mag(maskTrack,1),1001)-magGT(1))./distance*100,...
                       (smooth(resampledTrials2Plot(chosen).Mag(maskTrack,2),1001)-magGT(2))./distance*100,...
                       (smooth(resampledTrials2Plot(chosen).Mag(maskTrack,3),1001)-magGT(3))./distance*100],2,2)];
h = dabarplot(errorArray,'errorbars','SD',...
    'barspacing',0.8);
ylim([-12 12])
yticks(-12:6:12)

subplot(223)
hold on
for k = 1:12
    maskTrack = resampledTrials2Plot(chosen).MagDist <= 0.25;
    plot(xq2plot(chosen).xq(maskTrack),smooth(resampledTrials2Plot(chosen).B2(k,maskTrack),1001),"Color",sensColors(k))
end
xlim([min(xq2plot(chosen).xq(maskTrack)) max(xq2plot(chosen).xq(maskTrack))])
ylim([-150 100])
hold off

subplot(224)
maskTrack = resampledTrials2Plot(chosen).MagDist <= 0.25;
errorArray = [(smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,1),1001)-magGT(1))./distance*100, ...
              (smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,2),1001)-magGT(2))./distance*100, ...
              (smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,3),1001)-magGT(3))./distance*100, ...
              vecnorm([(smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,1),1001)-magGT(1))./distance*100,...
                       (smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,2),1001)-magGT(2))./distance*100,...
                       (smooth(resampledTrials2Plot(chosen).Mag2(maskTrack,3),1001)-magGT(3))./distance*100],2,2)];
h = dabarplot(errorArray,'errorbars','SD',...
    'barspacing',0.8);
ylim([-12 12])
yticks(-12:6:12)


%% Auxiliary Function

function yq = smoothANDresample(x,y,xq,nSamplFiltering)

yq = zeros(length(xq),size(y,2));

for i = 1:size(y,2)
    yq(:,i) = smooth(pchip(x,y(:,i),xq),nSamplFiltering);
end

end

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