%% FILE SUMMARY
% Purpose: Segment and visualize safety experiment trajectories for selected conditions.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Signal Processing Toolbox; Curve Fitting Toolbox.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: CSV logs in data/magnetic_data/safety_experiments.
% Outputs: Time/trajectory plots and per-trial safety behavior inspection figures.
% Run Notes: Change analysisType to select real/virtual and magnet-position cases.


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
    % Operations over path
    currentPath = pwd;
    cd ..
    addpath(genpath("data/session_01"))
    cd(currentPath)
end

addpath("dependencies")

%% Data Loading

% Number to switch the file analysed
analysisType = 3;

stopSpeed = 5e-4;

% Target points in the workspace for the robot end-effector.
waypoints = [0.466457,   0.3,  0.40
             0.466457,  -0.3,  0.40];

magnetPosition = [];

switch analysisType
    case 1 % Magnetic Tracking Active - Magnet on the Left side
        filename = "SAFETY_REAL1_exp_09_07_15_36_24.csv";
        magGT = [0.466 -0.215 0.3265 0 1 0];
        tStop = 300;
    case 2 % Magnetic Tracking Active - Magnet on the Center
        filename = "safety_real2_exp_09_07_17_26_05";
        magGT = [0.466 -0.01 0.3265 0 1 0];
        tStop = 300;
    case 3 % Magnetic Tracking Active - Magnet on the Right side
        filename = "SAFETY_REAL_3_exp_09_07_18_29_45.csv";
        magGT = [0.466 0.194 0.3265 0 1 0];
        tStop = 300;
    case 4 % Magnetic Tracking NOT Active - Magnet on the Left side
        filename = "SAFETY_VIRTUAL1_exp_09_07_16_19_16.csv";
        magGT = [0.466 -0.215 0.3265 0 1 0];
        tStop = 300;
    case 5 % Magnetic Tracking NOT Active - Magnet on the Center
        filename = "safety_virtual_2_exp_09_07_17_32_29.csv";
        magGT = [0.466 -0.01 0.3265 0 1 0];
        tStop = 300;
    case 6 % Magnetic Tracking NOT Active - Magnet on the Right side
        filename = "SAFETY_VIRTUAL_3_exp_09_07_18_27_01.csv";
        magGT = [0.466 0.194 0.3265 0 1 0];
        tStop = 300;
end

%% DATA EXTRACTIONS FROM FILE

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

%% Data Display

isOutliers = magPose(:,end) == -1;
magPose(isOutliers,:) = nan;

isZero = EE_position(:,1) == 0;
EE_position(isZero,:) = nan;
EE_velocity(isZero,:) = nan;

[pks,locs]   = findpeaks(EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);
[pks2,locs2] = findpeaks(-EE_position(:,2),"MinPeakHeight",-min(EE_position(:,2))/2);

figure("Units","normalized","Position",[0.3 0.3 0.5 0.3])
plot(time,magPose)
xlabel("Time (s)","FontSize",10,"Interpreter","latex")
legend(["$x_M$" "$y_M$" "$z_M$" "$m_x$" "$m_y$" "$m_z$"],...
       "FontSize",10,"Interpreter","latex","Location","eastoutside")

figure
plot(time,magField)
ylabel("Magnetic Field Strength ($\mu T$)","FontSize",10,"Interpreter","latex")
xlabel("Time (s)","FontSize",10,"Interpreter","latex")
legend(["$B_{1x}$", "$B_{1y}$", "$B_{1z}$", ...
        "$B_{2x}$", "$B_{2y}$", "$B_{2z}$", ...
        "$B_{3x}$", "$B_{3y}$", "$B_{3z}$", ...
        "$B_{4x}$", "$B_{4y}$", "$B_{4z}$"],...
       "FontSize",10,"Interpreter","latex","Location","eastoutside")

motionEndsIdx = sort([locs; locs2]);
speedIdx = find(abs(EE_velocity(:,2)) < stopSpeed & abs(EE_position(:,2)) < 0.2);

figure
subplot(211)
hold on
plot(time,EE_position)
plot(time(motionEndsIdx),EE_position(motionEndsIdx,2),'k*')
plot(time(speedIdx),EE_position(speedIdx,2),'ko')
hold off
ylabel("EE position (m)","FontSize",10,"Interpreter","latex")
xlabel("Time (s)","FontSize",10,"Interpreter","latex")
subplot(212)
plot(time,EE_velocity)
ylabel("EE velocity (m/s)","FontSize",10,"Interpreter","latex")
xlabel("Time (s)","FontSize",10,"Interpreter","latex")

figure
subplot(311)
plot(time,EE_position(:,2))
subplot(312)
hold on
plot(time(1:end-1),movmean(diff(EE_position(:,2)),250)*1e3)
plot(time,EE_velocity(:,2))
hold off
subplot(313)
plot(time(1:end-1),movmean(diff(EE_velocity(:,2)),250)*1e3)

%%

plotMask = time >= 55 & time <= 59;

magDist = vecnorm(EE_position-magGT(1:3),2,2);

figure
subplot(211)
% hold on
% plot(time(plotMask),EE_velocity(plotMask,2))
plot(time(plotMask),smooth(EE_velocity(plotMask,2),300))
% hold off
subplot(212)
plot(time(plotMask),smooth(magDist(plotMask),300))

%% TRIALS SEGMENTATION
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
        x = x/max(x);

        minDist(i) = norm(EE_position(endP,:)-magGT(1:3));

        % for EEx
        y = EE_position(startP:endP,1);
        resampledTrials_x(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEy
        y = EE_position(startP:endP,2);
        resampledTrials_y(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEz
        y = EE_position(startP:endP,3);
        resampledTrials_z(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);
    else
        nTrials2 = nTrials2-1;
    end
end

%%

figure
subplot(131)
plot(normalizedTime,resampledTrials_x);
subplot(132)
plot(normalizedTime,resampledTrials_y);
subplot(133)
plot(normalizedTime,resampledTrials_z);

R_stop = 0.3;
angles = linspace(0,2*pi,100);

[X,Y,Z] = sphere(100);
X = X*R_stop + magGT(:,1);
Y = Y*R_stop + magGT(:,2);
Z = Z*R_stop + magGT(:,3);

R_slow = 0.13;
angles = linspace(0,2*pi,100);

[X2,Y2,Z2] = sphere(100);
X2 = X2*R_slow + magGT(:,1);
Y2 = Y2*R_slow + magGT(:,2);
Z2 = Z2*R_slow + magGT(:,3);


figure
hold on
for i = 1:nTrials
    plot3(resampledTrials_x(:,i),resampledTrials_y(:,i),resampledTrials_z(:,i))
end
scatter3(magGT(1),magGT(2),magGT(3),8,'filled','k')
surf(X,Y,Z,"EdgeColor","none","FaceColor","#249A3C")
alpha 0.025
surf(X2,Y2,Z2,"EdgeColor","none","FaceColor","#249A3C")
alpha 0.025
scatter3(waypoints(:,1),waypoints(:,2),waypoints(:,3),35,'filled','k')
hold off
axis equal
axis off
view(40,25)

figure
hold on
for i = 1:nTrials
    plot(resampledTrials_y(:,i),resampledTrials_z(:,i))
end
scatter(magGT(2),magGT(3),8,'filled','k')
plot(magGT(:,2)+R_stop*cos(angles),magGT(:,3)+R_stop*sin(angles),'k--')
plot(magGT(:,2)+R_slow*cos(angles),magGT(:,3)+R_slow*sin(angles),'k--')
scatter(waypoints(:,2),waypoints(:,3),35,'filled','k')
axis equal
xlim([-0.6 0.6])
xticks(-0.6:0.15:0.6)

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