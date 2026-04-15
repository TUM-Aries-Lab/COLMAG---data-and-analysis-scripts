%% FILE SUMMARY
% Purpose: Reconstruct and visualize a 2D velocity field for avoidance trajectories.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Signal Processing Toolbox; Curve Fitting Toolbox.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: CSV logs in data/magnetic_data/avoidance_exp.
% Outputs: Interpolated velocity maps and streamline-like plots around the obstacle location.
% Run Notes: Set location and direction to choose scenario and motion orientation.

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

%%

Npt = 250;
[x, y] = meshgrid(linspace(-0.55,0.55,Npt),linspace(0,0.65,Npt));

location = 1;
direction = 2; % from left to right

switch location
    case 1
        filename = "Avoidance_VIRTUAL1_exp_09_07_16_11_46.csv";
        M = [-0.215 0.3265];
    case 2
        filename = "Avoidance_VIRTUAL2_exp_09_07_17_02_15.csv";
        M = [-0.01 0.3265];
    case 3
        filename = "Avoidance_VIRTUAL_3_exp_09_07_18_07_01.csv";
        M = [0.194 0.3265];
end

switch direction
    case 1
        A = [ 0.45,  0.40];
        B = [-0.45,  0.40];
    case 2
        B = [ 0.45,  0.40];
        A = [-0.45,  0.40];
end
        

dataTable = import_CSV_Table(filename);

time = dataTable.time;
time = time-time(1);
magPose = dataTable{:,2:7};
magField = dataTable{:,17:end};
EE_position = dataTable{:,8:10};
EE_velocity = dataTable{:,11:13};
velocity = zeros(Npt,Npt,2);

R = A;

for i = 1:Npt
    for j = 1:Npt
        EE = [x(i,j), y(i,j)];
        velocity(i,j,:) = (R-EE)/norm(R-EE)*fun1(R,EE) + (EE-M)/norm(EE-M)*fun2(EE,M);
    end
end

%% Processing Recorded Data

isOutliers = magPose(:,end) == -1;
magPose(isOutliers,:) = nan;

isZero = EE_position(:,1) == 0;
EE_position(isZero,:) = nan;
EE_velocity(isZero,:) = nan;

[pks,locs] = findpeaks(EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);
[pks2,locs2] = findpeaks(-EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);

figure
hold on
plot(time,EE_position(:,2))
plot(time(locs),pks,'r*')
plot(time,-EE_position(:,2))
plot(time(locs2),pks2,'k*')
hold off

numSamplesInterval = diff(locs);
maxNumSamplesInterval = max(numSamplesInterval);
normalizedTime = linspace(0,1,maxNumSamplesInterval);
nTrials = numel(locs)-1;
nSamplFiltering = 100;

resampledTrials_x = zeros(maxNumSamplesInterval,nTrials);
resampledTrials_y = resampledTrials_x;
resampledTrials_z = resampledTrials_x;
resampledTrials_vx = resampledTrials_x;
resampledTrials_vy = resampledTrials_x;
resampledTrials_vz = resampledTrials_x;
xq = normalizedTime';

for i = 1:nTrials
    switch direction
        case 1
            xt = time(locs2(i):locs(i+1));
            xt = xt-xt(1);
            xt = xt/max(xt);
        
            % for EEx
            yt = EE_position(locs2(i):locs(i+1),1);
            resampledTrials_x(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
            yt = EE_velocity(locs2(i):locs(i+1),1);
            resampledTrials_vx(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
        
            % for EEy
            yt = EE_position(locs2(i):locs(i+1),2);
            resampledTrials_y(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
            yt = EE_velocity(locs2(i):locs(i+1),2);
            resampledTrials_vy(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
        
            % for EEz
            yt = EE_position(locs2(i):locs(i+1),3);
            resampledTrials_z(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
            yt = EE_velocity(locs2(i):locs(i+1),3);
            resampledTrials_vz(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
        case 2
            xt = time(locs(i):locs2(i));
            xt = xt-xt(1);
            xt = xt/max(xt);
        
            % for EEx
            yt = EE_position(locs(i):locs2(i),1);
            resampledTrials_x(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
            yt = EE_velocity(locs(i):locs2(i),1);
            resampledTrials_vx(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
        
            % for EEy
            yt = EE_position(locs(i):locs2(i),2);
            resampledTrials_y(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
            yt = EE_velocity(locs(i):locs2(i),2);
            resampledTrials_vy(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
        
            % for EEz
            yt = EE_position(locs(i):locs2(i),3);
            resampledTrials_z(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
            yt = EE_velocity(locs(i):locs2(i),3);
            resampledTrials_vz(:,i) = smooth(pchip(xt,yt,xq),nSamplFiltering);
    end
end

figure
subplot(131)
plot(normalizedTime,resampledTrials_x);
subplot(132)
plot(normalizedTime,resampledTrials_y);
subplot(133)
plot(normalizedTime,resampledTrials_z);


%%

% resam4Max = abs(resampledTrials_z(:,1));
% [pks,locs] = findpeaks(resam4Max,'MinPeakHeight',0.5);

figure("Units","normalized","Position",[0.3 0.3 0.15 0.3])
% hold on
% plot(normalizedTime,resampledTrials_x,"Color",[0 0.4470 0.7410]);
yyaxis left
hold on
plot(normalizedTime,resampledTrials_y,"-","Color",[0 0.4470 0.7410]);
ylim([-0.5 0.5])
yticks(-0.5:0.25:0.5)
% hold off
yyaxis right
hold on
plot(normalizedTime,resampledTrials_z,"-","Color",[0.8500 0.3250 0.0980]);
% xline(normalizedTime(locs),"k--")
ylim([0.25 0.65])
yticks(0.25:0.1:0.65)
hold off
xlabel("normalized completion time")
% hold off

figure("Units","normalized","Position",[0.3 0.3 0.3 0.3])
% hold on
% plot(normalizedTime,resampledTrials_x,"Color",[0 0.4470 0.7410]);
yyaxis left
hold on
plot(normalizedTime,resampledTrials_vy,"-","Color",[0 0.4470 0.7410]);
hold off
switch direction
    case 1
        ylim([-0.04 0.12])
        yticks(-0.04:0.04:0.12)
    case 2
        ylim([-0.12 0.04])
        yticks(-0.12:0.04:0.04)
end
yyaxis right
hold on
plot(normalizedTime,resampledTrials_vz,"-","Color",[0.8500 0.3250 0.0980]);
% xline(normalizedTime(locs),"k--")
hold off
ylim([-0.08 0.08])
yticks(-0.08:0.04:0.08)
xlabel("normalized completion time")
% hold off


%% Generate virtual vector field

minV = min(velocity(:));
maxV = max(velocity(:));

dDetector = 0.3;

circX = M(1) + dDetector*cos(linspace(0,2*pi,50));
circY = M(2) + dDetector*sin(linspace(0,2*pi,50));

dDetectorPolicy = 0.25;

circX2 = M(1) + dDetectorPolicy*cos(linspace(0,2*pi,50));
circY2 = M(2) + dDetectorPolicy*sin(linspace(0,2*pi,50));

figure
subplot(131)
hold on
surf(x,y,velocity(:,:,1),"EdgeColor","none")
scatter3(A(1),A(2),maxV+1,15,"k","filled")
scatter3(B(1),B(2),maxV+1,15,"k","filled")
scatter3(M(1),M(2),maxV+1,15,"r","filled")
hold off
axis equal
axis off
colorbar
clim([minV maxV])
view(2)
subplot(132)
hold on
surf(x,y,velocity(:,:,2),"EdgeColor","none")
scatter3(A(1),A(2),maxV+1,15,"k","filled")
scatter3(B(1),B(2),maxV+1,15,"k","filled")
scatter3(M(1),M(2),maxV+1,15,"r","filled")
hold off
axis equal
axis off
view(2)
colorbar
clim([minV maxV])
subplot(133)
hold on
streamslice(x,y,velocity(:,:,1),velocity(:,:,2))
plot(circX,circY,"k")
plot(circX2,circY2,"k--")
scatter(A(1),A(2),15,"k","filled")
scatter(B(1),B(2),15,"k","filled")
scatter(M(1),M(2),15,"r","filled")
hold off
axis equal
axis off

%%

figure
hold on
vNorm = sqrt(velocity(:,:,1).^2+velocity(:,:,2).^2);
contourf(x,y,vNorm,100,"EdgeColor","none")
colormap("turbo")
clim([0 0.35])
l = streamslice(x,y,velocity(:,:,1),velocity(:,:,2),"cubic");
set(l,'Color','k');
plot(circX,circY,"k")
scatter(A(1),A(2),35,"k","filled")
scatter(B(1),B(2),35,"k","filled")
scatter(M(1),M(2),15,"k","filled")
plot(resampledTrials_y,resampledTrials_z,'r','LineWidth',2)
hold off
axis equal
axis off

%%

function v = fun1(vec1,vec2)

    Vmotion = 0.10;
    D = 0.025;% convergence factor
    v = Vmotion*(1-exp(-norm(vec1-vec2)/D));

end

function v = fun2(vec1,vec2)

    Vmotion = 0.25;
    D = 0.15;% convergence factor
    if norm(vec1-vec2) < 0.3
        v = Vmotion*(exp(-norm(vec1-vec2)/D));
    else
        v = 0;
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
