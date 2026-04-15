%% FILE SUMMARY
% Purpose: Run statistical analysis for radius-based avoidance experiments.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Signal Processing Toolbox; Statistics and Machine Learning Toolbox; Curve Fitting Toolbox; local packages dependencies/dabarplot.m, dependencies/swtest.m, dependencies/Levenetest.m, dependencies/SpectralArcLength.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: CSV logs in data/magnetic_data and data/session_fede_2_novembre.
% Outputs: SPARC metrics, normality/homoscedasticity checks, nonparametric tests, and figures.
% Run Notes: Configure direction and plotType to reproduce target analysis panels.


%% NOTE for CODE
% This code perform the statistical analysis on the collision avoidance data.
% Furthermore it computes additional metrics for data analysis, such as the
% Arc Length and SPARC index.
%
% For plotting, choose the value of direction = 1,2 to visualize trials in
% which the end-effector was moving from right to left or from left to
% right, respectively.
% Also, choose plotType = "2D" or "3D" to have 2d or 3d plots,
% respectively.

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

%% General Settings of Variables

waypoints = [0.466457,   0.45,  0.40
             0.466457,  -0.45,  0.40];

magnetPosition = [];
Ts = 1e-3; % sampling period

direction = 1; % 1:from right to left, 2: from left to right

% colors for plotting
Color = [0      0.4470 0.7410
         0.8500 0.3250 0.0980
         0.9290 0.6940 0.1250
         0.4940 0.1840 0.5560
         0.4660 0.6740 0.1880
         0.3010 0.7450 0.9330];

plotType = "2D"; % options: "2D", "3D"

%% DATA LOADING, SEGMENTATION and FIRST PLOTTING
% This part of code follows the strcture of Avoidance_data_analysis_01.m.

figure
hold on

for j = 1:6

    switch j
        case 1
            filename = "Avoidance_REAL1_exp_09_07_15_57_32.csv";
            magGT = [0.466 -0.215 0.3265 0 1 0];
        case 2
            filename = "Avoidance_REAL_2_exp_09_07_17_17_08.csv";
            magGT = [0.466 -0.01 0.3265 0 1 0];
        case 3
            filename = "Avoidance_REAL_3_exp_09_07_18_19_40.csv";
            magGT = [0.466 0.194 0.3265 0 1 0];
        case 4
            filename = "Avoidance_VIRTUAL1_exp_09_07_16_11_46.csv";
            magGT = [0.466 -0.215 0.3265 0 1 0];
        case 5
            filename = "Avoidance_VIRTUAL2_exp_09_07_17_02_15.csv";
            magGT = [0.466 -0.01 0.3265 0 1 0];
        case 6
            filename = "Avoidance_VIRTUAL_3_exp_09_07_18_07_01.csv";
            magGT = [0.466 0.194 0.3265 0 1 0];
    end

    % custom function to read data from csv file (see end of this code)
    dataTable = import_CSV_Table(filename);

    time = dataTable.time;
    
    time = time-time(1);                % reset time beginning
    magPose = dataTable{:,2:7};         % get magnet pose given by tracking algorithm
    magField = dataTable{:,17:end};     % get magnetic field readings recorded by the sensors (3 recordings for 4 sensors)
    EE_position = dataTable{:,8:10};    % get end-effector position
    EE_velocity = dataTable{:,11:13};   % get end-effector velocity

    % Data Display
    isOutlier = magPose(:,end) == -1;   % detect wheter the tracking was performed or not
    magPose(isOutlier,:) = nan;

    isZero = EE_position(:,1) == 0;     % to discard first data written in the file
    EE_position(isZero,:) = nan;
    EE_velocity(isZero,:) = nan;

    % trial segmentation is based on the detection of peaks in the position
    % of the end-effector (in particular along the direction of motion,
    % i.e. the y-axis).
    [pks,locs] = findpeaks(EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);
    [pks2,locs2] = findpeaks(-EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);

    % this gives automatically the length of the trials
    numSamplesInterval = diff(locs);
    maxNumSamplesInterval = max(numSamplesInterval);
    % construction of a normalized time axis
    normalizedTime = linspace(0,1,maxNumSamplesInterval);
    nTrials = numel(locs)-1;
    % number of samples for smoothing
    nSamplFiltering = 100;

    % preallocation of arrays to store resampled data in the common
    % normalized time axis
    resampledTrials_x = zeros(maxNumSamplesInterval,nTrials-1);
    resampledTrials_vx = resampledTrials_x;
    resampledTrials_y = resampledTrials_x;
    resampledTrials_vy = resampledTrials_x;
    resampledTrials_z = resampledTrials_x;
    resampledTrials_vz = resampledTrials_x;
    % common normalized time axis
    xq = normalizedTime';

    for i = 1:nTrials

        % definition of the time mask to isolate the desired trial portion,
        % i.e., the motion from right to left or from left to right of the
        % end-effector.
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

        % COMPUTATION OF SPARK INDEX

        speed = vecnorm([EE_velocity(timeMask,1) ...
                         EE_velocity(timeMask,2) ...
                         EE_velocity(timeMask,3)],2,2);
        SPARC(j).value1(i) = SpectralArcLength( speed, Ts );
        

        % RESAMPLING

        % for EEx
        y = EE_position(timeMask,1);
        resampledTrials_x(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);
        % for vEEx
        y = EE_velocity(timeMask,1);
        resampledTrials_vx(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEy
        y = EE_position(timeMask,2);
        resampledTrials_y(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);
        % for vEEy
        y = EE_velocity(timeMask,2);
        resampledTrials_vy(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEz
        y = EE_position(timeMask,3);
        resampledTrials_z(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);
        % for vEEx
        y = EE_velocity(timeMask,3);
        resampledTrials_vz(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        resampledNorm = vecnorm([resampledTrials_x(:,i) - magGT(1) ...
                                 resampledTrials_y(:,i) - magGT(2) ...
                                 resampledTrials_z(:,i) - magGT(3)],2,2);

        % store the minimum distance between the end-effector and the
        % magnet.
        minDistances(j).values(i) = min(resampledNorm);
    end

    TrialsData(j).resampledTrials_x = resampledTrials_x;
    TrialsData(j).resampledTrials_y = resampledTrials_y;
    TrialsData(j).resampledTrials_z = resampledTrials_z;

    TrialsData(j).resampledTrials_vx = resampledTrials_vx;
    TrialsData(j).resampledTrials_vy = resampledTrials_vy;
    TrialsData(j).resampledTrials_vz = resampledTrials_vz;

    % ------------------------------------------------------------------
    % to repeat analyses on the other side
    if direction == 1
        direction = 2;
    else
        direction = 1;
    end

    resampledTrials_x_nomem = zeros(maxNumSamplesInterval,nTrials-1);
    resampledTrials_y_nomem = resampledTrials_x;
    resampledTrials_z_nomem = resampledTrials_x;
    resampledTrials_vx_nomem = resampledTrials_x;
    resampledTrials_vy_nomem = resampledTrials_x;
    resampledTrials_vz_nomem = resampledTrials_x;

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

        speed = vecnorm([EE_velocity(timeMask,1) ...
                         EE_velocity(timeMask,2) ...
                         EE_velocity(timeMask,3)],2,2);
        SPARC(j).value2(i) = SpectralArcLength( speed, Ts );

        % for EEx
        y = EE_position(timeMask,1);
        resampledTrials_x_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);
        % for vEEx
        y = EE_velocity(timeMask,1);
        resampledTrials_vx_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEy
        y = EE_position(timeMask,2);
        resampledTrials_y_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);
        % for vEEy
        y = EE_velocity(timeMask,2);
        resampledTrials_vy_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        % for EEz
        y = EE_position(timeMask,3);
        resampledTrials_z_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);
        % for vEEz
        y = EE_velocity(timeMask,3);
        resampledTrials_vz_nomem(:,i) = smooth(pchip(x,y,xq),nSamplFiltering);

        resampledNorm = vecnorm([resampledTrials_x_nomem(:,i) - magGT(1) ...
                                 resampledTrials_y_nomem(:,i) - magGT(2) ...
                                 resampledTrials_z_nomem(:,i) - magGT(3)],2,2);

        minDistances(j).values(nTrials+i) = min(resampledNorm);
    end

    TrialsData(j).resampledTrials_x2 = resampledTrials_x_nomem;
    TrialsData(j).resampledTrials_y2 = resampledTrials_y_nomem;
    TrialsData(j).resampledTrials_z2 = resampledTrials_z_nomem;
    TrialsData(j).resampledTrials_vx2 = resampledTrials_vx_nomem;
    TrialsData(j).resampledTrials_vy2 = resampledTrials_vy_nomem;
    TrialsData(j).resampledTrials_vz2 = resampledTrials_vz_nomem;

    % re flip direction
    if direction == 1
        direction = 2;
    else
        direction = 1;
    end

    R_avoid = 0.18;
    angles = linspace(0,2*pi,100);

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

TrialsData(3).resampledTrials_x(:,1) = [];
TrialsData(3).resampledTrials_y(:,1) = [];
TrialsData(3).resampledTrials_z(:,1) = [];

switch plotType
    case "2D"
        scatter(waypoints(:,2),waypoints(:,3),35,'filled','k')
        hold off
        axis([-0.4 0.4 0 0.75])
        axis equal
    case "3D"
        scatter3(waypoints(:,1),waypoints(:,2),waypoints(:,3),35,'filled','k')
        hold off
        axis([0.4 0.5 -0.5 0.5 0 1])
        axis equal
        axis off
        view(40,25)
    otherwise
end

%%

minDistArray = [minDistances(1).values([1:7 15:21])
                minDistances(2).values([1:3 24:34]) 
                minDistances(3).values([4:10 16:18 26:28 30])
                minDistances(4).values(1:14)
                minDistances(5).values(1:14)
                minDistances(6).values(15:end)]';

figure("Units","normalized","Position",[0.3 0.3 0.15 0.3])
h = dabarplot(minDistArray,'errorbars','SD',...
    'barspacing',0.8);

%% Statistical Tests

X = minDistArray;
alpha = 0.05;

try
    if(test)

        for i = 1:6
            [H, pValue, W] = swtest(X(:,i), alpha);
        end

        groups = [repmat(1,14,1); repmat(2,14,1); repmat(3,14,1); repmat(4,14,1); repmat(5,14,1); repmat(6,14,1)];

        Xvert = [X(:) groups];

        % Levenetest(Xvert,alpha);

        Xkruskal = X(:,1:3);
        [p,tbl,stats] = friedman(Xkruskal);

        [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni");

        XWilc = [X(:,1) X(:,4); X(:,2) X(:,5); X(:,3) X(:,6)];
        figure
        h = dabarplot(XWilc,'errorbars','SD',...
            'barspacing',0.8);

        [p,h,stats] = ranksum(X(:,1),X(:,4));
        [p,h,stats] = ranksum(X(:,2),X(:,5));
        [p,h,stats] = ranksum(X(:,3),X(:,6));

        [p,h,stats] = ranksum(reshape(X(:,1:3),[],1),reshape(X(:,4:6),[],1));
    end
catch
    for i = 1:6
        disp("Testing Normality Distribution of " + num2str(i));
        [H, pValue, W] = swtest(X(:,i), alpha)
    end

    groups = [repmat(1,14,1); repmat(2,14,1); repmat(3,14,1); repmat(4,14,1); repmat(5,14,1); repmat(6,14,1)];

    Xvert = [X(:) groups];

    Levenetest(Xvert,alpha)

    Xkruskal = X(:,1:3);
    [p,tbl,stats] = friedman(Xkruskal)

    [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni")

    XWilc = [X(:,1) X(:,4); X(:,2) X(:,5); X(:,3) X(:,6)];
    figure
    h = dabarplot(XWilc,'errorbars','SD',...
        'barspacing',0.8);

    [p,h,stats] = ranksum(X(:,1),X(:,4))
    [p,h,stats] = ranksum(X(:,2),X(:,5))
    [p,h,stats] = ranksum(X(:,3),X(:,6))

    [p,h,stats] = ranksum(reshape(X(:,1:3),[],1),reshape(X(:,4:6),[],1))

    disp(abs(median(XWilc(:,1)) - mean(XWilc(:,2))))
end

%%

figure
subplot(3,2,1)
hold on
plot3(TrialsData(1).resampledTrials_x,TrialsData(1).resampledTrials_y,TrialsData(1).resampledTrials_z)
plot3(TrialsData(1).resampledTrials_x2,TrialsData(1).resampledTrials_y2,TrialsData(1).resampledTrials_z2)
hold off
axis equal
subplot(3,2,3)
hold on
plot3(TrialsData(2).resampledTrials_x,TrialsData(2).resampledTrials_y,TrialsData(2).resampledTrials_z)
plot3(TrialsData(2).resampledTrials_x2,TrialsData(2).resampledTrials_y2,TrialsData(2).resampledTrials_z2)
hold off
axis equal
subplot(3,2,5)
hold on
plot3(TrialsData(3).resampledTrials_x,TrialsData(3).resampledTrials_y,TrialsData(3).resampledTrials_z)
plot3(TrialsData(3).resampledTrials_x2,TrialsData(3).resampledTrials_y2,TrialsData(3).resampledTrials_z2)
hold off
axis equal
subplot(3,2,2)
hold on
plot3(TrialsData(4).resampledTrials_x,TrialsData(4).resampledTrials_y,TrialsData(4).resampledTrials_z)
plot3(TrialsData(4).resampledTrials_x2,TrialsData(4).resampledTrials_y2,TrialsData(4).resampledTrials_z2)
hold off
axis equal
subplot(3,2,4)
hold on
plot3(TrialsData(5).resampledTrials_x,TrialsData(5).resampledTrials_y,TrialsData(5).resampledTrials_z)
plot3(TrialsData(5).resampledTrials_x2,TrialsData(5).resampledTrials_y2,TrialsData(5).resampledTrials_z2)
hold off
axis equal
subplot(3,2,6)
hold on
plot3(TrialsData(6).resampledTrials_x,TrialsData(6).resampledTrials_y,TrialsData(6).resampledTrials_z)
plot3(TrialsData(6).resampledTrials_x2,TrialsData(6).resampledTrials_y2,TrialsData(6).resampledTrials_z2)
hold off
axis equal

minDataSegment = min([size(TrialsData(1).resampledTrials_x,1) ...
                      size(TrialsData(2).resampledTrials_x,1) ...
                      size(TrialsData(3).resampledTrials_x,1) ...
                      size(TrialsData(4).resampledTrials_x,1) ...
                      size(TrialsData(5).resampledTrials_x,1) ...
                      size(TrialsData(6).resampledTrials_x,1)]);

xq = linspace(0,1,minDataSegment);

for j = 1:6

    currentSegmentLength = size(TrialsData(j).resampledTrials_x,1);
    x = linspace(0,1,currentSegmentLength);

    numdataSegments = size(TrialsData(j).resampledTrials_x,2);

    for i = 1:numdataSegments
        y = TrialsData(j).resampledTrials_x(:,i);
        yq = pchip(x,y,xq);
        TrialsDataResampled(j).resampledTrials_x(:,i) = yq;

        y = TrialsData(j).resampledTrials_x2(:,i);
        yq = pchip(x,y,xq);
        TrialsDataResampled(j).resampledTrials_x2(:,i) = yq;

        y = TrialsData(j).resampledTrials_y(:,i);
        yq = pchip(x,y,xq);
        TrialsDataResampled(j).resampledTrials_y(:,i) = yq;

        y = TrialsData(j).resampledTrials_y2(:,i);
        yq = pchip(x,y,xq);
        TrialsDataResampled(j).resampledTrials_y2(:,i) = yq;

        y = TrialsData(j).resampledTrials_z(:,i);
        yq = pchip(x,y,xq);
        TrialsDataResampled(j).resampledTrials_z(:,i) = yq;

        y = TrialsData(j).resampledTrials_z2(:,i);
        yq = pchip(x,y,xq);
        TrialsDataResampled(j).resampledTrials_z2(:,i) = yq;
    end
    
end

figure
subplot(3,2,1)
hold on
plot3(TrialsDataResampled(1).resampledTrials_x,TrialsDataResampled(1).resampledTrials_y,TrialsDataResampled(1).resampledTrials_z)
plot3(TrialsDataResampled(1).resampledTrials_x2,TrialsDataResampled(1).resampledTrials_y2,TrialsDataResampled(1).resampledTrials_z2)
hold off
axis equal
subplot(3,2,3)
hold on
plot3(TrialsDataResampled(2).resampledTrials_x,TrialsDataResampled(2).resampledTrials_y,TrialsDataResampled(2).resampledTrials_z)
plot3(TrialsDataResampled(2).resampledTrials_x2,TrialsDataResampled(2).resampledTrials_y2,TrialsDataResampled(2).resampledTrials_z2)
hold off
axis equal
subplot(3,2,5)
hold on
plot3(TrialsDataResampled(3).resampledTrials_x,TrialsDataResampled(3).resampledTrials_y,TrialsDataResampled(3).resampledTrials_z)
plot3(TrialsDataResampled(3).resampledTrials_x2,TrialsDataResampled(3).resampledTrials_y2,TrialsDataResampled(3).resampledTrials_z2)
hold off
axis equal
subplot(3,2,2)
hold on
plot3(TrialsDataResampled(4).resampledTrials_x,TrialsDataResampled(4).resampledTrials_y,TrialsDataResampled(4).resampledTrials_z)
plot3(TrialsDataResampled(4).resampledTrials_x2,TrialsDataResampled(4).resampledTrials_y2,TrialsDataResampled(4).resampledTrials_z2)
hold off
axis equal
subplot(3,2,4)
hold on
plot3(TrialsDataResampled(5).resampledTrials_x,TrialsDataResampled(5).resampledTrials_y,TrialsDataResampled(5).resampledTrials_z)
plot3(TrialsDataResampled(5).resampledTrials_x2,TrialsDataResampled(5).resampledTrials_y2,TrialsDataResampled(5).resampledTrials_z2)
hold off
axis equal
subplot(3,2,6)
hold on
plot3(TrialsDataResampled(6).resampledTrials_x,TrialsDataResampled(6).resampledTrials_y,TrialsDataResampled(6).resampledTrials_z)
plot3(TrialsDataResampled(6).resampledTrials_x2,TrialsDataResampled(6).resampledTrials_y2,TrialsDataResampled(6).resampledTrials_z2)
hold off
axis equal

figure
hold on
Curve2 = [TrialsDataResampled(4).resampledTrials_x(:,1) ...
          TrialsDataResampled(4).resampledTrials_y(:,1) ...
          TrialsDataResampled(4).resampledTrials_z(:,1)];
plot3(Curve2(:,1),Curve2(:,2),Curve2(:,3))
for i = 1:size(TrialsDataResampled(1).resampledTrials_x,2)
    Curve1 = [TrialsDataResampled(1).resampledTrials_x(:,i) ...
              TrialsDataResampled(1).resampledTrials_y(:,i) ...
              TrialsDataResampled(1).resampledTrials_z(:,i)];

    R2 = sum( (dot(Curve1,Curve2,2))./(vecnorm(Curve1,2,2).*vecnorm(Curve2,2,2)) )/size(Curve1,1);
    TrialsDataResampled(1).R2(i) = R2;
    plot3(Curve1(:,1),Curve1(:,2),Curve1(:,3))
end
axis equal
hold off

%% ANALYSIS ON THE ARC LENGTH

% normalized time axis
t = linspace(0,1,length(TrialsDataResampled(1).resampledTrials_x(:,1)));
% time differential
dt = diff(t)';
% space differentials
dx_dt = diff(TrialsDataResampled(4).resampledTrials_x(:,1))./dt;
dy_dt = diff(TrialsDataResampled(4).resampledTrials_y(:,1))./dt;
dz_dt = diff(TrialsDataResampled(4).resampledTrials_z(:,1))./dt;

% computation of arclength for ground truth reference (Robot motion with no tracking)
s_GT(:,1) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
% [0; 1] normalization
s_GT(:,1) = (s_GT(:,1)-min(s_GT(:,1)))/range(s_GT(:,1));

for i = 1:size(TrialsDataResampled(1).resampledTrials_x,2)
    dx_dt = diff(TrialsDataResampled(1).resampledTrials_x(:,i))./dt;
    dy_dt = diff(TrialsDataResampled(1).resampledTrials_y(:,i))./dt;
    dz_dt = diff(TrialsDataResampled(1).resampledTrials_z(:,i))./dt;
    s1(:,i) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
    s1(:,i) = (s1(:,i)-min(s1(:,i)))/range(s1(:,i));
end

figure
subplot(121)
plot(TrialsDataResampled(1).resampledTrials_y,TrialsDataResampled(1).resampledTrials_z)
title("Magnet Left - EE from right to left")
subplot(122)
hold on
plot(t,s1)
plot(t,s_GT(:,1),'k','LineWidth',1.2)
hold off
title("Magnet Left - EE from right to left")
ylabel("Arc Length (-)")
xlabel("Normalized Completion Time (-)")

% --------------- %

dx_dt = diff(TrialsDataResampled(5).resampledTrials_x(:,1))./dt;
dy_dt = diff(TrialsDataResampled(5).resampledTrials_y(:,1))./dt;
dz_dt = diff(TrialsDataResampled(5).resampledTrials_z(:,1))./dt;

s_GT(:,2) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
s_GT(:,2) = (s_GT(:,2)-min(s_GT(:,2)))/range(s_GT(:,2));

for i = 1:size(TrialsDataResampled(2).resampledTrials_x,2)
    dx_dt = diff(TrialsDataResampled(2).resampledTrials_x(:,i))./dt;
    dy_dt = diff(TrialsDataResampled(2).resampledTrials_y(:,i))./dt;
    dz_dt = diff(TrialsDataResampled(2).resampledTrials_z(:,i))./dt;
    s2(:,i) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
    s2(:,i) = (s2(:,i)-min(s2(:,i)))/range(s2(:,i));
end

figure
subplot(121)
plot(TrialsDataResampled(2).resampledTrials_y,TrialsDataResampled(2).resampledTrials_z)
title("Magnet center - EE from right to left")
subplot(122)
hold on
plot(t,s2)
plot(t,s_GT(:,2),'k','LineWidth',1.2)
hold off
title("Magnet center - EE from right to left")
ylabel("Arc Length (-)")
xlabel("Normalized Completion Time (-)")

% --------------- %

dx_dt = diff(TrialsDataResampled(6).resampledTrials_x(:,1))./dt;
dy_dt = diff(TrialsDataResampled(6).resampledTrials_y(:,1))./dt;
dz_dt = diff(TrialsDataResampled(6).resampledTrials_z(:,1))./dt;

s_GT(:,3) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
s_GT(:,3) = (s_GT(:,3)-min(s_GT(:,3)))/range(s_GT(:,3));

for i = 1:size(TrialsDataResampled(3).resampledTrials_x,2)
    dx_dt = diff(TrialsDataResampled(3).resampledTrials_x(:,i))./dt;
    dy_dt = diff(TrialsDataResampled(3).resampledTrials_y(:,i))./dt;
    dz_dt = diff(TrialsDataResampled(3).resampledTrials_z(:,i))./dt;
    s3(:,i) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
    s3(:,i) = (s3(:,i)-min(s3(:,i)))/range(s3(:,i));
end

figure
subplot(121)
plot(TrialsDataResampled(3).resampledTrials_y,TrialsDataResampled(3).resampledTrials_z)
title("Magnet Right - EE from right to left")
subplot(122)
hold on
plot(t,s3)
plot(t,s_GT(:,3),'k','LineWidth',1.2)
hold off
title("Magnet Right - EE from right to left")
ylabel("Arc Length (-)")
xlabel("Normalized Completion Time (-)")

% ----------------- %

dx_dt = diff(TrialsDataResampled(4).resampledTrials_x2(:,1))./dt;
dy_dt = diff(TrialsDataResampled(4).resampledTrials_y2(:,1))./dt;
dz_dt = diff(TrialsDataResampled(4).resampledTrials_z2(:,1))./dt;

s_GT(:,4) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
s_GT(:,4) = (s_GT(:,4)-min(s_GT(:,4)))/range(s_GT(:,4));

for i = 1:size(TrialsDataResampled(1).resampledTrials_x2,2)
    dx_dt = diff(TrialsDataResampled(1).resampledTrials_x2(:,i))./dt;
    dy_dt = diff(TrialsDataResampled(1).resampledTrials_y2(:,i))./dt;
    dz_dt = diff(TrialsDataResampled(1).resampledTrials_z2(:,i))./dt;
    s4(:,i) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
    s4(:,i) = (s4(:,i)-min(s4(:,i)))/range(s4(:,i));
end

figure
subplot(121)
plot(TrialsDataResampled(1).resampledTrials_y2,TrialsDataResampled(1).resampledTrials_z2)
title("Magnet Left - EE from left to right")
subplot(122)
hold on
plot(t,s4)
plot(t,s_GT(:,4),'k','LineWidth',1.2)
hold off
title("Magnet Left - EE from left to right")
ylabel("Arc Length (-)")
xlabel("Normalized Completion Time (-)")

% --------------- %

dx_dt = diff(TrialsDataResampled(5).resampledTrials_x2(:,1))./dt;
dy_dt = diff(TrialsDataResampled(5).resampledTrials_y2(:,1))./dt;
dz_dt = diff(TrialsDataResampled(5).resampledTrials_z2(:,1))./dt;

s_GT(:,5) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
s_GT(:,5) = (s_GT(:,5)-min(s_GT(:,5)))/range(s_GT(:,5));

for i = 1:size(TrialsDataResampled(2).resampledTrials_x2,2)
    dx_dt = diff(TrialsDataResampled(2).resampledTrials_x2(:,i))./dt;
    dy_dt = diff(TrialsDataResampled(2).resampledTrials_y2(:,i))./dt;
    dz_dt = diff(TrialsDataResampled(2).resampledTrials_z2(:,i))./dt;
    s5(:,i) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
    s5(:,i) = (s5(:,i)-min(s5(:,i)))/range(s5(:,i));
end

figure
subplot(211)
plot(TrialsDataResampled(2).resampledTrials_y2,TrialsDataResampled(2).resampledTrials_z2)
title("Magnet Center - EE from left to right")
subplot(212)
hold on
plot(t,s5)
plot(t,s_GT(:,5),'k','LineWidth',1.2)
hold off
title("Magnet Center - EE from left to right")
ylabel("Arc Length (-)")
xlabel("Normalized Completion Time (-)")

% --------------- %

dx_dt = diff(TrialsDataResampled(6).resampledTrials_x2(:,1))./dt;
dy_dt = diff(TrialsDataResampled(6).resampledTrials_y2(:,1))./dt;
dz_dt = diff(TrialsDataResampled(6).resampledTrials_z2(:,1))./dt;

s_GT(:,6) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
s_GT(:,6) = (s_GT(:,6)-min(s_GT(:,6)))/range(s_GT(:,6));

for i = 1:size(TrialsDataResampled(3).resampledTrials_x2,2)
    dx_dt = diff(TrialsDataResampled(3).resampledTrials_x2(:,i))./dt;
    dy_dt = diff(TrialsDataResampled(3).resampledTrials_y2(:,i))./dt;
    dz_dt = diff(TrialsDataResampled(3).resampledTrials_z2(:,i))./dt;
    s6(:,i) = cumtrapz(t,sqrt([0; dx_dt].^2 + [0; dy_dt].^2 + [0; dz_dt.^2]));
    s6(:,i) = (s6(:,i)-min(s6(:,i)))/range(s6(:,i));
end

figure
subplot(121)
plot(TrialsDataResampled(3).resampledTrials_y2,TrialsDataResampled(3).resampledTrials_z2)
title("Magnet Right - EE from left to right")
subplot(122)
hold on
plot(t,s6)
plot(t,s_GT(:,6),'k','LineWidth',1.2)
hold off
title("Magnet Right - EE from left to right")
ylabel("Arc Length (-)")
xlabel("Normalized Completion Time (-)")

%%

t = t(:);
stp = 50;

figure
subplot(131)
hold on
mean_vals = mean(s4, 2); % Calculate the mean across columns (observations)
std_vals = std(s4, 0, 2); % Calculate the standard deviation across columns

tpatch = [t; flipud(t)];
ypatch = [mean_vals + std_vals; flipud(mean_vals - std_vals)];
patch(tpatch(1:stp:end), ypatch(1:stp:end), ...
     [0.8, 0.8, 0.8],'EdgeColor', 'none')

%
mean_vals = mean(s4(end,:)-flipud(s1), 2); % Calculate the mean across columns (observations)
std_vals = std(s4(end,:)-flipud(s1), 0, 2); % Calculate the standard deviation across columns

ypatch = [mean_vals + std_vals; flipud(mean_vals - std_vals)];
patch(tpatch(1:stp:end), ypatch(1:stp:end), ...
     [0.8, 0.8, 0.8],'EdgeColor', 'none')

% Plot the mean line
plot(t(1:stp:end), s_GT(1:stp:end,4), 'b', 'LineWidth', 1); % Black solid line for the mean
% Plot the mean line
plot(t(1:stp:end), flipud(s_GT(end,4)-s_GT(1:stp:end,1)), 'r', 'LineWidth', 1); % Black solid line for the mean
hold off
axis equal
axis([0 1 0 1])


subplot(132)
hold on
mean_vals = mean(s5, 2); % Calculate the mean across columns (observations)
std_vals = std(s5, 0, 2); % Calculate the standard deviation across columns

ypatch = [mean_vals + std_vals; flipud(mean_vals - std_vals)];
patch(tpatch(1:stp:end), ypatch(1:stp:end), ...
     [0.8, 0.8, 0.8],'EdgeColor', 'none')
%
mean_vals = mean(s5(end,:)-flipud(s2), 2); % Calculate the mean across columns (observations)
std_vals = std(s5(end,:)-flipud(s2), 0, 2); % Calculate the standard deviation across columns

ypatch = [mean_vals + std_vals; flipud(mean_vals - std_vals)];
patch(tpatch(1:stp:end), ypatch(1:stp:end), ...
     [0.8, 0.8, 0.8],'EdgeColor', 'none')

% Plot the mean line
plot(t, s_GT(:,5), 'b', 'LineWidth', 1); % Black solid line for the mean
% Plot the mean line
plot(t, flipud(s_GT(end,5)-s_GT(:,2)), 'r', 'LineWidth', 1); % Black solid line for the mean
hold off
axis equal
axis([0 1 0 1])


subplot(133)
hold on
mean_vals = mean(s6, 2); % Calculate the mean across columns (observations)
std_vals = std(s6, 0, 2); % Calculate the standard deviation across columns

ypatch = [mean_vals + std_vals; flipud(mean_vals - std_vals)];
patch(tpatch(1:stp:end), ypatch(1:stp:end), ...
     [0.8, 0.8, 0.8],'EdgeColor', 'none')
%
mean_vals = mean(s6(end,:)-flipud(s3), 2); % Calculate the mean across columns (observations)
std_vals = std(s6(end,:)-flipud(s3), 0, 2); % Calculate the standard deviation across columns

ypatch = [mean_vals + std_vals; flipud(mean_vals - std_vals)];
patch(tpatch(1:stp:end), ypatch(1:stp:end), ...
     [0.8, 0.8, 0.8],'EdgeColor', 'none')

% Plot the mean line
plot(t, s_GT(:,6), 'b', 'LineWidth', 1); % Black solid line for the mean
% Plot the mean line
plot(t, flipud(s_GT(end,6)-s_GT(:,3)), 'r', 'LineWidth', 1); % Black solid line for the mean
hold off
axis equal
axis([0 1 0 1])

%%

SPARC_Array = [SPARC(1).value1(1:7) SPARC(1).value2(1:7);
               SPARC(2).value1(1:7) SPARC(2).value2(1:7);
               SPARC(3).value1(1:7) SPARC(3).value2(1:7);
               SPARC(4).value1(1:7) SPARC(4).value2(1:7);
               SPARC(5).value1(1:7) SPARC(5).value2(1:7);
               SPARC(6).value1(1:7) SPARC(6).value2(1:7)]';

SPARC_Complessive = [SPARC_Array(:,1) SPARC_Array(:,4);
                     SPARC_Array(:,2) SPARC_Array(:,5);
                     SPARC_Array(:,3) SPARC_Array(:,6)];

figure("Units","normalized","Position",[0.3 0.3 0.15 0.3])
subplot(121)
h2 = dabarplot(SPARC_Array(:,[1 4 2 5 3 6]),'errorbars','SD',...
    'barspacing',0.8);
subplot(122)
h3 = dabarplot(SPARC_Complessive,'errorbars','SD',...
    'barspacing',0.8);


try
    if(test)
        [p,tbl,stats] = friedman(SPARC_Array(:,1:3));
        [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni");
        [p,h,stats] = ranksum(SPARC_Complessive(:,1),SPARC_Complessive(:,2));
    end 
catch exception
    [p,tbl,stats] = friedman(SPARC_Array(:,1:3))

    [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni")
    
    [p,h,stats] = ranksum(SPARC_Complessive(:,1),SPARC_Complessive(:,2))
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