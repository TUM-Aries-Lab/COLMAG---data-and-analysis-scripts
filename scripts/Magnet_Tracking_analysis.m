%% FILE SUMMARY
% Purpose: Aggregate multi-session 3D tracking performance and computation-time trends.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Curve Fitting Toolbox; local package dependencies/daboxplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: Multiple CSV logs in data/session_fede_2_novembre.
% Outputs: Spatial error plots, distance-vs-error trends, and computation-time summaries.
% Run Notes: Includes fitted trend extraction for publication-quality performance curves.

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

%%

filename = "MagTrackData_session1.csv";
data1 = importfile(filename);
filename = "MagTrackData_session2.csv";
data2 = importfile(filename);
filename = "MagTrackData_session3.csv";
data3 = importfile(filename);

data = [data1; data2; data3];

%%

washoutData = 2000;

% extract number of data samples
data(end-washoutData:end,:) = [];
nSamples = size(data,1);
newSamples = 1;

% extract end-effector position
EE = data(:,8:10);

EE_new(1,:) = EE(1,:);
validDataMask = false(nSamples,1);

for i = 2:nSamples
    if ~isequal(EE(i,:),EE(i-1,:))
        newSamples = newSamples+1;
        EE_new(newSamples,:) = EE(i,:);
        validDataMask(i) = true;
    end
end

% extract time axis
time = data(validDataMask,1);
% extract computation time
cTime = data(validDataMask,41);
% extract coefficient of determination, R2
R2 = data(validDataMask,42);
% mag position detector
EE_magPosition = data(validDataMask,2:4);
EE_magOrientation = data(validDataMask,5:7)./vecnorm(data(validDataMask,5:7),2,2);

% make time axis start from zero
time = time-time(1);

%%

% ground Truth mag position
O_magPosition_GT = [0.54788 0.00006 0.32819];
O_magOrientation_GT = [0 1 0];
O_magPosition = EE_magPosition;%zeros(size(EE_magPosition));
O_magOrientation = EE_magOrientation;
isOutlier = (O_magPosition(:,1) == 0 | O_magPosition(:,1) == -1);
O_magPosition(isOutlier,:) = [];
O_magOrientation(isOutlier,:) = [];
R2(isOutlier) = [];
cTime(isOutlier) = [];

isFitGood = R2 > 0.95;

figure
subplot(1,2,1)
hold on
histogram(cTime*1000,'EdgeColor','none')
histogram(cTime(isFitGood)*1000,'EdgeColor','none')
ylabel("computation time (ms)")
hold off
subplot(1,2,2)
hold on
histogram(R2,'EdgeColor','none')
histogram(R2(isFitGood),'EdgeColor','none')
ylabel("coefficient of determination, R^2 (-)")
hold off

NUM_DATA = length(EE(validDataMask,1));
EE_plot3 = EE(validDataMask,:);
EE_plot3(EE_plot3(:,3) == 0,:) = [];
alphaVal = linspace(0.2,1,NUM_DATA-1);
figure
hold on

EE_valid = EE(validDataMask,:);
EE_good = EE_valid(~isOutlier,:);
scatter3(EE_plot3(:,1),EE_plot3(:,2),EE_plot3(:,3),4,"filled","MarkerFaceColor",[0.9290 0.6940 0.1250],"MarkerFaceAlpha",0.1)
scatter3(O_magPosition_GT(1),O_magPosition_GT(2),O_magPosition_GT(3),25,'filled',"k")
hold off
ylim([-0.5 0.5])
axis equal
view(135,30)

O_magPositionGood = O_magPosition(isFitGood,:);
O_magOrientationGood = O_magOrientation(isFitGood,:);

remainedPoints = size(O_magPosition,1);
remainedGoodPoints = size(O_magPositionGood,1);

O_magPosition_GT_rep = repmat(O_magPosition_GT,remainedPoints,1);
O_magPosition_GT_good_rep = repmat(O_magPosition_GT,remainedGoodPoints,1);
O_magOrientation_GT = O_magOrientationGood(1,:);
O_magOrientation_GT_rep = repmat(O_magOrientation_GT,remainedPoints,1);
O_magOrientation_GT_good_rep = repmat(O_magOrientation_GT,remainedGoodPoints,1);

distances = vecnorm(O_magPosition_GT_rep-EE_good,2,2);
O_magOrientation_GT = median(O_magOrientation(distances < 0.03,:));
O_magOrientation_GT = O_magOrientation_GT/norm(O_magOrientation_GT);
O_magOrientation_GT_rep = repmat(O_magOrientation_GT,remainedPoints,1);
O_magOrientation_GT_good_rep = repmat(O_magOrientation_GT,remainedGoodPoints,1);

E_position = O_magPosition-O_magPosition_GT_rep;
E_position_good = O_magPositionGood-O_magPosition_GT_good_rep;

E_orientation = dot(O_magOrientation,O_magOrientation_GT_rep,2);
E_orientation_good = dot(O_magOrientationGood,O_magOrientation_GT_good_rep,2);
axScale = 0.05;

figure(Units="normalized",Position=[0.1 0.3 0.8 0.15])

subplot(1,4,1)
hold on
histogram(E_position(:,1),'EdgeColor','none')
histogram(E_position_good(:,1),'EdgeColor','none')
xline(median(E_position(:,1)),'k--')
txt = "\leftarrow " + " " + num2str(median(E_position(:,1)));
text(median(E_position(:,1)),size(E_position(:,1),1)/10,txt)
xlim([-1 1]*axScale)
hold off

subplot(1,4,2)
hold on
histogram(E_position(:,2),'EdgeColor','none')
histogram(E_position_good(:,2),'EdgeColor','none')
xline(median(E_position(:,2)),'k--')
txt = "\leftarrow " + " " + num2str(median(E_position(:,2)));
text(median(E_position(:,2)),size(E_position(:,2),1)/10,txt)
hold off
xlim([-1 1]*axScale)

subplot(1,4,3)
hold on
histogram(E_position(:,3),'EdgeColor','none')
histogram(E_position_good(:,3),'EdgeColor','none')
xline(median(E_position(:,3)),'k--')
txt = "\leftarrow " + " " + num2str(median(E_position(:,3)));
text(median(E_position(:,3)),size(E_position(:,3),1)/10,txt)
xlim([-1 1]*axScale)
hold off

subplot(1,4,4)
hold on
histogram(vecnorm(E_position,2,2),'EdgeColor','none')
histogram(vecnorm(E_position(isFitGood,:),2,2),'EdgeColor','none')
xline(median(vecnorm(E_position,2,2)),'k--')
txt = num2str(median(vecnorm(E_position,2,2))) + " " + "\rightarrow";
text(median(vecnorm(E_position,2,2)),size(vecnorm(E_position,2,2),1)/10,txt,'HorizontalAlignment','right')
xlim([-1 1]*axScale)
hold off

%%

distances = vecnorm(O_magPosition_GT_rep-EE_good,2,2);

figure
subplot(2,1,1)
histogram(R2,'EdgeColor','none','Normalization','probability')
xlim([0.9 1])
subplot(2,1,2)
scatter(distances,R2,8,'filled','MarkerFaceAlpha',0.1)
xlim([0 0.35])
ylim([0.9 1])

m = min(distances);
M = max(distances);
NUMBINSGRAPH_FITTING = 25;
medianPointsFitting = zeros(NUMBINSGRAPH_FITTING,1);
distvectorFitting = medianPointsFitting;
minPointsFitting = medianPointsFitting;
maxPointsFitting = medianPointsFitting;
binEdgesFitting = linspace(m,M,NUMBINSGRAPH_FITTING+1);
for i = 1:NUMBINSGRAPH_FITTING
    mask = (distances >= binEdgesFitting(i)) & (distances <= binEdgesFitting(i+1));
    distvectorFitting(i) = (binEdgesFitting(i+1)+binEdgesFitting(i))/2;
    medianPointsFitting(i) = median(cTime(mask)*1000);
    minPointsFitting(i) = prctile(cTime(mask)*1000,25);
    maxPointsFitting(i) = prctile(cTime(mask)*1000,75);
end

%% Computation Time Picture Elements

% Set up fittype and options.
[xData, yData] = prepareCurveData( medianPointsFitting, distvectorFitting );
ft = fittype( 'poly1' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );
opts.Robust = 'Bisquare';

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

figure("Units","normalized","Position",[0.3 0.1 0.2 0.8])
subplot(3,1,1)
% yyaxis right
hold on
scatter(cTime*1000,distances,8,'filled','MarkerFaceAlpha',0.1)
errorbar(medianPointsFitting,distvectorFitting,[],[],medianPointsFitting-minPointsFitting,...
                                 maxPointsFitting-medianPointsFitting,'k.')
plot(linspace(min(xData),max(xData),100),fitresult(linspace(min(xData),max(xData),100)),'k--')
xlim([0 15])
hold off

subplot(3,1,2)
hold on
% yyaxis left
histogram(cTime*1000,'EdgeColor','none','Normalization','probability','BinWidth',0.25)
xline(median(cTime*1000),'k--')
txt = "\leftarrow" + " " +  num2str(round(median(cTime*1000)*100)/100);
text(median(cTime*1000),0.1,txt,'HorizontalAlignment','left')
hold off
xlim([0 15])
ylim([0 0.15])

subplot(3,1,3)
h = daboxplot(cTime*1000,'fill',1,...
    'outliers',0);
camroll(-90)
ylim([0 15])


%%

addpath(genpath("daboxplot"))

figure
subplot(2,1,1)
hold on
histogram(cTime*1000,'EdgeColor','none','Normalization','probability','BinWidth',0.25)
xline(median(cTime*1000),'k--')
txt = "\leftarrow" + " " +  num2str(round(median(cTime*1000)*100)/100);
text(median(cTime*1000),0.05,txt,'HorizontalAlignment','left')
hold off
xlim([0 20])

subplot(2,1,2)
h = daboxplot(cTime*1000,'fill',1,...
    'outliers',0);
camroll(-90)
ylim([0 20])


NUMBINSGRAPH = 15;
medianPoints = zeros(NUMBINSGRAPH,1);
minPoints = medianPoints;
maxPoints = medianPoints;
distvector   = medianPoints;
medianPointsOri = medianPoints;
minPointsOri = medianPoints;
maxPointsOri = medianPoints;
m = min(distances);
M = max(distances);
errors = vecnorm(E_position,2,2);
E_orientation(1) = 1;
errorsOri = acos(E_orientation)*180/pi;
binEdges = linspace(m,M,NUMBINSGRAPH+1);

for i = 1:NUMBINSGRAPH
    mask = (distances >= binEdges(i)) & (distances <= binEdges(i+1));
    distvector(i) = (binEdges(i+1)+binEdges(i))/2;
    medianPoints(i) = median(errors(mask));
    minPoints(i) = prctile(errors(mask),25);
    maxPoints(i) = prctile(errors(mask),75);
    medianPointsOri(i) = median(errorsOri(mask));
    minPointsOri(i) = prctile(errorsOri(mask),25);
    maxPointsOri(i) = prctile(errorsOri(mask),75);
end

NUMBINSGRAPH_FITTING = 25;
medianPointsFitting = zeros(NUMBINSGRAPH_FITTING,1);
distvectorFitting = medianPointsFitting;
minPointsFitting = medianPointsFitting;
maxPointsFitting = medianPointsFitting;
medianPointsFittingOri = medianPointsFitting;
minPointsFittingOri = medianPointsFitting;
maxPointsFittingOri = medianPointsFitting;
binEdgesFitting = linspace(m,M,NUMBINSGRAPH_FITTING+1);
for i = 1:NUMBINSGRAPH_FITTING
    mask = (distances >= binEdgesFitting(i)) & (distances <= binEdgesFitting(i+1));
    distvectorFitting(i) = (binEdgesFitting(i+1)+binEdgesFitting(i))/2;
    medianPointsFitting(i) = median(errors(mask));
    minPointsFitting(i) = prctile(errors(mask),25);
    maxPointsFitting(i) = prctile(errors(mask),75);
    medianPointsFittingOri(i) = median(errorsOri(mask));
    minPointsFittingOri(i) = prctile(errorsOri(mask),25);
    maxPointsFittingOri(i) = prctile(errorsOri(mask),75);
end

[xData, yData] = prepareCurveData( distvectorFitting, medianPointsFitting );

% Set up fittype and options.
ft = fittype( 'poly3' );

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft );


figure
hold on
% yyaxis left
scatter(vecnorm(O_magPosition_GT_rep-EE_good,2,2),vecnorm(E_position,2,2),8,"filled",...
    "MarkerFaceAlpha",0.1)
scatter(distvectorFitting,medianPointsFitting,25,'filled',"ko")
errorbar(distvectorFitting,medianPointsFitting,medianPointsFitting-minPointsFitting,...
                                 maxPointsFitting-medianPointsFitting,'k.')
axis equal
hold off
xlabel("Distance from Detector (m)")
ylabel("Tracking Error Position (m)")
axis([0 0.35 0 0.25])

figure
hold on
% yyaxis left
scatter(vecnorm(O_magPosition_GT_rep-EE_good,2,2),errorsOri,8,"filled",...
    "MarkerFaceAlpha",0.1)
scatter(distvectorFitting,medianPointsFittingOri,25,'filled',"ko")
errorbar(distvectorFitting,medianPointsFittingOri,medianPointsFittingOri-minPointsFittingOri,...
                                 maxPointsFittingOri-medianPointsFittingOri,'k.')
% axis equal
hold off
xlabel("Distance from Detector (m)")
ylabel("Tracking Error Orientation (deg)")
xlim([0 0.35])


%% Auxiliary Functions

function data = importfile(filename, dataLines)
% Auto-generated by MATLAB on 03-Nov-2024 08:18:28

% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 58);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "mag_pos_x", "mag_pos_y", "mag_pos_z", "mag_orien_x", "mag_orien_y", "mag_orien_z", "ee_x", "ee_y", "ee_z", "v_ee_x", "v_ee_y", "v_ee_z", "F_ee_x", "F_ee_y", "F_ee_z", "s1_x", "s1_y", "s1_z", "s2_x", "s2_y", "s2_z", "s3_x", "s3_y", "s3_z", "s4_x", "s4_y", "s4_z", "n_s1_x", "n_s1_y", "n_s1_z", "n_s2_x", "n_s2_y", "n_s2_z", "n_s3_x", "n_s3_y", "n_s3_z", "n_s4_x", "n_s4_y", "n_s4_z", "ls_compute_time", "r2", "O_T_EE_1", "O_T_EE_2", "O_T_EE_3", "O_T_EE_4", "O_T_EE_5", "O_T_EE_6", "O_T_EE_7", "O_T_EE_8", "O_T_EE_9", "O_T_EE_10", "O_T_EE_11", "O_T_EE_12", "O_T_EE_13", "O_T_EE_14", "O_T_EE_15", "O_T_EE_16"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);

% Convert to output type
data = table2array(data);
end