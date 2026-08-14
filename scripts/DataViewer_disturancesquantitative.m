%% FILE SUMMARY
% Purpose: Quantify disturbance rejection against common metallic/electromagnetic objects
%          (driller, crimper, hammer, caliper, wrenches, phone) approached near the detector,
%          and reproduce the peak field / peak R2 / average R2 / misclassification summary
%          reported in Table S2.
% Last Updated: 2026-08-10.
% Dependencies: MATLAB; Signal Processing Toolbox; Statistics and Machine Learning Toolbox (for table()).
% Data: CSV logs in data/disturbances.
% Outputs: Per-object field/R2 trace plots and the summaryTable reproducing Table S2.
% Run Notes: Each object's analysis window spans several end-effector approach/retreat cycles
%            (identified automatically via findpeaks on the end-effector position), not a single
%            brief exposure; see the "Exposure duration" note below.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath("data/disturbances")
        addpath("dependencies")
    end
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % operations over path
    currentPath = pwd;
    cd ..
    addpath("data/disturbances")
    cd(currentPath)
    addpath("../dependencies")
end

%%

fileNames = ["Demo_exp_07_11_18_23_53.csv"  % driller
             "Demo_exp_07_11_18_24_37.csv"  % crimper
             "Demo_exp_07_11_18_25_33.csv"  % hammer
             "Demo_exp_07_11_18_26_23.csv"  % caliper
             "Demo_exp_07_11_18_27_26.csv"  % wrenches
             "Demo_exp_07_11_18_28_55.csv"];  % phone

% Locs2Choose(i,:) selects the [start end] end-effector approach/retreat peaks (from
% findpeaks on EE_pos(:,2)) bounding the analysis window for object i.
Locs2Choose = [ 1 5
                1 5
                1 5
                2 6
                5 9
                1 5];

%%

for i = 1:size(fileNames,1)

    fileName = fileNames(i,:);
    data = importfile(fileName);

    mask = 100:size(data,1);
    time = data(mask,1)-data(mask(1),1);
    magPos = data(mask,2:7);
    EE_pos = data(mask,8:10);
    EE_vel = data(mask,11:13);
    Bfield = data(mask,17:28);
    Bfield_withNord = data(mask,29:40);
    compTime = data(mask,41);
    r_squared = data(mask,42);
    state  = data(mask,end);

    [pks,locs] = findpeaks(EE_pos(:,2));

    dataMask = locs(Locs2Choose(i,1)):locs(Locs2Choose(i,2));
    structPlot(i).EE_pos = EE_pos(dataMask,:);
    structPlot(i).EE_vel = EE_vel(dataMask,:);
    structPlot(i).Bfield = Bfield(dataMask,:);
    structPlot(i).R2     = r_squared(dataMask,:);
    structPlot(i).state  = state(dataMask,:);
    structPlot(i).time   = time(dataMask,:)-time(dataMask(1),:);

    if i == 5
        FieldBaselineM = mean(Bfield(time >= 6 & time <= 10,:));
        FieldBaselineS = std(Bfield(time >= 6 & time <= 10,:));
        for j = 1:4
            idxS = (j-1)*3+1;
            idxE = (j-1)*3+3;
            FieldBaselineM_norm(j) = norm(FieldBaselineM(idxS:idxE));
            FieldBaselineS_norm(j) = norm(FieldBaselineS(idxS:idxE));
        end
    end

    figure(i)
    subplot(311)
    hold on
    plot(time,EE_pos)
    plot(time(locs),pks,"k*")
    hold off
    subplot(312)
    plot(time,Bfield)
    subplot(313)
    hold on
    % plot(time,magPos)
    plot(time,state)
    hold off

end

%%

% Figure Option 1 - all measurements
figure
subplot(7,1,1)
hold on
for i = 1:size(fileNames,1)
plot(structPlot(i).time, structPlot(i).EE_pos(:,2))
end
xlim([structPlot(i).time(1) structPlot(i).time(end)])

for i = 1:size(fileNames,1)
subplot(7,1,i+1)
hold on
plot(structPlot(i).time, structPlot(i).Bfield)
yline(max(FieldBaselineM+FieldBaselineS),'k--')
yline(min(FieldBaselineM-FieldBaselineS),'k--')
hold off
xlim([structPlot(i).time(1) structPlot(i).time(end)])
end

% Figure Option 2 - only norm measurements
figure
subplot(7,1,1)
hold on
for i = 1:size(fileNames,1)
plot(structPlot(i).time, structPlot(i).EE_pos(:,2))
end
xlim([structPlot(i).time(1) structPlot(i).time(end)])

ylimits = [150 50 150 50 300 200];

for i = 1:size(fileNames,1)
subplot(7,1,i+1)
hold on
clear BfieldNorm
for j = 1:4
    idxS = (j-1)*3+1;
    idxE = (j-1)*3+3;
    BfieldNorm(:,j) = vecnorm(structPlot(i).Bfield(:,idxS:idxE),2,2);
end
plot(structPlot(i).time, BfieldNorm)
yline(max(FieldBaselineM_norm+FieldBaselineS_norm),'k--')
yline(min(FieldBaselineM_norm),'k--')
hold off
xlim([structPlot(i).time(1) structPlot(i).time(end)])
ylim([0 ylimits(i)])
end

%% Quantitative summary (reproduces Table S2)

objectNames = ["Driller"; "Crimper"; "Hammer"; "Caliper"; "Wrenches"; "Phone"];

pickField       = zeros(size(fileNames,1), 1);
pickR2          = zeros(size(fileNames,1), 1);
avgR2           = zeros(size(fileNames,1), 1);
nMisclassified  = zeros(size(fileNames,1), 1);
windowDuration  = zeros(size(fileNames,1), 1);

for i = 1:size(fileNames,1)

    % Peak field: max Euclidean norm across all 4 sensors in the window
    maxNorm = 0;
    for j = 1:4
        idxS = (j-1)*3+1;
        idxE = (j-1)*3+3;
        sensorNorm = vecnorm(structPlot(i).Bfield(:, idxS:idxE), 2, 2);
        maxNorm = max(maxNorm, max(sensorNorm));
    end
    pickField(i) = maxNorm;

    % Peak and average R²
    pickR2(i) = max(structPlot(i).R2);
    avgR2(i)  = mean(structPlot(i).R2);

    % Analysis window duration: spans several end-effector approach/retreat cycles
    % (i.e., repeated presentations of the object near the detector), not a single
    % brief exposure. See FILE SUMMARY note above.
    windowDuration(i) = structPlot(i).time(end);

    % Misclassification events: contiguous segments where state ~= 2 lasting > 0.1 s
    stateData   = structPlot(i).state;
    timeData    = structPlot(i).time;
    isWrong     = stateData ~= 2;
    transitions = diff([0; isWrong; 0]);
    segStarts   = find(transitions ==  1);
    segEnds     = find(transitions == -1) - 1;
    count = 0;
    for k = 1:numel(segStarts)
        if timeData(segEnds(k)) - timeData(segStarts(k)) > 0.1
            count = count + 1;
        end
    end
    nMisclassified(i) = count;

end

summaryTable = table(objectNames, pickField, pickR2, avgR2, windowDuration, nMisclassified, ...
    'VariableNames', {'Object', 'PeakField', 'PeakR2', 'AvgR2', 'WindowDuration_s', 'MisclassifiedEvents'});
disp(summaryTable)

%% Auxiliary Function

function data = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  DEMOEXP0707200608 = IMPORTFILE(FILENAME) reads data from text file
%  FILENAME for the default selection.  Returns the numeric data.
%
%  DEMOEXP0707200608 = IMPORTFILE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 07-Jul-2025 20:13:22

% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 59);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "mag_pos_x", "mag_pos_y", "mag_pos_z", "mag_orien_x", "mag_orien_y", "mag_orien_z", "ee_x", "ee_y", "ee_z", "v_ee_x", "v_ee_y", "v_ee_z", "F_ee_x", "F_ee_y", "F_ee_z", "s1_x", "s1_y", "s1_z", "s2_x", "s2_y", "s2_z", "s3_x", "s3_y", "s3_z", "s4_x", "s4_y", "s4_z", "n_s1_x", "n_s1_y", "n_s1_z", "n_s2_x", "n_s2_y", "n_s2_z", "n_s3_x", "n_s3_y", "n_s3_z", "n_s4_x", "n_s4_y", "n_s4_z", "ls_compute_time", "r2", "O_T_EE_1", "O_T_EE_2", "O_T_EE_3", "O_T_EE_4", "O_T_EE_5", "O_T_EE_6", "O_T_EE_7", "O_T_EE_8", "O_T_EE_9", "O_T_EE_10", "O_T_EE_11", "O_T_EE_12", "O_T_EE_13", "O_T_EE_14", "O_T_EE_15", "O_T_EE_16", "state"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);

% Convert to output type
data = table2array(data);
end
