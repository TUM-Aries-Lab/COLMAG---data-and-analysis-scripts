%% FILE SUMMARY
% Purpose: Batch-process stop experiments to compare minimum distance behavior.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Statistics and Machine Learning Toolbox; Curve Fitting Toolbox; local package dependencies/dabarplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: CSV logs in data/new_data/DataStop/10ms.
% Outputs: Per-trial distance traces, summary distributions, and nonparametric test results.
% Run Notes: Update fileNames to include/exclude recordings in the batch analysis.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath("data/session_03/DataStop/10ms")
    end 
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % operations over path
    currentPath = pwd;
    cd ..
    addpath("data/session_03/DataStop/10ms")
    cd(currentPath)
end

addpath(genpath("dependencies"))

%%

fileNames = ["Demo_exp_07_14_23_54_49.csv"
             "Demo_exp_07_14_23_55_36.csv"
             "Demo_exp_07_14_23_56_19.csv"
             "Demo_exp_07_14_23_57_29.csv"
             "Demo_exp_07_14_23_58_42.csv"
             "Demo_exp_07_14_23_59_51.csv"
             "Demo_exp_07_15_00_02_08.csv"
             "Demo_exp_07_15_00_04_14.csv"
             "Demo_exp_07_15_00_05_04.csv"
             "Demo_exp_07_15_00_05_56.csv"
             "Demo_exp_07_15_00_06_43.csv"
             "Demo_exp_07_15_00_07_30.csv"
             "Demo_exp_07_15_00_10_47.csv"
             "Demo_exp_07_15_00_15_13.csv"
             "Demo_exp_07_15_00_17_09.csv"
             "Demo_exp_07_15_00_19_01.csv"
             "Demo_exp_07_15_00_20_28.csv"
             "Demo_exp_07_15_00_23_52.csv"
             "Demo_exp_07_15_00_25_04.csv"
             "Demo_exp_07_15_00_25_55.csv"
             "Demo_exp_07_15_01_14_12.csv"
             "Demo_exp_07_15_01_15_26.csv"
             "Demo_exp_07_15_01_16_25.csv"];


for i = 1:size(fileNames,1)
    fileName = fileNames(i,:);

    data = importfile(fileName);

    mask = 1:size(data,1);

    time = data(mask,1)-data(mask(1),1);
    magPos = data(mask,2:7);
    EE_pos = data(mask,8:10);
    EE_vel = data(mask,11:13);
    Bfield = data(mask,17:28);
    Bfield_withNord = data(mask,29:40);
    compTime = data(mask,41);
    r_squared = data(mask,42);
    state  = data(mask,end);

    mask2 = ~(EE_vel(:,1) == 0 & EE_vel(:,2) == 0 & EE_vel(:,3) == 0);

    tstart = time(mask2);
    tstart = tstart(1);
    structStops(i).time = time(mask2)-tstart;
    structStops(i).EE_pos = EE_pos(mask2,:);
    structStops(i).EE_vel = EE_vel(mask2,:);
    structStops(i).dist_10 = min(vecnorm(EE_pos(mask2,:)-magPos(mask2,1:3),2,2));

    figure
    plot(structStops(i).time,vecnorm(magPos(mask2,1:3)-structStops(i).EE_pos,2,2))

end

%%

samplSmooth = 150;
figure
subplot(211)
hold on
k = 1;
for i = [1 3:4 7 9:17 20]
    plot(structStops(i).time,smooth(structStops(i).EE_pos(:,2),samplSmooth))
    tend(k) = structStops(i).time(end);
    nSampl(k) = length(structStops(i).time);
    k = k+1;
end
xlim([6 max(tend)])
ylim([-0.2 0.42])
tEndMin = min(nSampl);
data4mean = zeros(tEndMin,length([1 3:4 7 9:17 20]));
k = 1;
for i = [1 3:4 7 9:17 20]
    data4mean(:,k) = smooth(structStops(i).EE_pos(1:tEndMin,2),samplSmooth);
    k = k+1;
end
plot(0:0.001:(tEndMin-1)*0.001,mean(data4mean,2),"k","LineWidth",1)

subplot(212)
hold on
for i = [1 3:4 7 9:17 20]
    plot(structStops(i).time,smooth(structStops(i).EE_vel(:,2),samplSmooth))
end
xlim([6 max(tend)])
data4mean = zeros(tEndMin,length([1 3:4 7 9:17 20]));
k = 1;
for i = [1 3:4 7 9:17 20]
    data4mean(:,k) = smooth(structStops(i).EE_vel(1:tEndMin,2),samplSmooth);
    k = k+1;
end
plot(0:0.001:(tEndMin-1)*0.001,mean(data4mean,2),"k","LineWidth",1)

%%

dist_08 = [0.2259 0.2571 0.2114 0.2225 0.2399 0.2192 0.2473 0.2537 0.2213 0.2272];
dist_09 = [0.2265 0.2316 0.2025 0.2389 0.2139 0.2229 0.2409 0.1974 0.2184 0.2117];
dist_10 = [0.2011 0.1971 0.2070 0.2034 0.2260 0.1969 0.1947 0.2060 0.2238 0.2280];

distArray = [dist_08' dist_09' dist_10'];

try
    if(test)
        [p,tbl,stats] = friedman(distArray);
        [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni");
    end 
catch exception
    [p,tbl,stats] = friedman(distArray)
    [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni")
end

%%
figure("Units","normalized","Position",[0.3 0.3 0.15 0.3])
hold on
h = dabarplot(distArray,'errorbars','SD',...
    'barspacing',0.8);
scatter(ones(10),dist_08,25,"filled","k")
scatter(2*ones(10),dist_09,25,"filled","k")
scatter(3*ones(10),dist_10,25,"filled","k")

%%

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
%  Example:
%  Demoexp0707200608 = importfile("C:\Users\Federico\OneDrive\Lavoro ARIES\03 - Papers\TRO - Magnetic Manipulator\Science Advances\New Data\DataStop\Demo_exp_07_07_20_06_08.csv", [2, Inf]);
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