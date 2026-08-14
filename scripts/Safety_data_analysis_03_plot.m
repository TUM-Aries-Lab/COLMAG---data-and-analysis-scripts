%% FILE SUMMARY
% Purpose: Plot representative end-effector position/velocity profiles for the cleaned 1 m/s
%          stop-distance trials, and run the stop-distance-vs-speed statistical comparison
%          (0.8 / 0.9 / 1.0 m/s).
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Statistics and Machine Learning Toolbox; Curve Fitting Toolbox; local
%               package dependencies/dabarplot.m.
% Data: data/session_03/DataStop/StopProfiles_10ms_clean.mat (produced by
%       Safety_data_analysis_03_detect.m).
% Run Notes: Run Safety_data_analysis_03_detect.m first to (re)generate the cleaned trial data.
%            This script performs no trial selection of its own; every trial it plots has
%            already passed the documented QC criteria in the detect script.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath("data/session_03/DataStop")
        addpath("dependencies")
        dataStopDir = "data/session_03/DataStop";
    end
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % operations over path
    currentPath = pwd;
    cd ..
    addpath("data/session_03/DataStop")
    dataStopDir = fullfile(pwd,"data","session_03","DataStop");
    cd(currentPath)
    addpath("../dependencies")
end

%% Load the cleaned trial data (outlier/defect detection already applied upstream)

cleanDataFile = fullfile(dataStopDir,"StopProfiles_10ms_clean.mat");
if ~isfile(cleanDataFile)
    error("Cleaned trial data not found at %s. Run Safety_data_analysis_03_detect.m first.", cleanDataFile);
end
load(cleanDataFile, "structStopsClean")
structStops = structStopsClean;
nKept = numel(structStops);

%% Representative position/velocity profile overlay (kept trials only)

samplSmooth = 150;
figure
subplot(211)
hold on
tend = zeros(1,nKept);
nSampl = zeros(1,nKept);
for i = 1:nKept
    plot(structStops(i).time,smooth(structStops(i).EE_pos(:,2),samplSmooth))
    tend(i) = structStops(i).time(end);
    nSampl(i) = length(structStops(i).time);
end
xlim([6 max(tend)])
ylim([-0.2 0.42])
tEndMin = min(nSampl);
data4mean = zeros(tEndMin,nKept);
for i = 1:nKept
    data4mean(:,i) = smooth(structStops(i).EE_pos(1:tEndMin,2),samplSmooth);
end
plot(0:0.001:(tEndMin-1)*0.001,mean(data4mean,2),"k","LineWidth",1)

subplot(212)
hold on
for i = 1:nKept
    plot(structStops(i).time,smooth(structStops(i).EE_vel(:,2),samplSmooth))
end
xlim([6 max(tend)])
data4mean = zeros(tEndMin,nKept);
for i = 1:nKept
    data4mean(:,i) = smooth(structStops(i).EE_vel(1:tEndMin,2),samplSmooth);
end
plot(0:0.001:(tEndMin-1)*0.001,mean(data4mean,2),"k","LineWidth",1)

%% Stop-distance-vs-speed comparison (0.8, 0.9, 1.0 m/s)
% NOTE: dist_08/dist_09/dist_10 below are transcribed summary values (10 per speed) and are not
% currently regenerated here from the raw per-trial CSVs in data/session_03/DataStop/08ms,
% 09ms, 10ms. This provenance gap is separate from the trial-selection issue fixed by
% Safety_data_analysis_03_detect.m and should be reconciled against the raw session data
% (i.e., these arrays should be computed from the raw files, not hardcoded) before resubmission.

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
    disp("--- OMNIBUS Friedman test across speeds (0.8 / 0.9 / 1.0 m/s) ---")
    [pOmni,tbl,statsOmni] = friedman(distArray);
    disp("Omnibus chi2 = " + num2str(tbl{2,5}) + ", omnibus p = " + num2str(pOmni))

    disp("--- POST-HOC pairwise comparisons (Bonferroni-corrected) ---")
    [results,means,~,gnames] = multcompare(statsOmni,"CriticalValueType","bonferroni");
    for r = 1:size(results,1)
        disp(gnames(results(r,1)) + " vs " + gnames(results(r,2)) + ...
            ": p_bonferroni = " + num2str(results(r,6)))
    end
end

%%
figure("Units","normalized","Position",[0.3 0.3 0.15 0.3])
hold on
h = dabarplot(distArray,'errorbars','SD',...
    'barspacing',0.8);
scatter(ones(10),dist_08,25,"filled","k")
scatter(2*ones(10),dist_09,25,"filled","k")
scatter(3*ones(10),dist_10,25,"filled","k")
