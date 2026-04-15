%% test script

clc
clear
close all
warning off

%%


disp("************ Start test script to check repo ************")

rmpath(genpath("data"))
rmpath(genpath("dependencies"))

addpath("scripts")

currentPath = pwd;
filesMatlab = dir(fullfile(currentPath+"/scripts", '*.m'));

test = true;

numFiles = size(filesMatlab,1);

for indexFile = 1:numFiles
    disp("************ running script "+num2str(indexFile)+"/"+num2str(numFiles)+" ************")
    scriptName = filesMatlab(indexFile).name;
    run(scriptName)
    close all
    rmpath(genpath("data"))
    rmpath(genpath("dependencies"))
    disp("************ complete running script "+num2str(indexFile)+"/"+num2str(numFiles)+" ************")
end


disp("************ Test Passed! ************")