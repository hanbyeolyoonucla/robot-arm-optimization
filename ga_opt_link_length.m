clc; clear; close all;
addpath('functions');
lb = [0.045 0.082 0.100 0.040 0.090 0 0.1 -Inf -Inf];
% ub = [Inf Inf Inf Inf Inf pi Inf Inf Inf];
ub = [0.3 0.3 0.3 0.3 0.3 pi Inf Inf Inf];
tic
[x,fval,exitflag,output,population,scores] = ga(@PerformanceIndexFunction,9,[],[],[],[],lb,ub,@NonLinCon);
toc
%% save result
occusalCutOn = 0;
axialCutOn = 1;
maxillaOn = 0;
mandibleOn = 1;
n_angle = 5;
save('data/GA_220919','occusalCutOn','axialCutOn','maxillaOn','mandibleOn','n_angle',...
    'fval','lb','ub','population','scores','x');