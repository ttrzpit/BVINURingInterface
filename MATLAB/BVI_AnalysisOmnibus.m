%% === Load trial data ====================================================
close all;
clear all;
clc;

% Load trial data from the specified directory
[headers, trials, summary] = ParseNURingTrials('+TrialData');

disp(headers)

% === ALL PLOTS =========================================================

%% --- Trajectory Plot ---------------------------------------------------
close all;

% Select user and trial
key = "u123_t126" ;
hdr = headers(headers.trial_key == key, :);   % adjust to your actual user/target IDs
row = trials(trials.trial_key == key , :);
tt  = row.data{1};

PlotNURingTrial(tt, ...
    'TargetScreenPosition', [hdr.target_screen_position_x_mm, hdr.target_screen_position_y_mm], ...
    'TrialTitle', row.filename(1), ...
    'DrawShadowXY', false, ...
    'DrawEstimated', true, ...
    'PlotR2Comparison', false, ...
    'Threshold', 500) ;

