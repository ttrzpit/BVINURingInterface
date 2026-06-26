%% === Load trial data ====================================================
close all;
clear all;
clc;

% Load trial data from the specified directory
[headers, trials, summary] = ParseNURingTrials('+TrialData');



%% ========================================================================
%  === STATISTICAL ANALYSIS ===============================================
%  ========================================================================




%% ========================================================================
%  === ALL PLOTS ==========================================================
%  ========================================================================


%% --- Full Single Trial Visualization ------------------------------------
close all;

% Select user and trial
key = "u126_t267" ;
hdr = headers(headers.trial_key == key, :);   % adjust to your actual user/target IDs
row = trials(trials.trial_key == key , :);
tt  = row.data{1};

% Plot
VisualizeNURingTrial(tt, ...
    'TargetScreenPosition', [hdr.target_screen_position_x_mm, hdr.target_screen_position_y_mm], ...
    'TrialTitle', row.filename(1), ...
    'DrawShadowXY', false, ...
    'DrawEstimated', false, ...
    'Threshold', 500, ...
    'ShowVelocity', true, ...
    'Animate', true, ...
    'SetZLimit', 0, ...
    'ColumnWidths', [0.4 0.3 0.3]) ;






%% --- Single Trial Plot ---------------------------------------------
close all;

% Select user and trial
key = "u123_t383" ;
hdr = headers(headers.trial_key == key, :);   % adjust to your actual user/target IDs
row = trials(trials.trial_key == key , :);
tt  = row.data{1};

% Plot
PlotNURingTrial(tt, ...
    'TargetScreenPosition', [hdr.target_screen_position_x_mm, hdr.target_screen_position_y_mm], ...
    'TrialTitle', row.filename(1), ...
    'DrawShadowXY', false, ...
    'DrawEstimated', false, ...
    'Threshold', 500, ...
    'ShowVelocity', true, ...
    'Animate', false, ...
    'SetZLimit', 0) ;



%% --- Linear Regression (single participant) -----------------------------
close all; 

% Select user
user = "123" ;

% Plot
results = ParticipantGuidanceAnalysis(user, headers, trials, ...
    'MatchFingersightScale', true ) ; 
