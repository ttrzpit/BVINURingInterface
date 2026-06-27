%% === Load trial data ====================================================
close all;
clear all;
clc;

% Load trial data from the specified directory
[headers, trials, summary] = ParseNURingTrials('+TrialData', 'Participants', {});

%% ========================================================================
%  === STATISTICAL ANALYSIS ===============================================
%  ========================================================================




%% ========================================================================
%  === ALL PLOTS ==========================================================
%  ========================================================================


%% --- Full Single Trial Visualization ------------------------------------
close all;

% Select user and trial
key = "u801_t230" ;
hdr = headers(headers.trial_key == key, :);   % adjust to your actual user/target IDs
row = trials(trials.trial_key == key , :);
tt  = row.data{1};

% Plot
VisualizeNURingTrial(tt, ...
    'TargetScreenPosition', [hdr.target_screen_position_x_mm, hdr.target_screen_position_y_mm], ...
    'TrialTitle', row.filename(1), ...
    'DrawShadowXY', true, ...
    'DrawEstimated', false, ...
    'Threshold', 500, ...
    'ShowVelocity', true, ...
    'Animate', true, ...
    'SetZLimit', 0, ...
    'ColumnWidths', [0.4 0.3 0.3]) ;



%% --- Linear Regression (single participant) -----------------------------
close all; 

% Select user
user = "124" ;

% Plot
results = ParticipantGuidanceAnalysis(user, headers, trials, ...
    'MatchFingersightScale', true ) ; 



%% --- Linear Regression (whole study) ------------------------------------
close all; 

% Plot
results = StudyGuidanceAnalysis(headers, trials, ...
    'MatchFingersightScale', false ) ; 
