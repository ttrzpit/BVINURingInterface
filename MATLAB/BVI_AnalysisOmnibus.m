%% === Load trial data ====================================================
close all;
clear all;
clc;

% Load trial data from the specified directory
[trials, summary] = ParseNURingTrials('+TrialData');


%% === ALL PLOTS =========================================================

%% --- Trajectory Plot ---------------------------------------------------
close all;

% Select participant and trial
pID = 123 ;                                                    % Participant ID
tID = 331;                                                    % Target marker ID
allTrials = trials(trials.participant_id == string(pID), :);    % All trials for this participant
oneTrialRow = allTrials(allTrials.target_marker == sprintf('%03d', tID), :);

% Verify
if height(oneTrialRow) == 0
    error('No trial found for participant %d, target marker %d', pID, tID);
elseif height(oneTrialRow) > 1
    warning('Multiple trials found for participant %d, target marker %d; using the first.', pID, tID);
end

pTitle   = oneTrialRow.filename{1};   % grab filename from the row BEFORE overwriting
oneTrial = oneTrialRow.data{1};       % now extract the timetable

% Plot trial
PlotNURingTrial(oneTrial, ...
    'LeftWidth', 0.45, ...
    'TrialTitle', string(pTitle), ...
    'DrawShadowXY', false, ...
    'DrawEstimated', false);