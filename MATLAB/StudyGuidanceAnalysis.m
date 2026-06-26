function results = StudyGuidanceAnalysis(headers, trials, varargin)
%STUDYGUIDANCEANALYSIS Study-wide Euclidean-distance-vs-time regression
%   analysis and summary figure across all participants.
%
%   results = STUDYGUIDANCEANALYSIS(headers, trials)
%   processes every trial in the TRIALS table produced by
%   ParseNURingTrials and for each trial:
%       1. Computes the Euclidean distance to target at each timestep
%          (norm of [dx_mm, dy_mm, dz_mm] -- the fingertip-offset vector),
%       2. Fits a simple linear regression of that distance against
%          t_secs (distance ~ time),
%       3. Records the regression slope (mm/s), R^2, and the trial's
%          completion_time_s, 3D endpoint_radial_err_mm (norm of the
%          header's endpoint_error_x/y/z_mm), and 2D
%          endpoint_radial_err_2d_mm (norm of just endpoint_error_x/y_mm).
%
%   One figure is produced with three regions:
%       Top-left    - bar chart of summary metrics averaged (or median,
%                     where noted) across all trials in the study: mean
%                     R^2, mean regression slope, mean completion time,
%                     mean/median 3D endpoint error, mean/median 2D
%                     endpoint error. Error bars show 1 SEM.
%       Bottom-left - histogram of R^2 values across all trials, using
%                     bins of width 0.2.
%       Right       - a full-height plot of Euclidean distance (mm) vs.
%                     time (s) for every trial in the study: raw samples
%                     as small hollow circles, the fitted regression line
%                     as a solid line. Trials are colored by participant.
%                     No legend is shown.
%
%   RESULTS is a table with one row per trial:
%       participant_id, trial_key, target_id, slope_mm_per_s, r_squared,
%       completion_time_s, endpoint_radial_err_mm, endpoint_radial_err_2d_mm
%
%   results = STUDYGUIDANCEANALYSIS(..., 'MatchFingersightScale', true)
%   forces the right-hand plot's axis limits/ticks (and the top-left
%   y-axis) to match the FingerSight paper's scale for direct visual
%   comparison.
%
%   Example:
%       [headers, trials] = ParseNURingTrials('+TrialData');
%       results = StudyGuidanceAnalysis(headers, trials);


% =====================================================================
% === PARSE INPUTS ====================================================
% =====================================================================
p = inputParser;
addParameter(p, 'MatchFingersightScale', false, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
matchFingersightScale = p.Results.MatchFingersightScale;


% =========================================================================
% Select all real (non-summary) trials
% =========================================================================
realHeaders = headers(~ismember(headers.target_id, ["MEAN", "MEDIAN"]), :);
joined = innerjoin(realHeaders, trials, 'Keys', 'trial_key');

if isempty(joined)
    error('StudyGuidanceAnalysis:noTrials', 'No trials found in the provided data.');
end

joined = sortrows(joined, {'user_id', 'target_id'});
n = height(joined);

% Unique participants for color assignment
allParticipants   = unique(joined.user_id);
nParticipants     = numel(allParticipants);
participantColors = lines(nParticipants);


% =========================================================================
% Per-trial regression: euclidean distance (to target) vs time
% =========================================================================
participant_id    = joined.user_id;
trial_key         = joined.trial_key;
target_id         = joined.target_id;
slope_mm_per_s    = nan(n,1);
r_squared         = nan(n,1);
completion_time_s = joined.completion_time_s;

endpoint_radial_err_mm = sqrt( ...
    joined.endpoint_error_x_mm.^2 + ...
    joined.endpoint_error_y_mm.^2 + ...
    joined.endpoint_error_z_mm.^2);

endpoint_radial_err_2d_mm = sqrt( ...
    joined.endpoint_error_x_mm.^2 + ...
    joined.endpoint_error_y_mm.^2);

distFitData   = cell(n,1);  % struct per trial: t, dist, tFit, distFit
trialColorIdx = nan(n,1);   % index into participantColors

for i = 1:n
    tt = joined.data{i};

    dist = sqrt(tt.dx_mm.^2 + tt.dy_mm.^2 + tt.dz_mm.^2);
    t    = tt.t_secs;

    valid = ~isnan(dist) & ~isnan(t);
    t    = t(valid);
    dist = dist(valid);

    fitInfo = struct('t', t, 'dist', dist, 'tFit', [], 'distFit', []);

    if numel(t) >= 2
        pf = polyfit(t, dist, 1);
        slope_mm_per_s(i) = pf(1);

        distFit = polyval(pf, t);
        ssRes = sum((dist - distFit).^2);
        ssTot = sum((dist - mean(dist)).^2);
        if ssTot > 0
            r_squared(i) = 1 - ssRes / ssTot;
        else
            r_squared(i) = NaN;
        end

        fitInfo.tFit    = t;
        fitInfo.distFit = distFit;
    end

    distFitData{i} = fitInfo;

    % Color index: which participant is this trial?
    trialColorIdx(i) = find(allParticipants == participant_id(i), 1);
end

results = table(participant_id, trial_key, target_id, slope_mm_per_s, r_squared, ...
    completion_time_s, endpoint_radial_err_mm, endpoint_radial_err_2d_mm);


% =========================================================================
% Study-wide summary metrics + SEM
% =========================================================================
meanR2            = mean(r_squared, 'omitnan');
meanSlope         = mean(slope_mm_per_s, 'omitnan');
meanCompletion    = mean(completion_time_s, 'omitnan');
meanEndpointErr   = mean(endpoint_radial_err_mm, 'omitnan');
medEndpointErr    = median(endpoint_radial_err_mm, 'omitnan');
meanEndpointErr2D = mean(endpoint_radial_err_2d_mm, 'omitnan');
medEndpointErr2D  = median(endpoint_radial_err_2d_mm, 'omitnan');

summaryLabels = {'Mean R^2 [%]', 'Mean Slope [mm/s]', 'Mean Time [s]', ...
    'Err3DAvg [mm]', 'Err3DMed [mm]', ...
    'Err2DAvg [mm]', 'Err2DMed [mm]'};
summaryValues = [meanR2, meanSlope, meanCompletion, meanEndpointErr, medEndpointErr, ...
    meanEndpointErr2D, medEndpointErr2D];

nR2    = sum(~isnan(r_squared));
nSlope = sum(~isnan(slope_mm_per_s));
nTime  = sum(~isnan(completion_time_s));
nErr   = sum(~isnan(endpoint_radial_err_mm));
nErr2D = sum(~isnan(endpoint_radial_err_2d_mm));

semR2            = std(r_squared, 'omitnan')                 / sqrt(nR2);
semSlope         = std(slope_mm_per_s, 'omitnan')            / sqrt(nSlope);
semCompletion    = std(completion_time_s, 'omitnan')         / sqrt(nTime);
semEndpointErr   = std(endpoint_radial_err_mm, 'omitnan')    / sqrt(nErr);
semEndpointErr2D = std(endpoint_radial_err_2d_mm, 'omitnan') / sqrt(nErr2D);

summaryErrors = [semR2, semSlope, semCompletion, semEndpointErr, semEndpointErr, ...
    semEndpointErr2D, semEndpointErr2D];


% =========================================================================
% Figure layout
% =========================================================================
fig = figure('Units', 'normalized', 'Position', [0.1 0.1 0.7 0.7]);
fig.Name = sprintf('Study-Wide Guidance Analysis  (%d trials, %d participants)', ...
    n, nParticipants);

axTopLeft    = axes(fig, 'Position', [0.07 0.57 0.40 0.35]);
axBottomLeft = axes(fig, 'Position', [0.07 0.09 0.40 0.35]);
axRight      = axes(fig, 'Position', [0.55 0.09 0.40 0.83]);


% --- Top-left: summary metrics bar chart with SEM error bars -----------
axes(axTopLeft); %#ok<LAXES>
b1 = bar(axTopLeft, summaryValues);
b1.FaceColor = 'flat';
b1.CData = lines(numel(summaryValues));
set(axTopLeft, 'XTickLabel', summaryLabels);
ylabel(axTopLeft, 'Value');
title(axTopLeft, sprintf('Study-Wide Summary Metrics  (N = %d trials, %d participants)', ...
    n, nParticipants));
grid(axTopLeft, 'on');

hold(axTopLeft, 'on');
errorbar(axTopLeft, 1:numel(summaryValues), summaryValues, summaryErrors, ...
    'k', 'LineStyle', 'none', 'LineWidth', 1.2, 'CapSize', 8);

if matchFingersightScale
    ylim(axTopLeft, [-120, 120]);
    axTopLeft.YTick = -120:40:120;
end

for k = 1:numel(summaryValues)
    text(axTopLeft, k, summaryValues(k) + summaryErrors(k), ...
        sprintf('%.3g', summaryValues(k)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
hold(axTopLeft, 'off');


% --- Bottom-left: histogram of per-trial R^2 values -------------------
axes(axBottomLeft); %#ok<LAXES>
edges = 0:0.2:1;
histogram(axBottomLeft, r_squared, 'BinEdges', edges, ...
    'FaceColor', [0.3 0.5 0.7], 'EdgeColor', 'k');
xlabel(axBottomLeft, 'R^2');
ylabel(axBottomLeft, 'Number of Trials');
title(axBottomLeft, 'Distribution of Per-Trial R^2');
xlim(axBottomLeft, [0 1]);
axBottomLeft.XTick = edges;
grid(axBottomLeft, 'on');


% --- Right: full-height distance-vs-time plot, all trials -------------
axes(axRight); %#ok<LAXES>
hold(axRight, 'on');

for i = 1:n
    fi = distFitData{i};
    c  = participantColors(trialColorIdx(i), :);

    plot(axRight, fi.t, fi.dist, 'o', ...
        'MarkerEdgeColor', c, 'MarkerFaceColor', 'none', 'MarkerSize', 3, ...
        'LineStyle', 'none', 'HandleVisibility', 'off');

    if ~isempty(fi.tFit)
        plot(axRight, fi.tFit, fi.distFit, '-', 'Color', c, 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end
end

if matchFingersightScale
    xlim(axRight, [0, 45]);
    ylim(axRight, [-100, 480]);
    axRight.XTick = 0:5:45;
    axRight.YTick = -100:145:480;
end

hold(axRight, 'off');
xlabel(axRight, 'Time (s)');
ylabel(axRight, 'Euclidean Distance to Target (mm)');
title(axRight, sprintf('Distance to Target vs. Time  (all trials, colored by participant)'));
grid(axRight, 'on');

end