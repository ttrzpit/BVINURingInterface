function results = ParticipantGuidanceAnalysis(participantId, headers, trials, varargin)
%PARTICIPANTGUIDANCEANALYSIS Per-participant Euclidean-distance-vs-time
%   regression analysis and summary figure.
%
%   results = PARTICIPANTGUIDANCEANALYSIS(participantId, headers, trials)
%   selects every trial belonging to PARTICIPANTID out of the TRIALS table
%   produced by ParseNURingTrials, and for each trial:
%       1. Computes the Euclidean distance to target at each timestep
%          (norm of [dx_mm, dy_mm, dz_mm] -- the fingertip-offset vector),
%       2. Fits a simple linear regression of that distance against
%          t_secs (distance ~ time),
%       3. Records the regression slope (mm/s), R^2, and the trial's
%          completion_time_s, 3D endpoint_radial_err_mm (norm of the
%          header's endpoint_error_x/y/z_mm), and 2D
%          endpoint_radial_err_2d_mm (norm of just endpoint_error_x/y_mm).
%
%   It then draws one figure with three regions:
%       Top-left    - bar chart of summary metrics, averaged (or median,
%                      where noted) across this participant's trials:
%                      mean R^2, mean regression slope, mean completion
%                      time, mean/median 3D endpoint error, mean/median 2D
%                      endpoint error. Error bars show 1 SEM.
%       Bottom-left - histogram of the R^2 values across trials, using
%                      bins of width 0.2.
%       Right       - a full-height plot of Euclidean distance (mm) vs.
%                      time (s) for every trial: raw samples as small
%                      hollow circles, the fitted regression line as a
%                      solid line, one consistent color per trial.
%
%   RESULTS is a table with one row per trial:
%       trial_key, target_id, slope_mm_per_s, r_squared,
%       completion_time_s, endpoint_radial_err_mm, endpoint_radial_err_2d_mm
%
%   results = PARTICIPANTGUIDANCEANALYSIS(..., 'MatchFingersightScale', true)
%   forces the right-hand plot's axis limits/ticks (and the top-left
%   y-axis) to match the FingerSight paper's scale for direct visual
%   comparison.
%
%   Example:
%       [headers, trials] = ParseNURingTrials('+TrialData');
%       results = ParticipantGuidanceAnalysis("123", headers, trials);


% =====================================================================
% === PARSE INPUTS ====================================================
% =====================================================================
p = inputParser;
addParameter(p, 'MatchFingersightScale', false, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
matchFingersightScale = p.Results.MatchFingersightScale;


% =========================================================================
% Select this participant's trials
% =========================================================================
participantId = string(participantId);

realHeaders = headers(~ismember(headers.target_id, ["MEAN", "MEDIAN"]), :);
pHeaders = realHeaders(realHeaders.user_id == participantId, :);
pTrials  = trials(trials.participant_id == participantId, :);

joined = innerjoin(pHeaders, pTrials, 'Keys', 'trial_key');

if isempty(joined)
    error('ParticipantGuidanceAnalysis:noTrials', ...
        'No trials found for participant_id "%s".', participantId);
end

joined = sortrows(joined, 'target_id');
n = height(joined);

% =========================================================================
% Per-trial regression: euclidean distance (to target) vs time
% =========================================================================
trial_key          = joined.trial_key;
target_id          = joined.target_id;
slope_mm_per_s      = nan(n,1);
r_squared           = nan(n,1);
completion_time_s   = joined.completion_time_s;

endpoint_radial_err_mm = sqrt( ...
    joined.endpoint_error_x_mm.^2 + ...
    joined.endpoint_error_y_mm.^2 + ...
    joined.endpoint_error_z_mm.^2);

endpoint_radial_err_2d_mm = sqrt( ...
    joined.endpoint_error_x_mm.^2 + ...
    joined.endpoint_error_y_mm.^2);

distFitData = cell(n,1); % each cell: struct with t, dist, tFit, distFit

for i = 1:n
    tt = joined.data{i};

    dist = sqrt(tt.dx_mm.^2 + tt.dy_mm.^2 + tt.dz_mm.^2);
    t    = tt.t_secs;

    valid = ~isnan(dist) & ~isnan(t);
    t = t(valid);
    dist = dist(valid);

    fitInfo = struct('t', t, 'dist', dist, 'tFit', [], 'distFit', []);

    if numel(t) >= 2
        pf = polyfit(t, dist, 1); % pf(1) = slope, pf(2) = intercept
        slope_mm_per_s(i) = pf(1);

        distFit = polyval(pf, t);
        ssRes = sum((dist - distFit).^2);
        ssTot = sum((dist - mean(dist)).^2);
        if ssTot > 0
            r_squared(i) = 1 - ssRes/ssTot;
        else
            r_squared(i) = NaN;
        end

        fitInfo.tFit = t;
        fitInfo.distFit = distFit;
    end

    distFitData{i} = fitInfo;
end

results = table(trial_key, target_id, slope_mm_per_s, r_squared, ...
    completion_time_s, endpoint_radial_err_mm, endpoint_radial_err_2d_mm);

% =========================================================================
% Summary metrics (top-left bar chart) + SEM
% =========================================================================
meanR2            = mean(r_squared, 'omitnan');
meanSlope         = mean(slope_mm_per_s, 'omitnan');
meanCompletion    = mean(completion_time_s, 'omitnan');
meanEndpointErr   = mean(endpoint_radial_err_mm, 'omitnan');
medEndpointErr    = median(endpoint_radial_err_mm, 'omitnan');
meanEndpointErr2D = mean(endpoint_radial_err_2d_mm, 'omitnan');
medEndpointErr2D  = median(endpoint_radial_err_2d_mm, 'omitnan');

% summaryLabels = {'Mean R^2 [%]', 'Mean Slope [mm/s]', 'Mean Time [s]', ...
%     'Mean Err3D [mm]', 'Median Err3D [mm]', ...
%     'Mean Err2D [mm]', 'Median Err2D [mm]'};
summaryLabels = [
    "Mean R^2" + newline + "[%]"
    "Mean Slope" + newline + "[mm/s]"
    "Mean Time" + newline + "[s]"
    "Mean Err3D" + newline + "[mm]"
    "Median Err3D" + newline + "[mm]"
    "Mean Err2D" + newline + "[mm]"
    "Median Err2D" + newline + "[mm]"
];
summaryValues = [meanR2, meanSlope, meanCompletion, meanEndpointErr, medEndpointErr, ...
    meanEndpointErr2D, medEndpointErr2D];

% SEM for each metric (median endpoint error uses the same SEM as the mean,
% i.e. SD/sqrt(n), since there's no closed-form SEM for a median)
nR2     = sum(~isnan(r_squared));
nSlope  = sum(~isnan(slope_mm_per_s));
nTime   = sum(~isnan(completion_time_s));
nErr    = sum(~isnan(endpoint_radial_err_mm));
nErr2D  = sum(~isnan(endpoint_radial_err_2d_mm));

semR2            = std(r_squared, 'omitnan') / sqrt(nR2);
semSlope         = std(slope_mm_per_s, 'omitnan') / sqrt(nSlope);
semCompletion    = std(completion_time_s, 'omitnan') / sqrt(nTime);
semEndpointErr   = std(endpoint_radial_err_mm, 'omitnan') / sqrt(nErr);
semEndpointErr2D = std(endpoint_radial_err_2d_mm, 'omitnan') / sqrt(nErr2D);

summaryErrors = [semR2, semSlope, semCompletion, semEndpointErr, semEndpointErr, ...
    semEndpointErr2D, semEndpointErr2D];

% =========================================================================
% Figure layout
% =========================================================================
fig = figure('Units', 'normalized', 'Position', [0.1 0.1 0.7 0.7]);
fig.Name = sprintf('Participant %s - Guidance Analysis', participantId);

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
title(axTopLeft, sprintf('Participant %s - Summary Metrics', participantId));
grid(axTopLeft, 'on');

hold(axTopLeft, 'on');
errorbar(axTopLeft, 1:numel(summaryValues), summaryValues, summaryErrors, ...
    'k', 'LineStyle', 'none', 'LineWidth', 1.2, 'CapSize', 8);

% Limits to match Fingersight
if ( matchFingersightScale )
    ylim(axTopLeft, [-120, 120]);
    axTopLeft.YTick = -120:40:120;
end

hold(axTopLeft, 'off');

for k = 1:numel(summaryValues)
    text(axTopLeft, k, summaryValues(k) + summaryErrors(k), sprintf('%.3g', summaryValues(k)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

% --- Bottom-left: histogram of per-trial R^2 values ---------------------
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

% --- Right: full-height distance-vs-time plot, all trials --------------
axes(axRight); %#ok<LAXES>
hold(axRight, 'on');
colors = lines(n);
legendHandles = gobjects(n,1);
legendLabels  = strings(n,1);

for i = 1:n
    fi = distFitData{i};
    c = colors(i,:);

    plot(axRight, fi.t, fi.dist, 'o', ...
        'MarkerEdgeColor', c, 'MarkerFaceColor', 'none', 'MarkerSize', 4, ...
        'LineStyle', 'none', 'HandleVisibility', 'off');

    if ~isempty(fi.tFit)
        h = plot(axRight, fi.tFit, fi.distFit, '-', 'Color', c, 'LineWidth', 1.75);
        legendHandles(i) = h;
        legendLabels(i) = sprintf('Target %s (R^2=%.2f)', string(target_id(i)), r_squared(i));
    end
end

% Limits to match Fingersight
if ( matchFingersightScale )
    xlim(axRight, [0, 45]);
    ylim(axRight, [-100, 480]);
    axRight.XTick = 0:5:45;
    axRight.YTick = -100:145:480;
end

hold(axRight, 'off');
xlabel(axRight, 'Time (s)');
ylabel(axRight, 'Euclidean Distance to Target (mm)');
title(axRight, sprintf('Participant %s - Distance to Target vs. Time (all trials)', participantId));
grid(axRight, 'on');

validLegend = isgraphics(legendHandles);
legend(axRight, legendHandles(validLegend), legendLabels(validLegend), ...
    'Location', 'eastoutside', 'FontSize', 10);

end