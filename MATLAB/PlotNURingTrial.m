function fig = PlotNURingTrial(tt, varargin)
%PLOTNURINGTRIAL Plot a single NURing trial: 3D trajectory + xyz vs time.
%
%   fig = PLOTNURINGTRIAL(tt) creates a figure with:
%     - Left: a large 3D plot of the finger trajectory (tx_mm, ty_mm,
%       tz_mm) approaching the target.
%     - Right: three stacked subplots of tx_mm, ty_mm, and tz_mm vs time
%       (t_secs), one component per plot.
%
%   tt is the per-trial timetable produced by PARSENURINGTRIALS (i.e.
%   p123.data{1}), and must contain variables t_secs, tx_mm, ty_mm, tz_mm.
%
%   fig = PLOTNURINGTRIAL(tt, 'Name', Value, ...) supports:
%
%       'Azimuth'        (default -40)   - 3D view azimuth angle (deg)
%       'Elevation'      (default 20)    - 3D view elevation angle (deg)
%       'TargetPosition'  (default [0 0 0])
%                         - [x y z] mm position of the target marker in
%                           the same frame as tx_mm/ty_mm/tz_mm. Default
%                           assumes the transform is already
%                           target-relative (target at origin).
%       'TrialTitle'      (default '')   - optional title/subtitle string
%                           shown above the figure (e.g. trial filename)
%       'LeftWidth'       (default 0.45) - fraction (0-1) of the figure's
%                           usable width given to the left 3D plot; the
%                           remainder goes to the right-hand stacked
%                           time-series plots. Lower this if the 3D plot
%                           is getting clipped/cut off.
%
%   The default Azimuth/Elevation are chosen so the target (placed near
%   the trajectory's starting end, see note below) appears toward the
%   back-left of the 3D plot, and the trajectory begins near the
%   bottom-right -- tune 'Azimuth'/'Elevation' to taste for your data's
%   actual coordinate convention.
%
%   Example:
%       trials = ParseNURingTrials('+TrialData');
%       p123 = trials(trials.participant_id == "123", :);
%       tt = p123.data{1};
%       plotNURingTrial(tt, 'TrialTitle', p123.filename(1));
%
%   See also PARSENURINGTRIALS.






% =====================================================================
% === PARSE INPUTS ====================================================
% =====================================================================
p = inputParser;
addRequired(p, 'tt');
addParameter(p, 'Azimuth', -60, @isnumeric);
addParameter(p, 'Elevation', 15, @isnumeric);
addParameter(p, 'TargetPosition', [0 0 0], @(x) isnumeric(x) && numel(x)==3);
addParameter(p, 'TrialTitle', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'LeftWidth', 0.45, @(x) isnumeric(x) && x > 0 && x < 1);
addParameter(p, 'DrawShadowXY', false, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'DrawEstimated', false, @(x) islogical(x) || isnumeric(x));
parse(p, tt, varargin{:});

az            = p.Results.Azimuth;
el            = p.Results.Elevation;
targetPos     = p.Results.TargetPosition;
trialTitle    = string(p.Results.TrialTitle);
leftWidthFrac = p.Results.LeftWidth;
drawShadowXY  = p.Results.DrawShadowXY;
drawEstimated = p.Results.DrawEstimated;

requiredVars = {'t_secs','tx_mm','ty_mm','tz_mm'};
for i = 1:numel(requiredVars)
    if ~ismember(requiredVars{i}, tt.Properties.VariableNames)
        error('plotNURingTrial:missingVar', ...
            'Input timetable is missing required variable "%s".', requiredVars{i});
    end
end



% =====================================================================
% === EXTRACT DATA ====================================================
% =====================================================================
% Per-axis data
t  = tt.t_secs;
tx = tt.tx_mm;
ty = tt.ty_mm;
tz = tt.tz_mm;
vis = tt.detected;

% Axis limits
xMax = max(abs(tx));
yMax = max(abs(ty));
zMax = max(abs(tz));
xLim = ceil(xMax/50) * 50 ;
yLim = ceil(yMax/50) * 50 ;
zLim = ceil(zMax/50) * 50 ;



% =====================================================================
% === NEW FIGURE ======================================================
% =====================================================================

% New figure
fig = figure('Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.7 0.7]);

% Add trial title
if trialTitle ~= ""
    annotation(fig, 'textbox', [0 0.95 1 0.05], 'String', trialTitle, ...
        'Interpreter', 'none', 'EdgeColor', 'none', ...
        'HorizontalAlignment', 'center', 'FontSize', Globals.fontSizeTitle, 'FontWeight', 'bold');
end

% Display endpoint
endX = tt.tx_mm(end) ;
endY = tt.ty_mm(end) ; 
endZ = tt.tz_mm(end) ;
rXY  = sqrt(power(endX, 2) + power(endY, 2) ) ;
endpointStr = sprintf('Endpoint error: %.1f mm, %.1f mm, R = %.1f mm, Z = %.1f mm', endX, endY, rXY, endZ);
annotation(fig, 'textbox', [0 0.90 1 0.05], 'String', endpointStr, ...
        'Interpreter', 'none', 'EdgeColor', 'none', ...
        'HorizontalAlignment', 'center', 'FontSize', Globals.fontSizeTitle, 'FontWeight', 'bold');

% Layout geometry (all normalized figure units, [0,1]) ---
margin       = 0.06;   % outer margin
gap          = 0.04;   % gap between left/right halves and between stacked plots
topMargin    = 0.06;   % extra space reserved at top for the title
leftWidth    = leftWidthFrac;      % fraction of usable width given to the 3D plot
rightWidth   = 1 - leftWidth;

usableW = 1 - 2*margin;
usableH = 1 - 2*margin - topMargin;

leftAxW  = usableW * leftWidth - gap/2;
rightAxW = usableW * rightWidth - gap/2;
leftAxX  = margin;
rightAxX = margin + leftAxW + gap;



% =====================================================================
% === LEFT: 3D TRAJECTORY PLOT ========================================
% =====================================================================

% Axis
ax3d = axes(fig, 'Position', [leftAxX, margin, leftAxW, usableH]);
hold(ax3d, 'on');

% Visual-axis remap: plot3/scatter3 args are (visualX, visualY, visualZ).
% We want visual X = tx (horizontal), visual Y = tz (depth, into screen),
% visual Z = ty (vertical) -- so pass (tx, tz, ty), not (tx, ty, tz).
% Plot trajectory data & shadow
plot3(ax3d, tx, tz, ty, '-', 'Color', [Colors.graMd 0.5], 'LineWidth', Globals.lineWidthTraj);
plot3(ax3d, tx, tz, ty, '.', 'Color', [Colors.graDk], 'LineWidth', Globals.lineWidthTraj,  'HandleVisibility','off');

% Plot important points
scatter3(ax3d, tx(1), tz(1), ty(1), 70, 'g', 'filled', 'MarkerEdgeColor', 'k'); % start
scatter3(ax3d, tx(end), tz(end), ty(end), 70, 'r', 'filled', 'MarkerEdgeColor', 'k'); % end of trajectory
% scatter3(ax3d, targetPos(1), targetPos(3), targetPos(2), 120, 'k', 's', 'filled'); % target marker

legend(ax3d, {'Trajectory','Start','End','Target'}, 'Location', 'best');
xlabel(ax3d, 'X (mm)');
ylabel(ax3d, 'Z (mm)'); % visual Y axis carries tz data
zlabel(ax3d, 'Y (mm)'); % visual Z axis carries ty data
title(ax3d, 'Finger Trajectory Toward Target', 'FontSize',Globals.fontSizeTitle);
grid(ax3d, 'on');
set(ax3d, 'DataAspectRatio', [1 1 1]); % equal scaling without vis3d's overflow behavior
set(ax3d, 'YDir', 'reverse'); % reverses Z-data axis (visual Y/depth)
set(ax3d, 'XDir', 'reverse'); % flips X-data axis (visual X, horizontal)
view(ax3d, az, el);

% Explicitly set the 3D axes limits to match the same limMax/zMax used by
% the 2D subplots below, so spacing/scale is consistent everywhere.
% Visual X = tx -> +/-limMax, Visual Y (depth) = tz -> [0 zMax],
% Visual Z (vertical) = ty -> +/-limMax.
xlim(ax3d, [-xLim xLim]);
ylim(ax3d, [0 zMax]);
zlim(ax3d, [-yLim yLim]);

% Tick mark formatting
ax3d.XTick = -xLim:50:xLim;
ax3d.YTick = 0:50:zMax;
ax3d.ZTick = -yLim:50:yLim ;

% Re-assert equal data scaling so 50 units = 50 units = 50 units visually
set(ax3d, 'DataAspectRatio', [1 1 1]);

% Draw reference lines through the origin along each data axis
% Remember the visual-axis remap: visual X = tx, visual Y = tz, visual Z = ty.
xl = xlim(ax3d); % tx range
yl = ylim(ax3d); % tz range (visual Y / depth)
zl = zlim(ax3d); % ty range (visual Z / vertical)

% X-axis (y = 0)
plot3(ax3d, xl, [0 0], [0 0], '-', 'Color', Colors.greLt, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line along tx-axis (ty=0, tz=0)
plot3(ax3d, [xl(1) xl(1)], [0 max(yl)], [0 0], '-', 'Color', Colors.greLt, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line at x=xl(1), z from 0 to max, y=0

% Y-axis (x = 0)
plot3(ax3d, [0 0], [0 0], zl, '-', 'Color', Colors.redLt, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line along ty-axis (tx=0, tz=0)
plot3(ax3d, [yl(1) yl(1)], [0 max(yl)] , [zl(1) zl(1)], '-', 'Color', Colors.redLt, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line at x=xl(1), z from 0 to max, y=0

% Projection of X data (tx) onto the bottom wall (tz = min(tz))
plot3(ax3d, tx, tz, repmat(zl(1), size(tx)), '-', 'Color', [Colors.redMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');

% Projection of Y data (ty) onto the back wall (tx = min(tx))
plot3(ax3d, repmat(xl(1), size(tz)), tz, ty, '-', 'Color', [Colors.greMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');

% Projection of Z data (tz) onto the back plane (tz = min(tz), spanning tx and ty)
if (drawShadowXY)
    plot3(ax3d, tx, repmat(yl(1), size(tx)), ty, '-', 'Color', [Colors.bluMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');
end

% Target marker: drawn as an actual flat 8mm x 8mm square, lying in the
% same plane used by the drawShadowXY projection (visual Y / tz held
% constant at yl(1)), centered at targetPos and spanning tx (visual X)
% and ty (visual Z).
targetSide = 8; % mm, full side length of the target square
half = targetSide / 2;
targetCornersX = targetPos(1) + [-half  half  half -half];
targetCornersY = repmat(yl(1), 1, 4); % flat in the shadow plane (tz = yl(1))
targetCornersZ = targetPos(2) + [-half -half  half  half];
patch(ax3d, targetCornersX, targetCornersY, targetCornersZ, Colors.magMd, ...
    'FaceAlpha', 1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
 


hold(ax3d, 'off');



% =====================================================================
% === RIGHT: THREE STACKED SUBPLOTS ===================================
% =====================================================================

% Subplot width
rightAxH = (usableH - 2*gap) / 3;
hold on; 

% Define each axis
axX = axes(fig, 'Position', [rightAxX, margin + 2*(rightAxH+gap), rightAxW, rightAxH]);
axY = axes(fig, 'Position', [rightAxX, margin + 1*(rightAxH+gap), rightAxW, rightAxH]);
axZ = axes(fig, 'Position', [rightAxX, margin, rightAxW, rightAxH]);

% Hold plots
hold ( axX , 'on' ) ; 

% Plot data
plot(axX, t, tx, 'Color', Colors.redMd, 'LineWidth', Globals.lineWidthData);
plot(axY, t, ty, 'Color', Colors.greMd, 'LineWidth', Globals.lineWidthData);
plot(axZ, t, tz, 'Color', Colors.bluMd, 'LineWidth', Globals.lineWidthData);

if (drawEstimated)
    % Shade areas where pose based on real data
    yl_axX = [-xLim xLim];   % matches the ylim you set further down for axX
    yl_axY = [-yLim yLim];   % matches the ylim you set further down for axX
    yl_axZ = [-zLim zLim];   % matches the ylim you set further down for axX
    runs = findContiguousRuns(vis == 1);  % helper below, or inline it
    for k = 1:size(runs, 1)
        iStart = runs(k,1);
        iEnd   = runs(k,2);
        patch(axX, ...
            [t(iStart) t(iEnd) t(iEnd) t(iStart)], ...
            [yl_axX(1) yl_axX(1) yl_axX(2) yl_axX(2)], ...
            Colors.redLt, 'EdgeColor', 'none', 'FaceAlpha', 0.3, ...
            'HandleVisibility', 'off');
        patch(axY, ...
            [t(iStart) t(iEnd) t(iEnd) t(iStart)], ...
            [yl_axY(1) yl_axY(1) yl_axY(2) yl_axY(2)], ...
            Colors.greLt, 'EdgeColor', 'none', 'FaceAlpha', 0.3, ...
            'HandleVisibility', 'off');
        patch(axZ, ...
            [t(iStart) t(iEnd) t(iEnd) t(iStart)], ...
            [yl_axZ(1) yl_axZ(1) yl_axZ(2) yl_axZ(2)], ...
            Colors.bluLt, 'EdgeColor', 'none', 'FaceAlpha', 0.3, ...
            'HandleVisibility', 'off');
    end
end




% Plot axis zero lines
yline(axX, 0, 'Color', Colors.redLt, 'LineWidth', Globals.lineWidthAxis);
yline(axY, 0, 'Color', Colors.greLt, 'LineWidth', Globals.lineWidthAxis);
yline(axZ, 0, 'Color', Colors.bluLt, 'LineWidth', Globals.lineWidthAxis);

% Axis Labels
ylabel(axX, 'X (mm)', 'FontWeight','bold');
ylabel(axY, 'Y (mm)', 'FontWeight','bold');
ylabel(axZ, 'Z (mm)', 'FontWeight','bold');

% Plot labels
title(axX, 'Position vs Time', 'FontSize', 16);
xlabel(axZ, 'Time (s)', 'FontWeight','bold');

% Grids
grid(axX, 'on');
grid(axY, 'on');
grid(axZ, 'on');

% Limits
ylim(axX, [-xLim xLim])
ylim(axY, [-yLim yLim])
ylim(axZ, [0 zMax]);

% Release hold
hold ( axX , 'off' ) ; 

% Link axis based on 'x'
linkaxes([axX, axY, axZ], 'x');

end



function runs = findContiguousRuns(mask)
%FINDCONTIGUOUSRUNS Return [startIdx endIdx] for each run of consecutive true values.
mask = mask(:);
d = diff([0; mask; 0]);
starts = find(d == 1);
ends   = find(d == -1) - 1;
runs = [starts, ends];
end