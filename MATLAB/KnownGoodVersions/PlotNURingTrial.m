function fig = PlotNURingTrial(tt, varargin)
    %PLOTNURINGTRIALONSCREEN Plot a single NURing trial against the physical
    %   target screen: 3D trajectory + xyz vs time.
    %
    %   fig = PLOTNURINGTRIALONSCREEN(tt) creates a figure with:
    %     - Left: a large 3D plot of the finger trajectory (dx_mm, dy_mm,
    %       dz_mm, offset to be relative to the target's on-screen position)
    %       approaching the target, drawn against a to-scale 527mm x 296mm
    %       screen plane.
    %     - Right: three stacked subplots of x, y, and z position vs time
    %       (t_secs), one component per plot, each with an optional second
    %       y-axis showing that component's velocity.
    %
    %   tt is the per-trial timetable produced by PARSENURINGTRIALS (i.e.
    %   trials.data{i}), and must contain variables t_secs, dx_mm, dy_mm,
    %   dz_mm.
    %
    %   IMPORTANT - target/screen relationship:
    %   dx_mm/dy_mm/dz_mm in tt are fingertip position relative to the
    %   TARGET (i.e. the target sits at (0,0,0) in tt's own frame). This
    %   function re-expresses that trajectory in SCREEN coordinates by
    %   offsetting dx_mm/dy_mm by the target's on-screen position
    %   (TargetScreenPosition, normally taken from headers.target_screen_
    %   position_x_mm / _y_mm for this trial), so the plotted trajectory
    %   approaches the target at its true location on the 527x296mm screen
    %   plane rather than at the origin.
    %
    %   fig = PLOTNURINGTRIALONSCREEN(tt, 'Name', Value, ...) supports:
    %
    %       'TargetScreenPosition' (default [0 0])
    %                         - [x y] mm position of the target on the
    %                           physical screen, in the same screen
    %                           coordinate frame as PlaneWidth/PlaneHeight
    %                           (origin at screen center). Typically:
    %                               headers.target_screen_position_x_mm(i)
    %                               headers.target_screen_position_y_mm(i)
    %                           for the trial matching tt via trial_key.
    %       'PlaneWidth'      (default 527)   - physical screen width (mm)
    %       'PlaneHeight'     (default 296)   - physical screen height (mm)
    %       'Azimuth'        (default -30)   - 3D view azimuth angle (deg)
    %       'Elevation'      (default 30)    - 3D view elevation angle (deg)
    %       'TrialTitle'      (default '')   - optional title/subtitle string
    %                           shown above the figure (e.g. trial filename)
    %       'LeftWidth'       (default 0.45) - fraction (0-1) of the figure's
    %                           usable width given to the left 3D plot; the
    %                           remainder goes to the right-hand stacked
    %                           time-series plots. Lower this if the 3D plot
    %                           is getting clipped/cut off.
    %       'DrawShadowXY'    (default false) - draw the XY-plane shadow
    %                           projection of the trajectory
    %       'DrawEstimated'   (default false) - shade time regions where the
    %                           marker was detected
    %       'PlotR2Comparison'(default true)  - also produce the separate
    %                           actual-vs-ideal R^2 comparison figure
    %       'Threshold'       (default 15)    - mm threshold used for the
    %                           "Nmm completion time"/R^2 metrics
    %       'ShowVelocity'    (default true)  - overlay each component's
    %                           velocity (mm/s) on a second y-axis in the
    %                           three stacked subplots on the right. Set to
    %                           false to show position only.
    %
    %   Example:
    %       [headers, trials] = ParseNURingTrials('+TrialData');
    %       row = trials(trials.trial_key == "u123_t284", :);
    %       hdr = headers(headers.trial_key == "u123_t284", :);
    %       tt  = row.data{1};
    %       PlotNURingTrialOnScreen(tt, ...
    %           'TargetScreenPosition', [hdr.target_screen_position_x_mm, ...
    %                                    hdr.target_screen_position_y_mm], ...
    %           'TrialTitle', row.filename(1));
    %
    %   See also PARSENURINGTRIALS.

    % =====================================================================
    % === PARSE INPUTS ====================================================
    % =====================================================================
    p = inputParser;
    addRequired(p, 'tt');
    addParameter(p, 'TargetScreenPosition', [0 0], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'PlaneWidth', 527, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'PlaneHeight', 296, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'Azimuth', -30, @isnumeric);
    addParameter(p, 'Elevation', 30, @isnumeric);
    addParameter(p, 'TrialTitle', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'LeftWidth', 0.45, @(x) isnumeric(x) && x > 0 && x < 1);
    addParameter(p, 'DrawShadowXY', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'DrawEstimated', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'PlotR2Comparison', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Threshold', 15, @isnumeric);
    addParameter(p, 'ShowVelocity', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Animate', false, @(x) islogical(x) || isnumeric(x));
    parse(p, tt, varargin{:});

    az                  = p.Results.Azimuth;
    el                  = p.Results.Elevation;
    targetScreenPos     = p.Results.TargetScreenPosition; % [x y] mm on screen
    planeWidth          = p.Results.PlaneWidth;
    planeHeight         = p.Results.PlaneHeight;
    trialTitle          = string(p.Results.TrialTitle);
    leftWidthFrac       = p.Results.LeftWidth;
    drawShadowXY        = p.Results.DrawShadowXY;
    drawEstimated       = p.Results.DrawEstimated;
    plotR2Comparison    = p.Results.PlotR2Comparison;
    crossThresh         = p.Results.Threshold;
    showVelocity        = logical(p.Results.ShowVelocity);
    animatePlots        = logical(p.Results.Animate);

    % targetPos: target's 3D position in the SAME (now screen-relative)
    % frame as the offset trajectory below -- on the screen plane (z=0),
    % at its on-screen (x,y) location.
    targetPos = [targetScreenPos(1), targetScreenPos(2), 0];

    requiredVars = {'t_secs','dx_mm','dy_mm','dz_mm'}; % Pull position based on compensated fingertip
    for i = 1:numel(requiredVars)
        if ~ismember(requiredVars{i}, tt.Properties.VariableNames)
            error('plotNURingTrialOnScreen:missingVar', ...
                'Input timetable is missing required variable "%s".', requiredVars{i});
        end
    end



    % =====================================================================
    % === EXTRACT DATA =====================================================
    % =====================================================================
    % Per-axis data, re-expressed in SCREEN coordinates: dx_mm/dy_mm are
    % target-relative in tt, so offsetting by the target's on-screen
    % position re-centers the trajectory on the target's true screen
    % location instead of the origin. Depth (dz_mm) is unaffected, since
    % TargetScreenPosition only describes the target's 2D position on the
    % flat screen plane.
    t  = tt.t_secs;
    tx = tt.dx_mm + targetScreenPos(1);
    ty = tt.dy_mm + targetScreenPos(2);
    tz = tt.dz_mm;

    % Marker-frame (target-relative, centered at 0) versions of the same
    % data, used ONLY for the three stacked time-series subplots on the
    % right -- the 3D plot above stays in world/screen frame.
    txm = tt.dx_mm;
    tym = tt.dy_mm;
    tzm = tt.dz_mm;

    % Velocity of each marker-frame component (mm/s), via numerical
    % gradient against the (possibly non-uniform) sample times -- used for
    % the second y-axis on each of the three stacked subplots, when
    % ShowVelocity is enabled.
    vxm = gradient(txm, t);
    vym = gradient(tym, t);
    vzm = gradient(tzm, t);

    % Endpoint data
    endX = tx(end);
    endY = ty(end);
    endZ = tz(end);

    % Is marker visible
    vis = tt.detected;

    % Half-extents of the physical screen plane
    planeHalfW = planeWidth  / 2;
    planeHalfH = planeHeight / 2;

    % Axis limits -- expanded to cover both the trajectory data AND the
    % physical screen plane, so the plane is never clipped. (World/screen
    % frame -- used for the 3D plot.)
    xMax = max([abs(tx); planeHalfW]);
    yMax = max([abs(ty); planeHalfH]);
    zMax = max(abs(tz));

    xLim = ceil( (( Globals.screenWidth/2)+100)/50)*50;
    yLim = ceil( ((Globals.screenHeight/2)+100)/50)*50;
    zLim = ceil(zMax/50) * 50 ;
    % xLim = ceil(xMax/50) * 50 ;
    % yLim = ceil(yMax/50) * 50 ;
    % zLim = ceil(zMax/50) * 50 ;

    % Marker-frame axis limits (target-relative, centered at 0) -- used for
    % the three stacked time-series subplots on the right. No plane
    % half-extent term here since those subplots aren't showing the screen.
    xMaxM = max(abs(txm));
    yMaxM = max(abs(tym));
    zMaxM = max(abs(tzm));
    xLimM = xLim ;
    yLimM = yLim;
    zLimM = zLim;
    % xLimM = ceil(xMaxM/50) * 50 ; Original
    % yLimM = ceil(yMaxM/50) * 50 ;
    % zLimM = ceil(zMaxM/50) * 50 ;

    % =====================================================================
    % === COMPLETION TIME / R^2 METRICS ==================================
    % =====================================================================

    % Trial completion time: total duration of the trial
    trialCompletionTime = t(end) - t(1);

    % Nmm completion time: time from FIRST crossing of depth=Threshold to
    % the end. find(...,1,'first') guarantees we only ever count the first
    % crossing, so backing out past threshold again later has no effect.
    crossIdx = find(tz <= crossThresh, 1, 'first');
    if isempty(crossIdx)
        timeAtThreshold = NaN;
    else
        timeAtThreshold = t(end) - t(crossIdx);
    end

    % Endpoint accuracy relative to the target's true screen position
    errX = endX - targetPos(1);
    errY = endY - targetPos(2);
    errZ = endZ - targetPos(3);
    rXY  = sqrt(power(errX, 2) + power(errY, 2));
    rXYZ = sqrt(power(errX, 2) + power(errY, 2) + power(errZ, 2));

    % --- R^2: full trajectory vs ideal straight line (start -> target) ---
    actualPts = [tx, ty, tz];
    startPt   = actualPts(1, :);
    targetPt  = targetPos(:)';
    N         = size(actualPts, 1);
    fracs     = ((1:N)' - 1) / max(N - 1, 1);
    idealPts  = startPt + fracs .* (targetPt - startPt);

    SSres     = sum(sum((actualPts - idealPts).^2));
    SStot     = sum(sum((actualPts - mean(actualPts, 1)).^2));
    R2_total  = 1 - SSres / SStot;

    % --- R^2: threshold segment vs ideal straight line (crossing -> target) ---
    if ~isempty(crossIdx) && crossIdx < N
        subPts      = actualPts(crossIdx:end, :);
        Nsub        = size(subPts, 1);
        startPtThreshold  = subPts(1, :);
        fracsThreshold    = ((1:Nsub)' - 1) / max(Nsub - 1, 1);
        idealPtsThreshold = startPtThreshold + fracsThreshold .* (targetPt - startPtThreshold);

        SSresThreshold    = sum(sum((subPts - idealPtsThreshold).^2));
        SStotThreshold    = sum(sum((subPts - mean(subPts, 1)).^2));
        R2_Threshold      = 1 - SSresThreshold / SStotThreshold;
    else
        idealPtsThreshold = [];
        R2_Threshold      = NaN;
    end

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



    % =====================================================================
    % === ANNOTATIONS =====================================================
    % =====================================================================

    % Top-left details block
    statsStr = sprintf(['Trial Completion Time: %.2f s\n' ...
        '%.0f mm Completion Time: %.2f s\n' ...
        'Endpoint Accuracy (X|Y|Z): (%.1f | %.1f | %.1f) mm\n' ...
        'Endpoint Accuracy (2D|3D): (%.1f | %.1f) mm\n' ...
        'Total R^2 Fit: %.3f\n' ...
        '%.0f mm R^2 Fit: %.3f'], ...
        trialCompletionTime, crossThresh, timeAtThreshold, errX, errY, errZ, rXY, rXYZ,...
        R2_total, crossThresh , R2_Threshold);

    topEdge = 0.99;
    boxW    = 0.22;

    ann = annotation(fig, 'textbox', [0.01, 0, boxW, 0.01], 'String', statsStr, ...
        'Interpreter', 'none', 'EdgeColor', 'none', 'BackgroundColor', 'none', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
        'FontSize', Globals.fontSizeTitle - 2, 'FontWeight', 'normal', ...
        'FitBoxToText', 'on');

    % Re-anchor so the top edge sits at topEdge, after MATLAB has resized
    % the box to fit statsStr
    pos = ann.Position;
    ann.Position = [0.005, topEdge - pos(4), boxW, pos(4)];

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
    ax3d.SortMethod = 'childorder';

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

    % === Target reference lines ===
    % Red target X line
    plot3(ax3d, [targetScreenPos(1) targetScreenPos(1)], [0 0], zl, '-', 'Color', Colors.redLt, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line along ty-axis (tx=0, tz=0)
    plot3(ax3d, [targetScreenPos(1) targetScreenPos(1)], [0 max(yl)] , [zl(1) zl(1)], '-', 'Color', Colors.redLt, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line at x=xl(1), z from 0 to max, y=0
    % Green target Y line
    plot3(ax3d, xl, [0 0], [targetScreenPos(2) targetScreenPos(2)], '-', 'Color', Colors.greLt, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line along tx-axis (ty=0, tz=0)
    plot3(ax3d, [-xl(1) -xl(1)], [0 max(yl)], [targetScreenPos(2) targetScreenPos(2)], '-', 'Color', Colors.greLt, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line at x=xl(1), z from 0 to max, y=0

    % === World reference lines ===
    % X-axis (y = 0)
    plot3(ax3d, xl, [0 0], [0 0], '-', 'Color', Colors.graMd, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line along tx-axis (ty=0, tz=0)
    plot3(ax3d, [-xl(1) -xl(1)], [0 max(yl)], [0 0], '-', 'Color', Colors.graMd, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line at x=xl(1), z from 0 to max, y=0
    % Y-axis (x = 0)
    plot3(ax3d, [0 0], [0 0], zl, '-', 'Color', Colors.graMd, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line along ty-axis (tx=0, tz=0)
    plot3(ax3d, [yl(1) yl(1)], [0 max(yl)] , [zl(1) zl(1)], '-', 'Color', Colors.graMd, 'LineWidth', Globals.lineWidthAxis, 'HandleVisibility','off'); % line at x=xl(1), z from 0 to max, y=0
   
    % === Data projections === 
    % Projection of X data (tx) onto the bottom wall (tz = min(tz))
    plot3(ax3d, tx, tz, repmat(zl(1), size(tx)), '-', 'Color', [Colors.redMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');
    % Projection of Y data (ty) onto the back wall (tx = min(tx))
    plot3(ax3d, repmat(-xl(1), size(tz)), tz, ty, '-', 'Color', [Colors.greMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');
    % Projection of Z data (tz) onto the back plane (tz = min(tz), spanning tx and ty)
    if (drawShadowXY)
        plot3(ax3d, tx, repmat(yl(1), size(tx)), ty, '-', 'Color', [Colors.bluMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');
    end

    % === Touchscreen outline === 
    screenCornersX = [-planeHalfW  planeHalfW  planeHalfW -planeHalfW];
    screenCornersY = repmat(yl(1), 1, 4); % flat in the shadow plane (tz = yl(1))
    screenCornersZ = [-planeHalfH -planeHalfH  planeHalfH  planeHalfH];
    patch(ax3d, screenCornersX, screenCornersY, screenCornersZ, Colors.bluLt, ...
        'FaceAlpha', 0.10, 'EdgeColor', Colors.bluLt, 'LineWidth', 1, ...
        'HandleVisibility', 'off');

    % === Target marker === 
    targetSide = 8; % mm, full side length of the target square
    half = targetSide / 2;
    targetCornersX = targetScreenPos(1) + [-half  half  half -half];
    targetCornersY = repmat(yl(1), 1, 4); % flat in the shadow plane (tz = yl(1))
    targetCornersZ = targetScreenPos(2) + [-half -half  half  half];
    patch(ax3d, targetCornersX, targetCornersY, targetCornersZ, Colors.magMd, ...
        'FaceAlpha', 1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    % === Registration markers ===
    coarseSide = 56; % mm, full side length of the target square
    coarseHalf = coarseSide / 2;
    ul = Globals.registrationMarkerUL ;
    ur = Globals.registrationMarkerUR ;
    ll = Globals.registrationMarkerLL ;
    lr = Globals.registrationMarkerLR ;
    % Calculate marker positions
    ulX = ul(1) + [-coarseHalf  coarseHalf coarseHalf -coarseHalf];
    urX = ur(1) + [-coarseHalf  coarseHalf coarseHalf -coarseHalf];
    llX = ll(1) + [-coarseHalf  coarseHalf coarseHalf -coarseHalf];
    lrX = lr(1) + [-coarseHalf  coarseHalf coarseHalf -coarseHalf];
    ulY = repmat(yl(1), 1, 4); % flat in the shadow plane (tz = yl(1))
    urY = repmat(yl(1), 1, 4); % flat in the shadow plane (tz = yl(1))
    llY = repmat(yl(1), 1, 4); % flat in the shadow plane (tz = yl(1))
    lrY = repmat(yl(1), 1, 4); % flat in the shadow plane (tz = yl(1))
    ulZ = ul(2) + [-coarseHalf -coarseHalf coarseHalf coarseHalf];
    urZ = ur(2) + [-coarseHalf -coarseHalf coarseHalf coarseHalf];
    llZ = ll(2) + [-coarseHalf -coarseHalf coarseHalf coarseHalf];
    lrZ = lr(2) + [-coarseHalf -coarseHalf coarseHalf coarseHalf];
    % Plot markers
    patch(ax3d, ulX, ulY, ulZ, Colors.graLt, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(ax3d, urX, urY, urZ, Colors.graLt, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(ax3d, llX, llY, llZ, Colors.graLt, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(ax3d, lrX, lrY, lrZ, Colors.graLt, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'HandleVisibility', 'off');


    % === Plot trajectory === 
    % Plot trajectory data
    plot3(ax3d, tx, tz, ty, '-', 'Color', [Colors.graMd 0.5], 'LineWidth', Globals.lineWidthTraj);
    % Plot trajectory shadow
    plot3(ax3d, tx, tz, ty, '.', 'Color', [Colors.graDk], 'LineWidth', Globals.lineWidthTraj,  'HandleVisibility','off');

    % Plot important points
    scatter3(ax3d, tx(1), tz(1), ty(1), 70, 'g', 'filled', 'MarkerEdgeColor', 'k'); % start
    scatter3(ax3d, tx(end), tz(end), ty(end), 70, 'r', 'filled', 'MarkerEdgeColor', 'k'); % end of trajectory

    legend(ax3d, {'Trajectory','Start','End','Screen','Target'}, 'Location', 'best');
    xlabel(ax3d, 'X (mm)');
    ylabel(ax3d, 'Z (mm)'); % visual Y axis carries tz data
    zlabel(ax3d, 'Y (mm)'); % visual Z axis carries ty data
    title(ax3d, 'Raw Finger Trajectory', 'FontSize',Globals.fontSizeTitle);
    grid(ax3d, 'on');
    set(ax3d, 'DataAspectRatio', [1 1 1]); % equal scaling without vis3d's overflow behavior
    set(ax3d, 'YDir', 'reverse'); % reverses Z-data axis (visual Y/depth)
    % set(ax3d, 'XDir', 'reverse'); % flips X-data axis (visual X, horizontal)
    view(ax3d, az, el);


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

    % Plot data (marker frame -- target-relative, centered at 0)
    plot(axX, t, txm, 'Color', Colors.redMd, 'LineWidth', Globals.lineWidthData);
    plot(axY, t, tym, 'Color', Colors.greMd, 'LineWidth', Globals.lineWidthData);
    plot(axZ, t, tzm, 'Color', Colors.bluMd, 'LineWidth', Globals.lineWidthData);

    if (drawEstimated)
        % Shade areas where pose based on real data
        yl_axX = [-xLimM xLimM];   % matches the ylim you set further down for axX
        yl_axY = [-yLimM yLimM];   % matches the ylim you set further down for axX
        yl_axZ = [-zLimM zLimM];   % matches the ylim you set further down for axX
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

    % Plot axis zero lines -- the target itself, since these subplots are
    % in marker (target-relative) frame, where the target always sits at 0
    yline(axX, 0, 'Color', Colors.redLt, 'LineWidth', Globals.lineWidthAxis);
    yline(axY, 0, 'Color', Colors.greLt, 'LineWidth', Globals.lineWidthAxis);
    yline(axZ, 0, 'Color', Colors.bluLt, 'LineWidth', Globals.lineWidthAxis);

    % Axis Labels
    ylabel(axX, 'X (mm)', 'FontWeight','bold');
    ylabel(axY, 'Y (mm)', 'FontWeight','bold');
    ylabel(axZ, 'Z (mm)', 'FontWeight','bold');

    % Plot labels
    title(axX, 'Error vs Time', 'FontSize', 16);
    xlabel(axZ, 'Time (s)', 'FontWeight','bold');

    % Grids
    grid(axX, 'on');
    grid(axY, 'on');
    grid(axZ, 'on');

    % Limits (marker frame)
    ylim(axX, [-xLimM xLimM])
    ylim(axY, [-yLimM yLimM])
    ylim(axZ, [0 zLimM]);

    % Tick mark formatting
    axX.YTick = -xLimM:50:xLimM;
    axY.YTick = -yLimM:50:yLimM ;
    axZ.YTick = 0:50:zLimM;

    % =====================================================================
    % Velocity overlay -- second y-axis on each subplot, plotted AFTER the
    % position-axis (left) limits/ticks above are finalized, so adding the
    % right axis doesn't disturb them. Controlled by 'ShowVelocity'.
    % =====================================================================
    if showVelocity
        yyaxis(axX, 'right');
        plot(axX, t, vxm, '-', 'Color', [Colors.redMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        ylabel(axX, 'Velocity (mm/s)', 'FontWeight','bold');
        axX.YAxis(2).Color = Colors.redMd;
        yyaxis(axX, 'left'); % leave left as the active axis (matches position-axis ylim/ticks set above)

        yyaxis(axY, 'right');
        plot(axY, t, vym, '-', 'Color', [Colors.greMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        ylabel(axY, 'Velocity (mm/s)', 'FontWeight','bold');
        axY.YAxis(2).Color = Colors.greMd;
        yyaxis(axY, 'left');

        yyaxis(axZ, 'right');
        plot(axZ, t, vzm, '-', 'Color', [Colors.bluMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        ylabel(axZ, 'Velocity (mm/s)', 'FontWeight','bold');
        axZ.YAxis(2).Color = Colors.bluMd;
        yyaxis(axZ, 'left');
    end


    % Release hold
    hold ( axX , 'off' ) ;

    % Link axis based on 'x'
    linkaxes([axX, axY, axZ], 'x');

    % Add R2 comparison
    if plotR2Comparison
        if ~isempty(crossIdx) && crossIdx < N
            subPtsForPlot = actualPts(crossIdx:end, :);
        else
            subPtsForPlot = [];
        end
        PlotR2ComparisonFig(actualPts, idealPts, R2_total, ...
            subPtsForPlot, idealPtsThreshold, R2_Threshold, az, el, trialTitle);
    end

end



function runs = findContiguousRuns(mask)
    %FINDCONTIGUOUSRUNS Return [startIdx endIdx] for each run of consecutive true values.
    mask = mask(:);
    d = diff([0; mask; 0]);
    starts = find(d == 1);
    ends   = find(d == -1) - 1;
    runs = [starts, ends];
end


function PlotR2ComparisonFig(actualPts, idealPts, R2_total, subPts, idealPts500, R2_500, az, el, trialTitle)
%PLOTR2COMPARISONFIG Visualize actual trajectory vs. ideal straight-line
%   path, for both the full trial and the threshold segment, side by side.

fig2 = figure('Color', 'w', 'Units', 'normalized', 'Position', [0.15 0.15 0.65 0.5]);

if trialTitle ~= ""
    annotation(fig2, 'textbox', [0 0.93 1 0.05], 'String', ...
        trialTitle + " -- R^2 Fit Comparison", 'Interpreter', 'none', ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'FontSize', Globals.fontSizeTitle, 'FontWeight', 'bold');
end

% --- Panel 1: full trajectory ---
ax1 = subplot(1, 2, 1, 'Parent', fig2);
hold(ax1, 'on');
plot3(ax1, actualPts(:,1), actualPts(:,3), actualPts(:,2), '-', ...
    'Color', Colors.graDk, 'LineWidth', Globals.lineWidthTraj);
plot3(ax1, idealPts(:,1), idealPts(:,3), idealPts(:,2), '--', ...
    'Color', Colors.magMd, 'LineWidth', Globals.lineWidthAxis);
scatter3(ax1, actualPts(1,1), actualPts(1,3), actualPts(1,2), 70, 'g', 'filled', 'MarkerEdgeColor', 'k');
scatter3(ax1, actualPts(end,1), actualPts(end,3), actualPts(end,2), 70, 'r', 'filled', 'MarkerEdgeColor', 'k');
legend(ax1, {'Actual', 'Ideal (straight line)', 'Start', 'End'}, 'Location', 'best');
xlabel(ax1, 'X (mm)'); ylabel(ax1, 'Z (mm)'); zlabel(ax1, 'Y (mm)');
title(ax1, sprintf('Full Trajectory (R^2 = %.3f)', R2_total), 'FontSize', Globals.fontSizeTitle);
grid(ax1, 'on'); set(ax1, 'DataAspectRatio', [1 1 1]);
set(ax1, 'YDir', 'reverse'); 
set(ax1, 'XDir', 'reverse');
view(ax1, az, el);
hold(ax1, 'off');

% --- Panel 2: threshold segment ---
ax2 = subplot(1, 2, 2, 'Parent', fig2);
hold(ax2, 'on');
if ~isempty(subPts)
    plot3(ax2, subPts(:,1), subPts(:,3), subPts(:,2), '-', ...
        'Color', Colors.graDk, 'LineWidth', Globals.lineWidthTraj);
    plot3(ax2, idealPts500(:,1), idealPts500(:,3), idealPts500(:,2), '--', ...
        'Color', Colors.magMd, 'LineWidth', Globals.lineWidthAxis);
    scatter3(ax2, subPts(1,1), subPts(1,3), subPts(1,2), 70, 'g', 'filled', 'MarkerEdgeColor', 'k');
    scatter3(ax2, subPts(end,1), subPts(end,3), subPts(end,2), 70, 'r', 'filled', 'MarkerEdgeColor', 'k');
    legend(ax2, {'Actual', 'Ideal (straight line)', 'Start (threshold crossing)', 'End'}, 'Location', 'best');
    title(ax2, sprintf('Threshold Segment (R^2 = %.3f)', R2_500), 'FontSize', Globals.fontSizeTitle);
else
    text(0.5, 0.5, 'Trajectory never crossed threshold', 'Parent', ax2, ...
        'HorizontalAlignment', 'center');
    title(ax2, 'Threshold Segment (N/A)', 'FontSize', Globals.fontSizeTitle);
end
xlabel(ax2, 'X (mm)'); ylabel(ax2, 'Z (mm)'); zlabel(ax2, 'Y (mm)');
grid(ax2, 'on'); set(ax2, 'DataAspectRatio', [1 1 1]);
set(ax2, 'YDir', 'reverse'); set(ax2, 'XDir', 'reverse');
view(ax2, az, el);
hold(ax2, 'off');

end