function fig = VisualizeNURingTrial(tt, varargin)
    %VISUALIZENURINGTRIAL Plot a single NURing trial: 3D trajectory, error
    %   vs time, and a live motor/virtual-fingertip panel.
    %
    %   This is an extension of PLOTNURINGTRIAL: everything that function
    %   does (left 3D trajectory plot, right-hand stacked X/Y/Z error vs
    %   time plots, static figure + optional animated video export) is
    %   unchanged here. A THIRD column has been added containing:
    %
    %     - Upper 70%: a circular "virtual fingertip & motor" panel showing
    %         * the virtual fingertip position (virtual_x_mm/virtual_y_mm)
    %           as a moving dot, in the fingertip's own frame (0,0 at the
    %           center of the circle)
    %         * three thin fixed reference lines at 35, 145, and 270
    %           degrees (motors A, B, C respectively), each ending in a
    %           small labeled marker at the circle's radius
    %         * three THICK lines along the same three directions whose
    %           length grows/shrinks with that motor's PWM output (PWM 0 =
    %           full power = full radius; PWM 2047 = no output = zero
    %           length), colored Colors.oraMd / Colors.cyaMd / Colors.magMd
    %           for motors A / B / C respectively
    %         * a line from the center pointing toward the target (i.e.
    %           the direction of travel needed to reach the target),
    %           derived from dx_mm/dy_mm (the target-relative position)
    %     - Lower 30%: a single "Motor PWM Output" plot, in the same style
    %         as the Error vs Time plots, with three lines (one per motor,
    %         same Colors.oraMd / Colors.cyaMd / Colors.magMd coloring)
    %         showing each motor's PWM output vs. time, as a 0-100%
    %         percentage of max output (100% = PWM 0, 0% = PWM 2047).
    %
    %   Both new elements animate (frame-accurate, same scheme as the rest
    %   of the figure) when 'Animate' is true, and are included in the
    %   exported video. The PWM-vs-time plot animates by growing in place
    %   (with a moving "current value" marker per line), exactly like the
    %   Error vs Time plots in the middle column.
    %
    %   tt is the per-trial timetable produced by PARSENURINGTRIALS (i.e.
    %   trials.data{i}), and must contain variables t_secs, dx_mm, dy_mm,
    %   dz_mm. For the new third column it should also contain pwm_a,
    %   pwm_b, pwm_c, virtual_x_mm, and virtual_y_mm; if any of these are
    %   missing, that element is filled with NaN (and a one-time warning is
    %   issued) so the rest of the figure still renders.
    %
    %   fig = VISUALIZENURINGTRIAL(tt, 'Name', Value, ...) supports
    %   everything PLOTNURINGTRIAL supports:
    %
    %       'TargetScreenPosition' (default [0 0])
    %       'PlaneWidth'      (default 527)
    %       'PlaneHeight'     (default 296)
    %       'Azimuth'        (default -30)
    %       'Elevation'      (default 30)
    %       'TrialTitle'      (default '')
    %       'DrawShadowXY'    (default false)
    %       'DrawEstimated'   (default false)
    %       'Threshold'       (default 15)
    %       'ShowVelocity'    (default true)
    %       'Animate'         (default false)
    %       'SourceFilename'  (default '')
    %       'VideoFrameRate'  (default 30)
    %       'SetZLimit'       (default 0)
    %
    %   plus the following NEW parameters:
    %
    %       'ColumnWidths'    (default [0.40 0.35 0.25])
    %                         - 1x3 vector of fractions (must sum to 1.0)
    %                           giving the width of, in order: (1) the 3D
    %                           trajectory plot, (2) the stacked X/Y/Z
    %                           error-vs-time plots, (3) the new virtual
    %                           fingertip/motor column. Supersedes the old
    %                           'LeftWidth' parameter from PLOTNURINGTRIAL.
    %       'CircleRadiusMM'  (default 18)
    %                         - radius, in mm, of the virtual fingertip /
    %                           motor circle. Motor lines at full power
    %                           (PWM = 0) reach exactly this radius.
    %       'MaxPWM'          (default 2047)
    %                         - PWM value corresponding to zero motor
    %                           output (PWM = 0 is full output). Used both
    %                           to scale the motor lines in the circle and
    %                           to compute the 0-100% PWM-vs-time plot.
    %
    %   Example:
    %       [headers, trials] = ParseNURingTrials('+TrialData');
    %       row = trials(trials.trial_key == "u123_t284", :);
    %       hdr = headers(headers.trial_key == "u123_t284", :);
    %       tt  = row.data{1};
    %       VisualizeNURingTrial(tt, ...
    %           'TargetScreenPosition', [hdr.target_screen_position_x_mm, ...
    %                                    hdr.target_screen_position_y_mm], ...
    %           'TrialTitle', row.filename(1), ...
    %           'Animate', true, ...
    %           'SourceFilename', row.filepath(1));
    %
    %   See also PARSENURINGTRIALS, PLOTNURINGTRIAL.

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
    addParameter(p, 'ColumnWidths', [0.40 0.35 0.25], ...
        @(x) isnumeric(x) && numel(x)==3 && all(x > 0) && abs(sum(x) - 1.0) < 1e-3);
    addParameter(p, 'DrawShadowXY', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'DrawEstimated', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Threshold', 15, @isnumeric);
    addParameter(p, 'ShowVelocity', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Animate', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'SourceFilename', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'VideoFrameRate', 30, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'SetZLimit', 0, @(x) isnumeric(x) && isscalar(x) );
    addParameter(p, 'CircleRadiusMM', 18, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'MaxPWM', 2047, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, tt, varargin{:});

    az                  = p.Results.Azimuth;
    el                  = p.Results.Elevation;
    targetScreenPos     = p.Results.TargetScreenPosition; % [x y] mm on screen
    planeWidth          = p.Results.PlaneWidth;
    planeHeight         = p.Results.PlaneHeight;
    trialTitle          = string(p.Results.TrialTitle);
    columnWidths        = p.Results.ColumnWidths(:)' / sum(p.Results.ColumnWidths); % normalize defensively
    drawShadowXY        = p.Results.DrawShadowXY;
    drawEstimated       = p.Results.DrawEstimated;
    crossThresh         = p.Results.Threshold;
    showVelocity        = logical(p.Results.ShowVelocity);
    animatePlots        = logical(p.Results.Animate);
    sourceFilename      = string(p.Results.SourceFilename);
    videoFrameRate      = p.Results.VideoFrameRate;
    setZLimit           = p.Results.SetZLimit;
    radiusMM            = p.Results.CircleRadiusMM;
    maxPWM              = p.Results.MaxPWM;

    % targetPos: target's 3D position in the SAME (now screen-relative)
    % frame as the offset trajectory below -- on the screen plane (z=0),
    % at its on-screen (x,y) location.
    targetPos = [targetScreenPos(1), targetScreenPos(2), 0];

    requiredVars = {'t_secs','dx_mm','dy_mm','dz_mm'}; % Pull position based on compensated fingertip
    for i = 1:numel(requiredVars)
        if ~ismember(requiredVars{i}, tt.Properties.VariableNames)
            error('VisualizeNURingTrial:missingVar', ...
                'Input timetable is missing required variable "%s".', requiredVars{i});
        end
    end

    % Optional variables needed for the new third column. Fall back to
    % NaN (with a one-time warning) if a trial happens not to have them,
    % rather than erroring out the whole plot.
    optionalVars = {'pwm_a','pwm_b','pwm_c','virtual_x_mm','virtual_y_mm'};
    for i = 1:numel(optionalVars)
        if ~ismember(optionalVars{i}, tt.Properties.VariableNames)
            warning('VisualizeNURingTrial:missingOptionalVar', ...
                'Input timetable is missing "%s"; filling with NaN for the motor/virtual-fingertip panel.', ...
                optionalVars{i});
            tt.(optionalVars{i}) = nan(height(tt), 1);
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

    if (setZLimit > 0)
        zMax = setZLimit;
    else
        zMax = max(abs(tz));
    end

    xLim = ceil( (( Globals.screenWidth/2)+100)/50)*50;
    yLim = ceil( ((Globals.screenHeight/2)+100)/50)*50;
    zLim = ceil(zMax/50) * 50 ;

    % Marker-frame axis limits (target-relative, centered at 0) -- used for
    % the three stacked time-series subplots on the right. No plane
    % half-extent term here since those subplots aren't showing the screen.
    maxDx = max(abs(tt.dx_mm) ) ;
    maxDy = max(abs(tt.dy_mm) ) ; 
    maxDz = max(abs(tt.dz_mm) ) ; 
    xLimM = ceil( maxDx / 50 ) * 50 ;
    yLimM = ceil( maxDy / 50 ) * 50 ; 
    zLimM = ceil( maxDz / 100) * 100; 

    % Fixed velocity (right-yyaxis) limits, computed up front from the
    % FULL velocity arrays -- same idea as xLimM/yLimM/zLimM above. Setting
    % these once, before any plotting/animation happens, prevents MATLAB's
    % auto-scaling from rescaling (and visually jumping) the right axis as
    % the animated velocity line grows frame-by-frame.
    vLimX = max(abs(vxm)) * 1.1;
    vLimY = max(abs(vym)) * 1.1;
    vLimZ = max(abs(vzm)) * 1.1;
    if vLimX == 0, vLimX = 1; end
    if vLimY == 0, vLimY = 1; end
    if vLimZ == 0, vLimZ = 1; end

    % =====================================================================
    % === THIRD COLUMN DATA: virtual fingertip + motor PWM ===============
    % =====================================================================
    % Virtual fingertip position -- already in its own frame, with (0,0)
    % at the center of the circle, so it's used directly with no offset.
    vx = tt.virtual_x_mm;
    vy = tt.virtual_y_mm;

    % Motor direction unit vectors at 35/145/270 degrees (standard math
    % convention, counter-clockwise from +X). MATLAB axes have Y increasing
    % upward (same as dx_mm/dy_mm's already-corrected sign convention), so
    % -- unlike the OpenCV reference snippet, which flips Y because image
    % rows increase downward -- no Y negation is needed here.
    motorAngles = [35 145 270]; % degrees: A, B, C
    motorUx = cosd(motorAngles); % 1x3
    motorUy = sind(motorAngles); % 1x3

    % PWM -> output fraction: PWM 0 = full power (fraction 1), PWM = MaxPWM
    % = no output (fraction 0). Clipped to [0,1] in case of out-of-range
    % samples.
    pwmRaw = [tt.pwm_a, tt.pwm_b, tt.pwm_c]; % Nx3 (A,B,C)
    pwmFrac = (maxPWM - pwmRaw) / maxPWM;
    pwmFrac = min(max(pwmFrac, 0), 1);

    % Per-motor line endpoints (Nx3 each), scaled so fraction=1 reaches
    % exactly the circle's radius.
    rMotor    = pwmFrac * radiusMM;          % Nx3
    motorLineX = rMotor .* motorUx;          % Nx3, broadcast over columns
    motorLineY = rMotor .* motorUy;          % Nx3

    % PWM-vs-time plot values: 0-100%, percent of max output, one column
    % per motor (A, B, C).
    pwmPct = pwmFrac * 100; % Nx3

    % Target-direction line: dx_mm/dy_mm is the fingertip's position
    % relative to the target (target sits at the origin of that frame), so
    % the direction FROM the fingertip TO the target is the negated,
    % normalized (dx_mm, dy_mm) vector. Drawn from the circle's center out
    % to the radius, in the direction the fingertip needs to move.
    targetVec  = -[tt.dx_mm, tt.dy_mm];                  % Nx2
    targetNorm = sqrt(sum(targetVec.^2, 2));             % Nx1
    targetNorm(targetNorm == 0 | isnan(targetNorm)) = eps; % avoid divide-by-zero
    targetDir  = targetVec ./ targetNorm;                % Nx2, unit vectors
    targetLineX = targetDir(:,1) * radiusMM;             % Nx1
    targetLineY = targetDir(:,2) * radiusMM;             % Nx1

    % Circle-panel axis limits: must cover the circle itself AND the
    % virtual fingertip's actual excursion (which, per the fixed mm-scale
    % used here, may legitimately fall outside the circle's radius).
    maxVx = max(abs(vx), [], 'omitnan');
    maxVy = max(abs(vy), [], 'omitnan');
    if isempty(maxVx) || isnan(maxVx), maxVx = 0; end
    if isempty(maxVy) || isnan(maxVy), maxVy = 0; end
    circleLim = max([radiusMM * 1.3, maxVx * 1.15, maxVy * 1.15]);
    if circleLim <= 0 || isnan(circleLim), circleLim = radiusMM * 1.3; end

    % Index used to render the "static" (non-animated) state of the
    % circle panel (it's an instantaneous indicator, not a trajectory):
    % the trial's final sample, consistent with the 3D plot's endpoint
    % marker showing where the trajectory ended up. During animation,
    % this is what every frame walks toward and what's left on screen
    % once the animation completes.
    idxStatic = numel(t);

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


    % =====================================================================
    % === NEW FIGURE ======================================================
    % =====================================================================

    % New figure
    fig = figure('Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.75 0.8]);

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
        'Total R^2 Fit: %.3f'], ...
        trialCompletionTime, crossThresh, timeAtThreshold, errX, errY, errZ, rXY, rXYZ,...
        R2_total);

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
    margin       = 0.04;   % outer margin
    gap          = 0.05;   % gap between columns, and between stacked plots
    topMargin    = 0.06;   % extra space reserved at top for the title

    usableW = 1 - 2*margin;
    usableH = 1 - 2*margin - topMargin;

    % Three columns now instead of two: widths come from ColumnWidths
    % (validated/normalized above to sum to 1.0). Two internal gaps
    % separate the three columns.
    colSpaceW = usableW - 2*gap;
    colW      = columnWidths * colSpaceW; % [col1W col2W col3W]

    leftAxX  = margin;                 % column 1 (3D plot)
    leftAxW  = colW(1);
    rightAxX = leftAxX + leftAxW + gap; % column 2 (stacked X/Y/Z error plots)
    rightAxW = colW(2);
    col3X    = rightAxX + rightAxW + gap; % column 3 (virtual fingertip/motor panel)
    col3W    = colW(3);



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
    % Projection onto the floor: visual-Z (ty, vertical) held constant at
    % zl(1) = -yLim (the bottom of the vertical range), spanning tx and tz
    hShadowX = plot3(ax3d, tx, tz, repmat(zl(1), size(tx)), '-', 'Color', [Colors.redMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');
    % Projection onto the back wall: visual-X (tx, horizontal) held constant
    % at -xl(1) = +xLim (the far horizontal edge), spanning tz and ty
    hShadowY = plot3(ax3d, repmat(-xl(1), size(tz)), tz, ty, '-', 'Color', [Colors.greMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');
    % Projection onto the front plane: visual-Y/depth (tz) held constant at
    % yl(1) = 0 (nearest depth), spanning tx and ty
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
    hTrajLine = plot3(ax3d, tx, tz, ty, '-', 'Color', [Colors.graMd 0.5], 'LineWidth', Globals.lineWidthTraj);
    % Plot trajectory shadow
    hTrajDots = plot3(ax3d, tx, tz, ty, '.', 'Color', [Colors.graDk], 'LineWidth', Globals.lineWidthTraj,  'HandleVisibility','off');

    % Plot important points
    hStartPt = scatter3(ax3d, tx(1), tz(1), ty(1), 70, 'g', 'filled', 'MarkerEdgeColor', 'k'); % start
    hEndPt   = scatter3(ax3d, tx(end), tz(end), ty(end), 70, 'r', 'filled', 'MarkerEdgeColor', 'k'); % end of trajectory

    % legend(ax3d, {'Trajectory','Target'}, 'Location', 'best');
    xlabel(ax3d, 'X (mm)');
    ylabel(ax3d, 'Z (mm)'); % visual Y axis carries tz data
    zlabel(ax3d, 'Y (mm)'); % visual Z axis carries ty data
    title(ax3d, 'Raw Finger Trajectory', 'FontSize',Globals.fontSizeTitle);
    grid(ax3d, 'on');
    set(ax3d, 'DataAspectRatio', [1 1 1]); % equal scaling without vis3d's overflow behavior
    set(ax3d, 'YDir', 'reverse'); % reverses Z-data axis (visual Y/depth)
    view(ax3d, az, el);


    hold(ax3d, 'off');



    % =====================================================================
    % === MIDDLE: THREE STACKED SUBPLOTS ==================================
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
    hPosLineX = plot(axX, t, txm, 'Color', Colors.redMd, 'LineWidth', Globals.lineWidthData);
    hPosLineY = plot(axY, t, tym, 'Color', Colors.greMd, 'LineWidth', Globals.lineWidthData);
    hPosLineZ = plot(axZ, t, tzm, 'Color', Colors.bluMd, 'LineWidth', Globals.lineWidthData);

    if (drawEstimated)
        % Shade areas where pose based on real data
        yl_axX = [-xLimM xLimM];   % matches the ylim you set further down for axX
        yl_axY = [-yLimM yLimM];   % matches the ylim you set further down for axX
        yl_axZ = [-zLimM zLimM];   % matches the ylim you set further down for axX
        runs = FindContiguousRuns(vis == 1);  % helper below, or inline it
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
    axZ.YTick = 0:100:zLimM;

    % =====================================================================
    % Velocity overlay -- second y-axis on each subplot, plotted AFTER the
    % position-axis (left) limits/ticks above are finalized, so adding the
    % right axis doesn't disturb them. Controlled by 'ShowVelocity'.
    % =====================================================================
    if showVelocity
        yyaxis(axX, 'right');
        hVelLineX = plot(axX, t, vxm, '-', 'Color', [Colors.redMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        ylabel(axX, 'Velocity (mm/s)', 'FontWeight','bold');
        axX.YAxis(2).Color = Colors.redMd;
        ylim(axX, [-vLimX vLimX]); % fixed -- prevents auto-rescale jump during animation
        yyaxis(axX, 'left'); % leave left as the active axis (matches position-axis ylim/ticks set above)

        yyaxis(axY, 'right');
        hVelLineY = plot(axY, t, vym, '-', 'Color', [Colors.greMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        ylabel(axY, 'Velocity (mm/s)', 'FontWeight','bold');
        axY.YAxis(2).Color = Colors.greMd;
        ylim(axY, [-vLimY vLimY]);
        yyaxis(axY, 'left');

        yyaxis(axZ, 'right');
        hVelLineZ = plot(axZ, t, vzm, '-', 'Color', [Colors.bluMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        ylabel(axZ, 'Velocity (mm/s)', 'FontWeight','bold');
        axZ.YAxis(2).Color = Colors.bluMd;
        ylim(axZ, [-vLimZ vLimZ]);
        yyaxis(axZ, 'left');
    else
        hVelLineX = [];
        hVelLineY = [];
        hVelLineZ = [];
    end


    % Release hold
    hold ( axX , 'off' ) ;

    % Link axis based on 'x'
    linkaxes([axX, axY, axZ], 'x');

    % =====================================================================
    % === THIRD COLUMN: VIRTUAL FINGERTIP / MOTOR PANEL ===================
    % =====================================================================
    col3Gap   = gap;                                 % vertical gap between circle and PWM-vs-time panels
    circleAxH = (usableH - col3Gap) * 0.7;           % upper 70%
    pwmAxH    = (usableH - col3Gap) * 0.3;           % lower 30%
    pwmAxY    = margin;
    circleAxY = margin + pwmAxH + col3Gap;

    % Per-motor colors, shared between the circle's thick PWM lines and
    % the PWM-vs-time plot below, so the two panels read together.
    motorColors = [Colors.oraDk; Colors.cyaDk; Colors.magDk]; % A, B, C
    motorLabels = {'A','B','C'};

    % --- Upper: virtual fingertip / motor circle ---
    axCircle = axes(fig, 'Position', [col3X, circleAxY, col3W, circleAxH]);
    hold(axCircle, 'on');
    axis(axCircle, 'equal');
    xlim(axCircle, [-circleLim circleLim]);
    ylim(axCircle, [-circleLim circleLim]);
    box(axCircle, 'on');
    title(axCircle, 'Virtual Fingertip & Motor Output', 'FontSize', Globals.fontSizeTitle);

    % Circle outline + center point
    circTheta = linspace(0, 2*pi, 200);
    plot(axCircle, radiusMM*cos(circTheta), radiusMM*sin(circTheta), '-', ...
        'Color', Colors.graDk, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(axCircle, 0, 0, '.', 'Color', Colors.graDk, 'MarkerSize', 14, 'HandleVisibility', 'off');

    hPwmLine = gobjects(1,3);
    for m = 1:3
        % Thin fixed direction reference line (full radius, static)
        plot(axCircle, [0 motorUx(m)*radiusMM], [0 motorUy(m)*radiusMM], '-', ...
            'Color', Colors.graMd, 'LineWidth', 1, 'HandleVisibility', 'off');
        % Fixed motor marker + label, at the full-radius position
        scatter(axCircle, motorUx(m)*radiusMM, motorUy(m)*radiusMM, 110, ...
            Colors.graDk, 'filled', 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
        text(axCircle, motorUx(m)*radiusMM, motorUy(m)*radiusMM, motorLabels{m}, ...
            'Color', 'w', 'FontWeight', 'bold', 'FontSize', 9, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        % Thick variable-length PWM line (animated; initialized at the
        % static/final-sample value)
        hPwmLine(m) = plot(axCircle, [0 motorLineX(idxStatic,m)], [0 motorLineY(idxStatic,m)], '-', ...
            'Color', motorColors(m,:), 'LineWidth', 6);
    end

    % Target-direction line. Distinct from the motor colors (which now
    % occupy orange/cyan/magenta) and from the virtual fingertip dot.
    hTargetLine = plot(axCircle, [0 targetLineX(idxStatic)], [0 targetLineY(idxStatic)], '-', ...
        'Color', Colors.redMd, 'LineWidth', 2);

    % Virtual fingertip dot -- a distinct color not used by the motor
    % lines (orange/cyan/magenta), the target line (red), or the circle's
    % gray reference elements.
    virtualDotColor = [0.85 0.85 0.05]; % yellow
    hVirtualDot = scatter(axCircle, vx(idxStatic), vy(idxStatic), 90, ...
        virtualDotColor, 'filled', 'MarkerEdgeColor', 'k');

    legend(axCircle, [hPwmLine(1) hTargetLine hVirtualDot], ...
        {'Motor PWM', 'To Target', 'Virtual Fingertip'}, ...
        'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 7);

    hold(axCircle, 'off');

    % --- Lower: motor PWM output vs. time (single plot, 3 lines) ---
    axPwm = axes(fig, 'Position', [col3X, pwmAxY, col3W, pwmAxH]);
    hold(axPwm, 'on');

    hPwmPctLine = gobjects(1,3);
    hPwmPctLine(1) = plot(axPwm, t, pwmPct(:,1), 'Color', motorColors(1,:), 'LineWidth', Globals.lineWidthData);
    hPwmPctLine(2) = plot(axPwm, t, pwmPct(:,2), 'Color', motorColors(2,:), 'LineWidth', Globals.lineWidthData);
    hPwmPctLine(3) = plot(axPwm, t, pwmPct(:,3), 'Color', motorColors(3,:), 'LineWidth', Globals.lineWidthData);

    xlim(axPwm, [t(1) t(end)]);
    ylim(axPwm, [0 100]);

    axPwm.YTick = 0:25:100;
    ylabel(axPwm, '% of Max Output', 'FontWeight', 'bold');
    xlabel(axPwm, 'Time (s)', 'FontWeight', 'bold');
    title(axPwm, 'Motor PWM Output', 'FontSize', Globals.fontSizeTitle);
    grid(axPwm, 'on');
    legend(axPwm, hPwmPctLine, motorLabels, ...
        'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 7);

    hold(axPwm, 'off');

    % =====================================================================
    % === ANIMATION / VIDEO EXPORT ========================================
    % =====================================================================
    if animatePlots
        AnimateAndExportVideo(fig, t, tx, ty, tz, txm, tym, tzm, vxm, vym, vzm, ...
            ax3d, axX, axY, axZ, ...
            hTrajLine, hTrajDots, hStartPt, hEndPt, hShadowX, hShadowY, ...
            hPosLineX, hPosLineY, hPosLineZ, ...
            hVelLineX, hVelLineY, hVelLineZ, ...
            showVelocity, sourceFilename, trialTitle, videoFrameRate, ...
            xl, yl, zl, ...
            motorLineX, motorLineY, targetLineX, targetLineY, vx, vy, ...
            pwmPct, motorColors, ...
            hPwmLine, hTargetLine, hVirtualDot, hPwmPctLine, axPwm);
    end

end




%% ========================================================================
% === CONTINUOUS RUN MASK =================================================
% =========================================================================

function runs = FindContiguousRuns(mask)
    %FINDCONTIGUOUSRUNS Return [startIdx endIdx] for each run of consecutive true values.
    mask = mask(:);
    d = diff([0; mask; 0]);
    starts = find(d == 1);
    ends   = find(d == -1) - 1;
    runs = [starts, ends];
end



%% ========================================================================
% === ANIMATION CONTROLLER ================================================
% =========================================================================
function AnimateAndExportVideo(fig, t, tx, ty, tz, txm, tym, tzm, vxm, vym, vzm, ...
        ax3d, axX, axY, axZ, ...
        hTrajLine, hTrajDots, hStartPt, hEndPt, hShadowX, hShadowY, ...
        hPosLineX, hPosLineY, hPosLineZ, ...
        hVelLineX, hVelLineY, hVelLineZ, ...
        showVelocity, sourceFilename, trialTitle, videoFrameRate, ...
        xl, yl, zl, ...
        motorLineX, motorLineY, targetLineX, targetLineY, vx, vy, ...
        pwmPct, motorColors, ...
        hPwmLine, hTargetLine, hVirtualDot, hPwmPctLine, axPwm)
    %ANIMATEANDEXPORTVIDEO Replay the trajectory and position/velocity
    %   subplots over time and export a video whose PLAYBACK duration
    %   matches the trial's real-world duration (t(end)-t(1)), with
    %   FRAME-ACCURATE timing: the video is rendered at a fixed frame rate
    %   (videoFrameRate), but the data shown at each output frame is
    %   whichever sample was current at that frame's real elapsed time --
    %   so irregular/non-uniform sample spacing (dropped frames, variable
    %   loop timing) still displays with correct real-world delay between
    %   visible updates, rather than giving every sample equal screen time.
    %
    %   This temporarily hides the fully-drawn static trajectory/position/
    %   velocity/PWM-vs-time lines and points, draws growing "animated"
    %   copies plus a moving "current position" marker on top frame-by-
    %   frame, captures each frame, and restores the original static lines
    %   once done so the figure handed back to the caller still shows the
    %   complete static plot.
    %
    %   The circle-panel elements (motor PWM lines, target-direction line,
    %   virtual fingertip dot) represent instantaneous current-frame state
    %   rather than an accumulating trajectory, so -- unlike the 3D/time-
    %   series elements -- there's no "static full path" vs. "growing
    %   animated subset" distinction needed for them: their handles are
    %   simply updated in place each frame, and are left showing the final
    %   frame's values once the loop completes (matching the static value
    %   they were initialized to before this function was called). The
    %   PWM-vs-time plot, however, IS a time series (like the Error vs
    %   Time plots), so it uses the same hide/grow/restore pattern as
    %   those.

    numSamples = numel(t);
    duration   = t(end) - t(1);
    if numSamples < 2 || duration <= 0
        warning('VisualizeNURingTrial:AnimateAndExportVideo:badTimebase', ...
            'Cannot animate: trial has fewer than 2 samples or non-positive duration.');
        return;
    end

    % ---------------------------------------------------------------
    % Resolve output video filename: prefer SourceFilename, fall back to
    % TrialTitle (commonly set to the source CSV's filename by the
    % caller), and only fall back to a generic name if neither is given.
    % ---------------------------------------------------------------
    if sourceFilename ~= ""
        baseNameSource = sourceFilename;
    elseif trialTitle ~= ""
        baseNameSource = trialTitle;
    else
        baseNameSource = "";
    end

    if baseNameSource == ""
        warning('VisualizeNURingTrial:AnimateAndExportVideo:noSourceFilename', ...
            'Neither ''SourceFilename'' nor ''TrialTitle'' was provided; saving animation as ''NURingTrialAnimation.mp4'' in the current folder.');
        outFile = fullfile(pwd, 'NURingTrialAnimation.mp4');
    else
        [folder, name, ~] = fileparts(char(baseNameSource));
        if folder == ""
            folder = pwd;
        end
        outFile = fullfile(folder, [name '.mp4']);
    end

    % ---------------------------------------------------------------
    % Fixed-frame-rate output time grid, spanning the trial's real-world
    % duration exactly. For each output frame time, find the latest
    % sample at or before that time -- this is what makes timing
    % frame-accurate: a long gap between samples holds the last frame
    % for several output frames (correct real-world delay), while
    % several samples arriving faster than the frame rate get collapsed
    % into a single output frame (nothing to show them individually).
    % ---------------------------------------------------------------
    numOutFrames = max(2, round(duration * videoFrameRate) + 1);
    outTimes     = linspace(t(1), t(end), numOutFrames);
    % sampleIdxForFrame(j) = index of the latest sample with t <= outTimes(j)
    sampleIdxForFrame = zeros(1, numOutFrames);
    searchFrom = 1;
    for j = 1:numOutFrames
        idx = find(t(searchFrom:end) <= outTimes(j), 1, 'last');
        if isempty(idx)
            idx = 1;
        else
            idx = searchFrom + idx - 1;
        end
        sampleIdxForFrame(j) = idx;
        searchFrom = idx; % monotonic: never search earlier than the last match
    end

    try
        vidObj = VideoWriter(outFile, 'MPEG-4');
    catch
        % MPEG-4 profile isn't available on all platforms (e.g. some Linux
        % installs) -- fall back to Motion JPEG AVI in that case.
        [folder, name, ~] = fileparts(outFile);
        outFile = fullfile(folder, [name '.avi']);
        vidObj = VideoWriter(outFile, 'Motion JPEG AVI');
    end
    vidObj.FrameRate = videoFrameRate;
    open(vidObj);

    % ---------------------------------------------------------------
    % Hide the fully-drawn static lines/points during the animation, and
    % create empty "animated" counterparts to grow frame-by-frame, plus a
    % moving "current position" marker on each axis.
    % ---------------------------------------------------------------
    set([hTrajLine, hTrajDots, hStartPt, hEndPt, hShadowX, hShadowY], 'Visible', 'off');
    set([hPosLineX, hPosLineY, hPosLineZ], 'Visible', 'off');
    if showVelocity
        set([hVelLineX, hVelLineY, hVelLineZ], 'Visible', 'off');
    end
    set(hPwmPctLine, 'Visible', 'off');

    hold(ax3d, 'on');
    hAnimTraj  = plot3(ax3d, nan, nan, nan, '-', 'Color', Colors.graDk, 'LineWidth', Globals.lineWidthTraj, 'HandleVisibility', 'off');
    hAnimPoint = scatter3(ax3d, nan, nan, nan, 15, 'm', 'filled', 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    % Animated counterparts of the red/green wall-projection shadows
    hAnimShadowX = plot3(ax3d, nan, nan, nan, '-', 'Color', [Colors.redMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');
    hAnimShadowY = plot3(ax3d, nan, nan, nan, '-', 'Color', [Colors.greMd 0.25], 'LineWidth', Globals.lineWidthData, 'HandleVisibility','off');
    hold(ax3d, 'off');

    % NOTE: hold is kept 'on' through ALL object creation on a given right-
    % hand axis (position line + point + velocity line), then released
    % once at the end. Calling plot(...) with hold OFF clears the axes'
    % existing children first -- releasing hold between the position line
    % and the velocity line below previously wiped out the just-created
    % position line/point handles, leaving them invalid by the time the
    % frame loop tried to update them.
    hold(axX, 'on');
    hAnimLineX = plot(axX, nan, nan, 'Color', Colors.redMd, 'LineWidth', Globals.lineWidthData);
    hAnimPtX = scatter(axX, nan, nan, 15, Colors.redDk, 'filled', 'HandleVisibility', 'off');

    hold(axY, 'on');
    hAnimLineY = plot(axY, nan, nan, 'Color', Colors.greMd, 'LineWidth', Globals.lineWidthData);
    hAnimPtY = scatter(axY, nan, nan, 15, Colors.greDk, 'filled', 'HandleVisibility', 'off');

    hold(axZ, 'on');
    hAnimLineZ = plot(axZ, nan, nan, 'Color', Colors.bluMd, 'LineWidth', Globals.lineWidthData);
    hAnimPtZ = scatter(axZ, nan, nan, 15, Colors.bluDk, 'filled', 'HandleVisibility', 'off');

    if showVelocity
        yyaxis(axX, 'right');
        hAnimVelLineX = plot(axX, nan, nan, '-', 'Color', [Colors.redMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        yyaxis(axX, 'left');

        yyaxis(axY, 'right');
        hAnimVelLineY = plot(axY, nan, nan, '-', 'Color', [Colors.greMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        yyaxis(axY, 'left');

        yyaxis(axZ, 'right');
        hAnimVelLineZ = plot(axZ, nan, nan, '-', 'Color', [Colors.bluMd 0.4], 'LineWidth', Globals.lineWidthData*0.75);
        yyaxis(axZ, 'left');
    end

    hold(axX, 'off');
    hold(axY, 'off');
    hold(axZ, 'off');

    % Animated counterparts of the PWM-vs-time lines (one growing line +
    % one "current value" point per motor), same growth pattern as the
    % X/Y/Z error subplots above.
    hold(axPwm, 'on');
    hAnimPwmLine = gobjects(1,3);
    hAnimPwmPt   = gobjects(1,3);
    for m = 1:3
        hAnimPwmLine(m) = plot(axPwm, nan, nan, '-', 'Color', motorColors(m,:), 'LineWidth', Globals.lineWidthData);
        hAnimPwmPt(m)   = scatter(axPwm, nan, nan, 15, motorColors(m,:), 'filled', 'HandleVisibility', 'off');
    end
    hold(axPwm, 'off');

    % ---------------------------------------------------------------
    % Frame loop: for each fixed-rate output frame, reveal data up through
    % whichever sample was current at that frame's real elapsed time
    % ---------------------------------------------------------------
    for j = 1:numOutFrames
        k = sampleIdxForFrame(j);

        % 3D trajectory (visual remap: visual X=tx, visual Y=tz, visual Z=ty)
        set(hAnimTraj, 'XData', tx(1:k), 'YData', tz(1:k), 'ZData', ty(1:k));
        set(hAnimPoint, 'XData', tx(k), 'YData', tz(k), 'ZData', ty(k));

        % Wall-projection shadows, growing alongside the trajectory:
        %   red  -- projection onto the bottom wall (visual Z held at zl(1))
        %   green -- projection onto the back wall (visual X held at -xl(1))
        set(hAnimShadowX, 'XData', tx(1:k), 'YData', tz(1:k), 'ZData', repmat(zl(1), size(tx(1:k))));
        set(hAnimShadowY, 'XData', repmat(-xl(1), size(tz(1:k))), 'YData', tz(1:k), 'ZData', ty(1:k));

        % Position subplots (marker frame)
        set(hAnimLineX, 'XData', t(1:k), 'YData', txm(1:k));
        set(hAnimPtX,   'XData', t(k),   'YData', txm(k));
        set(hAnimLineY, 'XData', t(1:k), 'YData', tym(1:k));
        set(hAnimPtY,   'XData', t(k),   'YData', tym(k));
        set(hAnimLineZ, 'XData', t(1:k), 'YData', tzm(1:k));
        set(hAnimPtZ,   'XData', t(k),   'YData', tzm(k));

        % Velocity overlays, if enabled -- these lines live on the right
        % yyaxis of axX/axY/axZ, but their data can be set directly via
        % the handle regardless of which side is currently "active".
        if showVelocity
            set(hAnimVelLineX, 'XData', t(1:k), 'YData', vxm(1:k));
            set(hAnimVelLineY, 'XData', t(1:k), 'YData', vym(1:k));
            set(hAnimVelLineZ, 'XData', t(1:k), 'YData', vzm(1:k));
        end

        % PWM-vs-time plot: growing lines + current-value points, one per
        % motor (A, B, C).
        for m = 1:3
            set(hAnimPwmLine(m), 'XData', t(1:k), 'YData', pwmPct(1:k,m));
            set(hAnimPwmPt(m),   'XData', t(k),   'YData', pwmPct(k,m));
        end

        % --- Circle panel: virtual fingertip / motor lines / target line ---
        % Each of these is an instantaneous current-frame indicator (not
        % an accumulating path), so the handles are updated directly --
        % no separate "static" vs "animated" copy is needed here.
        set(hPwmLine(1), 'XData', [0 motorLineX(k,1)], 'YData', [0 motorLineY(k,1)]);
        set(hPwmLine(2), 'XData', [0 motorLineX(k,2)], 'YData', [0 motorLineY(k,2)]);
        set(hPwmLine(3), 'XData', [0 motorLineX(k,3)], 'YData', [0 motorLineY(k,3)]);
        set(hTargetLine, 'XData', [0 targetLineX(k)], 'YData', [0 targetLineY(k)]);
        set(hVirtualDot, 'XData', vx(k), 'YData', vy(k));

        drawnow;
        frame = getframe(fig);
        writeVideo(vidObj, frame);
    end

    close(vidObj);

    % ---------------------------------------------------------------
    % Clean up: remove the animated overlay objects and restore the
    % original static lines/points, so the figure returned to the caller
    % still shows the complete, static plot. (Circle-panel handles are
    % left as-is -- they already hold the final frame's values, which is
    % exactly what they were initialized to before this function ran.)
    % ---------------------------------------------------------------
    delete([hAnimTraj, hAnimPoint, hAnimShadowX, hAnimShadowY, hAnimLineX, hAnimPtX, hAnimLineY, hAnimPtY, hAnimLineZ, hAnimPtZ]);
    if showVelocity
        delete([hAnimVelLineX, hAnimVelLineY, hAnimVelLineZ]);
    end
    delete([hAnimPwmLine, hAnimPwmPt]);
    set([hTrajLine, hTrajDots, hStartPt, hEndPt, hShadowX, hShadowY], 'Visible', 'on');
    set([hPosLineX, hPosLineY, hPosLineZ], 'Visible', 'on');
    if showVelocity
        set([hVelLineX, hVelLineY, hVelLineZ], 'Visible', 'on');
    end
    set(hPwmPctLine, 'Visible', 'on');

    fprintf('VisualizeNURingTrial: Animation saved to %s\n', outFile);

end