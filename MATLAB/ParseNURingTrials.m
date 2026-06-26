function [headers, trials, summary, userMeans, userMedians] = ParseNURingTrials(rootDir, varargin)
    %PARSENURINGTRIALS Parse NURing trial CSV files into MATLAB tables.
    %
    %   Each trial file is now expected to have the form:
    %
    %       {HEADER}
    %       user_id: 124
    %       target_id: 286
    %       target_screen_position_mm: [-183.92, 48.04]
    %       fingertip_offset: [3.03, 32.73, 19.23]
    %       completion_time: 10.2069
    %       endpoint_error_mm: [-0.5205, 17.6598, 2.6478]
    %       calibration_angle_count: 4
    %       calibration_angles_deg: [0.000000, 90.000000, 180.000000, 270.000000]
    %       control_points_theta: [0.000000, 1.570796, 3.141593, 4.712389]
    %       control_points_radius: [9.685735, 9.682273, 7.470439, 8.990629]
    %       control_points_accel: [0.497382, -2.264590, 3.190859, -1.423652]
    %       stiffness_measurements: [0.185872, 0.059771, 0.120415, 0.242385]
    %       {DATA}
    %       t_secs,target_id,detected,tx_mm,ty_mm,tz_mm,dx_mm,dy_mm,dz_mm,...
    %       0.0000,286,0,-75.0993,...
    %       ...
    %
    %   [headers, trials, summary] = PARSENURINGTRIALS(rootDir) scans rootDir
    %   (the +TrialData folder) for participant subfolders, each containing
    %   trial CSV files named like:
    %       124-286-06252026-184333.csv
    %       ^pid ^target ^date(MMddyyyy) ^time(HHmmss)
    %
    %   and returns:
    %
    %   HEADERS - one row per trial file, parsed from the {HEADER} block:
    %       trial_key                      (string)  - "u<user_id>_t<target_id>",
    %                                                   shared with TRIALS for
    %                                                   cross-referencing
    %       user_id                        (string)
    %       target_id                      (string)
    %       target_screen_position_x_mm    (double)
    %       target_screen_position_y_mm    (double)
    %       fingertip_offset_x_mm          (double)
    %       fingertip_offset_y_mm          (double)
    %       fingertip_offset_z_mm          (double)
    %       completion_time_s              (double)
    %       endpoint_error_x_mm            (double)
    %       endpoint_error_y_mm            (double)
    %       endpoint_error_z_mm            (double)
    %       calibration_angle_count        (double)
    %       calibration_angles_deg         (cell, 1x1 containing a numeric vector)
    %       control_points_theta           (cell, 1x1 containing a numeric vector)
    %       control_points_radius          (cell, 1x1 containing a numeric vector)
    %       control_points_accel           (cell, 1x1 containing a numeric vector)
    %       stiffness_measurements         (cell, 1x1 containing a numeric vector)
    %       filename                       (string)
    %       filepath                       (string)
    %
    %   The calibration/control-point/stiffness fields are stored as cells
    %   holding numeric row vectors (rather than fixed x/y/z-style columns)
    %   since their length is given by calibration_angle_count and isn't
    %   assumed to always be 4. Pull one out with e.g.:
    %       angles = headers.calibration_angles_deg{1};
    %
    %   After each participant's block of trial rows, HEADERS also
    %   contains two synthetic summary rows -- one with the MEAN, one with
    %   the MEDIAN, of that participant's completion_time_s,
    %   endpoint_error_x_mm, and endpoint_error_y_mm (endpoint_error_z_mm
    %   and all other fields are left NaN/blank on these rows, since they
    %   aren't aggregated). Summary rows are identified by
    %   target_id == "MEAN" / "MEDIAN" and can be excluded with e.g.:
    %       headers(~ismember(headers.target_id, ["MEAN","MEDIAN"]), :)
    %
    %   TRIALS - one row per trial file (same as before, plus trial_key):
    %       trial_key        (string)   - "u<user_id>_t<target_id>", shared
    %                                      with HEADERS for cross-referencing
    %       participant_id   (string)   - parsed from filename
    %       target_marker    (string)   - parsed from filename
    %       trial_datetime   (datetime)- parsed from filename date+time
    %       filename         (string)   - original file name
    %       filepath         (string)   - full path to source file
    %       n_samples        (double)   - number of rows in the trial
    %       duration_s        (double)   - t_secs(end) - t_secs(1)
    %       data             (cell)     - 1x1 cell containing a timetable
    %                                      with the per-sample data for
    %                                      that trial (see below)
    %
    %   The per-trial timetable (trials.data{i}) has RowTimes = seconds(t_secs)
    %   and variables:
    %       target_id, detected, tx_mm, ty_mm, tz_mm, dx_mm, dy_mm, dz_mm,
    %       qx, qy, qz, qw, pwm_a, pwm_b, pwm_c,
    %       virtual_x_mm, virtual_y_mm, touch_x_mm, touch_y_mm
    %   Empty touch_* fields are read as NaN.
    %
    %   HEADERS and TRIALS can be cross-referenced directly via trial_key,
    %   e.g.:
    %       row = trials(trials.trial_key == "u123_t284", :);
    %       hdr = headers(headers.trial_key == "u123_t284", :);
    %   or joined wholesale with:
    %       joined = innerjoin(headers, trials, 'Keys', 'trial_key');
    %
    %   SUMMARY is a table with one row per participant:
    %       User    (string) - participant_id
    %       Data    (cell)   - 1x1 cell containing a per-trial table with
    %                           columns:
    %           target_id            (string)
    %           start_euclidean_mm    (double) - sqrt(dx^2+dy^2+dz^2) at the
    %                                             first data sample (t_secs(1))
    %                                             of that trial
    %           completion_time_s    (double)
    %           endpoint_error_x_mm   (double)
    %           endpoint_error_y_mm   (double)
    %           endpoint_error_z_mm   (double)
    %           endpoint_error_2D_mm  (double) - sqrt(x^2+y^2) of endpoint error
    %           endpoint_error_3D_mm  (double) - sqrt(x^2+y^2+z^2) of endpoint error
    %   SUMMARY is also printed to the console automatically, one user at a
    %   time, as:
    %       User 123:
    %       <table>
    %
    %   USERMEANS / USERMEDIANS are tables with one row per participant,
    %   giving the across-trial mean / median of each of the per-trial
    %   metrics in SUMMARY.Data (NaNs omitted):
    %       userID
    %       mean_completion_time / median_completion_time
    %       mean_endpoint_error_x / median_endpoint_error_x
    %       mean_endpoint_error_y / median_endpoint_error_y
    %       mean_endpoint_error_z / median_endpoint_error_z
    %       mean_endpoint_error_2D / median_endpoint_error_2D
    %       mean_endpoint_error_3D / median_endpoint_error_3D
    %   These are also printed to the console automatically, right after
    %   the per-user SUMMARY tables, as "User Means:" / "User Medians:".
    %
    %   [...] = PARSENURINGTRIALS(rootDir, 'Name', Value, ...) supports:
    %       'UseParallel'   (default true)   - use parfor over trial files if
    %                                           Parallel Computing Toolbox and
    %                                           a pool are available
    %       'Participants'  (default {})     - cell array of participant ID
    %                                           strings/folder names to
    %                                           restrict parsing to. Empty
    %                                           means "all participants".
    %
    %   Example:
    %       [headers, trials, summary] = ParseNURingTrials('+TrialData');
    %       p123 = trials(trials.participant_id == "123", :);
    %       firstTrialData = p123.data{1};
    %       plot(firstTrialData.tx_mm)
    %
    %   See also FLATTENNURINGTRIALS (helper, defined as local function below
    %   for converting the nested table into one flat row-per-sample table).

    % ---------------------------------------------------------------------
    % Parse inputs
    % ---------------------------------------------------------------------
    p = inputParser;
    addRequired(p, 'rootDir', @(x) ischar(x) || isstring(x));
    addParameter(p, 'UseParallel', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Participants', {}, @(x) iscell(x) || isstring(x));
    parse(p, rootDir, varargin{:});

    rootDir      = char(p.Results.rootDir);
    useParallel  = logical(p.Results.UseParallel);
    participants = cellstr(p.Results.Participants);

    if ~isfolder(rootDir)
        error('parseNURingTrials:badRoot', 'Root directory not found: %s', rootDir);
    end

    % ---------------------------------------------------------------------
    % Discover participant folders
    % ---------------------------------------------------------------------
    d = dir(rootDir);
    d = d([d.isdir]);
    d = d(~ismember({d.name}, {'.', '..'}));

    if ~isempty(participants)
        d = d(ismember({d.name}, participants));
    end

    if isempty(d)
        warning('parseNURingTrials:noParticipants', ...
            'No participant subfolders found in %s', rootDir);
        headers = emptyHeadersTable();
        trials  = emptyTrialsTable();
        summary = emptySummaryTable();
        userMeans   = emptyUserStatTable('avg');
        userMedians = emptyUserStatTable('med');
        return;
    end

    % ---------------------------------------------------------------------
    % Discover all trial CSV files across participant folders
    % ---------------------------------------------------------------------
    filePaths = {};
    for i = 1:numel(d)
        participantDir = fullfile(rootDir, d(i).name);
        files = dir(fullfile(participantDir, '*.csv'));
        files = files(~[files.isdir]);
        for j = 1:numel(files)
            filePaths{end+1} = fullfile(participantDir, files(j).name); %#ok<AGROW>
        end
    end

    nFiles = numel(filePaths);
    if nFiles == 0
        warning('parseNURingTrials:noFiles', 'No trial CSV files found under %s', rootDir);
        headers = emptyHeadersTable();
        trials  = emptyTrialsTable();
        summary = emptySummaryTable();
        userMeans   = emptyUserStatTable('avg');
        userMedians = emptyUserStatTable('med');
        return;
    end

    % ---------------------------------------------------------------------
    % Decide whether to actually run in parallel
    % ---------------------------------------------------------------------
    runParallel = useParallel && nFiles > 1 && canUseParallel();

    % ---------------------------------------------------------------------
    % Parse each file into a row struct (parfor-friendly: independent work,
    % results collected into a preallocated struct array / cell array)
    % ---------------------------------------------------------------------
    rows = cell(nFiles, 1);

    if runParallel
        parfor k = 1:nFiles
            rows{k} = parseOneTrialFile(filePaths{k});
        end
    else
        for k = 1:nFiles
            rows{k} = parseOneTrialFile(filePaths{k});
        end
    end

    % Drop any files that failed to parse (parseOneTrialFile returns [] and
    % warns on failure rather than throwing, so one bad file doesn't kill
    % the whole batch)
    good = ~cellfun(@isempty, rows);
    if any(~good)
        warning('parseNURingTrials:skippedFiles', ...
            '%d file(s) could not be parsed and were skipped.', sum(~good));
    end
    rows = rows(good);

    if isempty(rows)
        headers = emptyHeadersTable();
        trials  = emptyTrialsTable();
        summary = emptySummaryTable();
        userMeans   = emptyUserStatTable('avg');
        userMedians = emptyUserStatTable('med');
        return;
    end

    % ---------------------------------------------------------------------
    % Assemble TRIALS table
    % ---------------------------------------------------------------------
    trial_key      = string(cellfun(@(r) r.trial_key, rows, 'UniformOutput', false));
    participant_id = string(cellfun(@(r) r.participant_id, rows, 'UniformOutput', false));
    target_marker  = string(cellfun(@(r) r.target_marker,  rows, 'UniformOutput', false));
    trial_datetime = cellfun(@(r) r.trial_datetime, rows); % datetime scalars -> datetime array
    filename       = string(cellfun(@(r) r.filename, rows, 'UniformOutput', false));
    filepath       = string(cellfun(@(r) r.filepath, rows, 'UniformOutput', false));
    n_samples      = cellfun(@(r) r.n_samples, rows);
    duration_s     = cellfun(@(r) r.duration_s, rows);
    data           = cellfun(@(r) r.data, rows, 'UniformOutput', false); % cell column of timetables

    trials = table(trial_key, participant_id, target_marker, trial_datetime, ...
        filename, filepath, n_samples, duration_s, data);

    % Sort for convenience: by participant, then time of trial
    trials = sortrows(trials, {'participant_id', 'trial_datetime'});

    % ---------------------------------------------------------------------
    % Assemble HEADERS table
    % ---------------------------------------------------------------------
    h_trial_key   = string(cellfun(@(r) r.trial_key, rows, 'UniformOutput', false));
    user_id       = string(cellfun(@(r) r.header.user_id, rows, 'UniformOutput', false));
    target_id     = string(cellfun(@(r) r.header.target_id, rows, 'UniformOutput', false));
    target_screen_position_x_mm = cellfun(@(r) r.header.target_screen_position_x_mm, rows);
    target_screen_position_y_mm = cellfun(@(r) r.header.target_screen_position_y_mm, rows);
    fingertip_offset_x_mm       = cellfun(@(r) r.header.fingertip_offset_x_mm, rows);
    fingertip_offset_y_mm       = cellfun(@(r) r.header.fingertip_offset_y_mm, rows);
    fingertip_offset_z_mm       = cellfun(@(r) r.header.fingertip_offset_z_mm, rows);
    completion_time_s           = cellfun(@(r) r.header.completion_time_s, rows);
    endpoint_error_x_mm         = cellfun(@(r) r.header.endpoint_error_x_mm, rows);
    endpoint_error_y_mm         = cellfun(@(r) r.header.endpoint_error_y_mm, rows);
    endpoint_error_z_mm         = cellfun(@(r) r.header.endpoint_error_z_mm, rows);
    calibration_angle_count     = cellfun(@(r) r.header.calibration_angle_count, rows);
    calibration_angles_deg      = cellfun(@(r) r.header.calibration_angles_deg, rows, 'UniformOutput', false);
    control_points_theta        = cellfun(@(r) r.header.control_points_theta, rows, 'UniformOutput', false);
    control_points_radius       = cellfun(@(r) r.header.control_points_radius, rows, 'UniformOutput', false);
    control_points_accel        = cellfun(@(r) r.header.control_points_accel, rows, 'UniformOutput', false);
    stiffness_measurements      = cellfun(@(r) r.header.stiffness_measurements, rows, 'UniformOutput', false);
    h_filename    = string(cellfun(@(r) r.filename, rows, 'UniformOutput', false));
    h_filepath    = string(cellfun(@(r) r.filepath, rows, 'UniformOutput', false));

    headers = table(h_trial_key, user_id, target_id, ...
        target_screen_position_x_mm, target_screen_position_y_mm, ...
        fingertip_offset_x_mm, fingertip_offset_y_mm, fingertip_offset_z_mm, ...
        completion_time_s, ...
        endpoint_error_x_mm, endpoint_error_y_mm, endpoint_error_z_mm,...
        calibration_angle_count, calibration_angles_deg, control_points_theta, ...
        control_points_radius, control_points_accel, stiffness_measurements, ...
        h_filename, h_filepath);
    headers.Properties.VariableNames{1} = 'trial_key';
    headers.Properties.VariableNames{end-1} = 'filename';
    headers.Properties.VariableNames{end} = 'filepath';

    headers = sortrows(headers, {'user_id', 'target_id'});

    % % Add two summary rows (mean, then median) after each participant's
    % % block of trials, covering completion_time_s, endpoint_error_x_mm,
    % % and endpoint_error_y_mm.
    % headers = appendParticipantSummaryRows(headers);

    fprintf('TRIAL SUMMARIES:\n\n');

    % ---------------------------------------------------------------------
    % Build per-participant summary table: User | Data{target_id, ...}
    % ---------------------------------------------------------------------
    summary = buildSummaryTable(headers, trials);
    for i = 1:height(summary)
        fprintf('  User %s:\n', summary.User(i));
        disp(summary.Data{i});
    end

    % ---------------------------------------------------------------------
    % Build and print across-trial per-user mean/median tables
    % ---------------------------------------------------------------------
    userMeans   = buildUserStatTable(summary, @(x) mean(x, 'omitnan'),   'avg');
    userMedians = buildUserStatTable(summary, @(x) median(x, 'omitnan'), 'med');

    fprintf('  User Means:\n');
    disp(userMeans);
    fprintf('  User Medians:\n');
    disp(userMedians);

    disp ("ParseNURingTrials: Data parsed.") ;

end

% =====================================================================
% Local helper functions
% =====================================================================

function row = parseOneTrialFile(fpath)
    %PARSEONETRIALFILE Parse a single trial CSV file. Returns [] on failure.
    %   File is expected to contain a {HEADER} section followed by a
    %   {DATA} section (CSV with its own column-header row).
    row = [];
    try
        [~, name, ~] = fileparts(fpath);
        meta = parseTrialFilename(name);

        % -----------------------------------------------------------
        % Split the file into HEADER lines and DATA (CSV) lines
        % -----------------------------------------------------------
        rawText = fileread(fpath);
        allLines = regexp(rawText, '\r\n|\r|\n', 'split');

        headerMarkerIdx = find(strcmp(strtrim(allLines), '{HEADER}'), 1, 'first');
        dataMarkerIdx   = find(strcmp(strtrim(allLines), '{DATA}'), 1, 'first');

        if isempty(dataMarkerIdx)
            error('No {DATA} marker found in %s', fpath);
        end
        if isempty(headerMarkerIdx)
            headerMarkerIdx = 0; % no {HEADER} marker; treat everything before {DATA} as header lines
        end

        headerLines = allLines((headerMarkerIdx+1):(dataMarkerIdx-1));
        dataLines   = allLines((dataMarkerIdx+1):end);
        dataLines   = dataLines(~cellfun(@(s) isempty(strtrim(s)), dataLines)); % drop trailing blank lines

        header = parseTrialHeader(headerLines);

        % Sanity check: header user_id/target_id vs filename-parsed values
        if header.user_id ~= "" && header.user_id ~= string(meta.participant_id)
            warning('parseNURingTrials:headerFilenameMismatch', ...
                'Header user_id (%s) does not match filename participant_id (%s) in %s', ...
                header.user_id, meta.participant_id, fpath);
        end
        if header.target_id ~= "" && header.target_id ~= string(meta.target_marker)
            warning('parseNURingTrials:headerFilenameMismatch', ...
                'Header target_id (%s) does not match filename target_marker (%s) in %s', ...
                header.target_id, meta.target_marker, fpath);
        end

        % -----------------------------------------------------------
        % Write the DATA section to a temp CSV file so the existing
        % detectImportOptions/readtable pipeline can be reused unchanged.
        % -----------------------------------------------------------
        tmpFile = [tempname(), '.csv'];
        fid = fopen(tmpFile, 'w');
        if fid == -1
            error('Could not create temp file for parsing %s', fpath);
        end
        fprintf(fid, '%s\n', dataLines{:});
        fclose(fid);

        cleanupObj = onCleanup(@() deleteIfExists(tmpFile));

        opts = detectImportOptions(tmpFile, 'Delimiter', ',');
        opts = setvartype(opts, {'t_secs',...
            'tx_mm','ty_mm','tz_mm', ...
            'dx_mm','dy_mm','dz_mm',...
            'qx','qy','qz','qw',...
            'pwm_a','pwm_b','pwm_c', ...
            'virtual_x_mm','virtual_y_mm','touch_x_mm','touch_y_mm'}, 'double');
        opts = setvartype(opts, {'target_id','detected'}, 'int16');

        % Treat blanks in numeric columns as NaN (default behavior for
        % double columns in detectImportOptions, but set explicitly).
        touchVars = {'virtual_x_mm','virtual_y_mm','touch_x_mm','touch_y_mm'};
        for v = 1:numel(touchVars)
            if any(strcmp(opts.VariableNames, touchVars{v}))
                opts = setvaropts(opts, touchVars{v}, 'TreatAsMissing', '');
            end
        end

        T = readtable(tmpFile, opts);

        if ~ismember('t_secs', T.Properties.VariableNames)
            error('Missing required column t_secs in %s', fpath);
        end

        tt = table2timetable(T, 'RowTimes', seconds(T.t_secs));
        tt.t_secs = T.t_secs; % keep original column too, for convenience

        % Correct Y-axis flip: dy_mm/ty_mm come in with a mirrored sign
        % convention relative to dx_mm/dz_mm (moving up in the real world
        % otherwise plots as moving down). Negate here, once, so every
        % downstream consumer (plots, analysis) sees the corrected sign.
        if ismember('dy_mm', tt.Properties.VariableNames)
            tt.dy_mm = -tt.dy_mm; % Original
            tt.dx_mm = -tt.dx_mm;
        end
        if ismember('ty_mm', tt.Properties.VariableNames)
            tt.ty_mm = -tt.ty_mm; % Original
            tt.tx_mm = -tt.tx_mm;
        end

        trialKey = sprintf('u%s_t%s', meta.participant_id, meta.target_marker);

        row = struct();
        row.trial_key        = trialKey;
        row.participant_id   = meta.participant_id;
        row.target_marker    = meta.target_marker;
        row.trial_datetime   = meta.trial_datetime;
        row.filename         = [name, '.csv'];
        row.filepath         = fpath;
        row.n_samples        = height(tt);
        if height(tt) >= 2
            row.duration_s = T.t_secs(end) - T.t_secs(1);
        else
            row.duration_s = NaN;
        end
        row.data   = tt;
        row.header = header;
    catch ME
        warning('parseNURingTrials:fileParseFailed', ...
            'Failed to parse %s: %s', fpath, ME.message);
        row = [];
    end

end

function deleteIfExists(fpath)
    if exist(fpath, 'file')
        delete(fpath);
    end
end

function header = parseTrialHeader(headerLines)
    %PARSETRIALHEADER Parse "key: value" lines from a {HEADER} block.
    %   Recognizes scalar fields (user_id, target_id) and the known
    %   bracketed-array fields, which get split into named components:
    %       target_screen_position_mm: [x, y]
    %           -> target_screen_position_x_mm, target_screen_position_y_mm
    %       fingertip_offset: [x, y, z]
    %           -> fingertip_offset_x_mm, fingertip_offset_y_mm, fingertip_offset_z_mm
    %       endpoint_error_mm: [x, y, z]
    %           -> endpoint_error_x_mm, endpoint_error_y_mm, endpoint_error_z_mm
    %   plus the scalar field completion_time -> completion_time_s, and the
    %   newer calibration/control-point/stiffness fields, which are kept as
    %   whole numeric vectors (length isn't fixed, so they aren't split
    %   into named x/y/z-style columns):
    %       calibration_angle_count   -> calibration_angle_count (scalar)
    %       calibration_angles_deg    -> calibration_angles_deg  (vector)
    %       control_points_theta      -> control_points_theta    (vector)
    %       control_points_radius     -> control_points_radius   (vector)
    %       control_points_accel      -> control_points_accel    (vector)
    %       stiffness_measurements    -> stiffness_measurements  (vector)

    header = struct( ...
        'user_id', "", ...
        'target_id', "", ...
        'target_screen_position_x_mm', NaN, ...
        'target_screen_position_y_mm', NaN, ...
        'fingertip_offset_x_mm', NaN, ...
        'fingertip_offset_y_mm', NaN, ...
        'fingertip_offset_z_mm', NaN, ...
        'completion_time_s',     NaN, ...
        'endpoint_error_x_mm',   NaN, ...
        'endpoint_error_y_mm',   NaN, ...
        'endpoint_error_z_mm',   NaN, ...
        'calibration_angle_count', NaN, ...
        'calibration_angles_deg',  [], ...
        'control_points_theta',    [], ...
        'control_points_radius',   [], ...
        'control_points_accel',    [], ...
        'stiffness_measurements',  []);

    for i = 1:numel(headerLines)
        line = strtrim(headerLines{i});
        if isempty(line)
            continue;
        end
        colonIdx = strfind(line, ':');
        if isempty(colonIdx)
            continue;
        end
        key = strtrim(line(1:colonIdx(1)-1));
        val = strtrim(line(colonIdx(1)+1:end));

        switch key
            case 'user_id'
                header.user_id = string(val);
            case 'target_id'
                header.target_id = string(val);
            case 'target_screen_position_mm'
                vals = parseBracketedNumericList(val);
                if numel(vals) >= 2
                    % Negated: the screen-position coordinate frame is
                    % mirrored on both axes relative to dx_mm/dy_mm's
                    % frame, which otherwise shows up as everything drawn
                    % on the Z=0 screen plane (target marker) being
                    % flipped in X and Y relative to the trajectory.
                    header.target_screen_position_x_mm = vals(1);
                    header.target_screen_position_y_mm = -vals(2);
                end
            case 'fingertip_offset'
                vals = parseBracketedNumericList(val);
                if numel(vals) >= 3
                    header.fingertip_offset_x_mm = vals(1);
                    header.fingertip_offset_y_mm = -vals(2);
                    header.fingertip_offset_z_mm = vals(3);
                end
            case 'completion_time'
                if iscell(val)
                    val = val{1};
                end
                header.completion_time_s = str2double(val);
            case 'endpoint_error_mm'
                vals = parseBracketedNumericList(val);
                if numel(vals) >= 3
                    header.endpoint_error_x_mm =  vals(1);
                    header.endpoint_error_y_mm = -vals(2);
                    header.endpoint_error_z_mm =  vals(3);
                end
            case 'calibration_angle_count'
                if iscell(val)
                    val = val{1};
                end
                header.calibration_angle_count = str2double(val);
            case 'calibration_angles_deg'
                header.calibration_angles_deg = parseBracketedNumericList(val);
            case 'control_points_theta'
                header.control_points_theta = parseBracketedNumericList(val);
            case 'control_points_radius'
                header.control_points_radius = parseBracketedNumericList(val);
            case 'control_points_accel'
                header.control_points_accel = parseBracketedNumericList(val);
            case 'stiffness_measurements'
                header.stiffness_measurements = parseBracketedNumericList(val);
            otherwise
                % Unknown header field: ignore (forward-compatible with
                % future header fields that this function doesn't yet know
                % how to split).
        end
    end
end

function vals = parseBracketedNumericList(str)
    %PARSEBRACKETEDNUMERICLIST Parse "[1.22, 33.85, 20.50]" -> [1.22 33.85 20.50]
    str = strtrim(str);
    str = regexprep(str, '^\[|\]$', ''); % strip leading/trailing brackets
    parts = strsplit(str, ',');
    vals = str2double(strtrim(parts));
end
        
function meta = parseTrialFilename(name)
    %PARSETRIALFILENAME Parse "PID-TARGET-MMddyyyy-HHmmss" filename (no ext),
    %   e.g. "124-286-06252026-184333".
    tok = regexp(name, '^(?<pid>\d+)-(?<target>\d+)-(?<date>\d{8})-(?<time>\d{6})$', 'names');
    if isempty(tok)
        error('Filename "%s" does not match expected PID-TARGET-MMddyyyy-HHmmss pattern', name);
    end
    meta.participant_id = tok.pid;
    meta.target_marker  = tok.target;
    try
        meta.trial_datetime = datetime([tok.date, tok.time], 'InputFormat', 'MMddyyyyHHmmss');
    catch
        meta.trial_datetime = NaT;
    end
end

function tf = canUseParallel()
    %CANUSEPARALLEL True if Parallel Computing Toolbox is licensed/available
    %and either a pool already exists or one can be started.
    tf = false;
    try
        if ~license('test', 'Distrib_Computing_Toolbox')
            return;
        end
        pool = gcp('nocreate');
        if isempty(pool)
            pool = parpool; %#ok<NASGU> attempt to start default pool
        end
        tf = true;
    catch
        tf = false;
    end
end

function summary = buildSummaryTable(headers, trials)
    %BUILDSUMMARYTABLE Build a per-user summary: one row per participant,
    %   with User (string) and Data (cell containing a per-trial table):
    %       target_id, start_euclidean_mm, completion_time_s,
    %       endpoint_error_x_mm, endpoint_error_y_mm, endpoint_error_z_mm,
    %       endpoint_error_2D_mm, endpoint_error_3D_mm
    %   start_euclidean_mm is computed from the first sample (t_secs(1)) of
    %   each trial's data: sqrt(dx_mm^2 + dy_mm^2 + dz_mm^2).
    %   Synthetic MEAN/MEDIAN/blank rows from appendParticipantSummaryRows
    %   are excluded -- this summary is built from real trials only.

    realHeaders = headers(~ismember(headers.target_id, ["MEAN", "MEDIAN", ""]), :);

    uniqueUsers = unique(realHeaders.user_id, 'stable');
    nUsers = numel(uniqueUsers);

    User = uniqueUsers;
    Data = cell(nUsers, 1);

    for i = 1:nUsers
        u = uniqueUsers(i);
        sub = realHeaders(realHeaders.user_id == u, :);
        sub = sortrows(sub, 'target_id');

        nTrials = height(sub);
        start_euclidean_mm = nan(nTrials, 1);
        for j = 1:nTrials
            trialRow = trials(trials.trial_key == sub.trial_key(j), :);
            if isempty(trialRow)
                continue;
            end
            tt = trialRow.data{1};
            if height(tt) == 0
                continue;
            end
            start_euclidean_mm(j) = sqrt(tt.dx_mm(1)^2 + tt.dy_mm(1)^2 + tt.dz_mm(1)^2);
        end

        target_id            = sub.target_id;
        completion_time_s    = sub.completion_time_s;
        endpoint_error_x_mm   = sub.endpoint_error_x_mm;
        endpoint_error_y_mm   = sub.endpoint_error_y_mm;
        endpoint_error_z_mm   = sub.endpoint_error_z_mm;
        endpoint_error_2D_mm  = sqrt(endpoint_error_x_mm.^2 + endpoint_error_y_mm.^2);
        endpoint_error_3D_mm  = sqrt(endpoint_error_x_mm.^2 + endpoint_error_y_mm.^2 + endpoint_error_z_mm.^2);

        Data{i} = table(target_id, start_euclidean_mm, completion_time_s, ...
            endpoint_error_x_mm, endpoint_error_y_mm, endpoint_error_z_mm, ...
            endpoint_error_2D_mm, endpoint_error_3D_mm);
    end

    summary = table(User, Data);
end

function T = buildUserStatTable(summary, statFun, statName)
    %BUILDUSERSTATTABLE Collapse each user's per-trial table in
    %   SUMMARY.Data down to a single row via STATFUN (e.g. mean/median,
    %   NaNs omitted), giving one row per user:
    %       userID, <statName>_completion_time, <statName>_endpoint_error_x,
    %       <statName>_endpoint_error_y, <statName>_endpoint_error_z,
    %       <statName>_endpoint_error_2D, <statName>_endpoint_error_3D
    %   STATNAME (e.g. "mean" or "median") is used both to label the
    %   columns and is purely cosmetic -- STATFUN does the actual math.

    nUsers = height(summary);
    userID = summary.User;

    completion_time = nan(nUsers, 1);
    err_x  = nan(nUsers, 1);
    err_y  = nan(nUsers, 1);
    err_z  = nan(nUsers, 1);
    err_2D = nan(nUsers, 1);
    err_3D = nan(nUsers, 1);

    for i = 1:nUsers
        d = summary.Data{i};
        completion_time(i) = statFun(d.completion_time_s);
        err_x(i)  = statFun(d.endpoint_error_x_mm);
        err_y(i)  = statFun(d.endpoint_error_y_mm);
        err_z(i)  = statFun(d.endpoint_error_z_mm);
        err_2D(i) = statFun(d.endpoint_error_2D_mm);
        err_3D(i) = statFun(d.endpoint_error_3D_mm);
    end

    T = table(userID, completion_time, err_x, err_y, err_z, err_2D, err_3D);
    T.Properties.VariableNames = {'userID', ...
        [statName, '_completion_time'], [statName, '_endpoint_error_x'], ...
        [statName, '_endpoint_error_y'], [statName, '_endpoint_error_z'], ...
        [statName, '_endpoint_error_2D'], [statName, '_endpoint_error_3D']};

    % Append an "ALL" row: pool every trial across every participant
    % (not an average-of-averages) and apply the same stat function.
    if nUsers > 0
        allData = vertcat(summary.Data{:});
        allRow = table("ALL", statFun(allData.completion_time_s), ...
            statFun(allData.endpoint_error_x_mm), statFun(allData.endpoint_error_y_mm), ...
            statFun(allData.endpoint_error_z_mm), statFun(allData.endpoint_error_2D_mm), ...
            statFun(allData.endpoint_error_3D_mm));
        allRow.Properties.VariableNames = T.Properties.VariableNames;
        T = [T; allRow];
    end

end

function row = makeSummaryRow(templateRow, userId, label, completionTime, errX, errY)
    %MAKESUMMARYROW Build a single synthetic headers-table row (same
    %   schema as templateRow) holding an aggregate (mean/median) value for
    %   completion_time_s/endpoint_error_x_mm/endpoint_error_y_mm, with
    %   every other field left NaN/blank and target_id/trial_key marked
    %   with LABEL ("MEAN" or "MEDIAN") so the row is identifiable.
    row = templateRow;
    row.trial_key   = sprintf('u%s_%s', userId, label);
    row.user_id     = userId;
    row.target_id   = string(label);
    row.target_screen_position_x_mm = NaN;
    row.target_screen_position_y_mm = NaN;
    row.fingertip_offset_x_mm       = NaN;
    row.fingertip_offset_y_mm       = NaN;
    row.fingertip_offset_z_mm       = NaN;
    row.completion_time_s   = completionTime;
    row.endpoint_error_x_mm = errX;
    row.endpoint_error_y_mm = errY;
    row.endpoint_error_z_mm = NaN;
    row.calibration_angle_count = NaN;
    row.calibration_angles_deg  = {[]};
    row.control_points_theta    = {[]};
    row.control_points_radius   = {[]};
    row.control_points_accel    = {[]};
    row.stiffness_measurements  = {[]};
    row.filename = "";
    row.filepath = "";
end

function T = emptyTrialsTable()
    %EMPTYTRIALSTABLE Return a correctly-typed empty trials table.
    trial_key      = string([]);
    participant_id = string([]);
    target_marker  = string([]);
    trial_datetime = datetime([]);
    filename       = string([]);
    filepath       = string([]);
    n_samples      = [];
    duration_s     = [];
    data           = {};
    T = table(trial_key, participant_id, target_marker, trial_datetime, ...
        filename, filepath, n_samples, duration_s, data);
end

function T = emptyHeadersTable()
    %EMPTYHEADERSTABLE Return a correctly-typed empty headers table.
    trial_key                    = string([]);
    user_id                      = string([]);
    target_id                    = string([]);
    target_screen_position_x_mm = [];
    target_screen_position_y_mm = [];
    fingertip_offset_x_mm       = [];
    fingertip_offset_y_mm       = [];
    fingertip_offset_z_mm       = [];
    completion_time_s           = [];
    endpoint_error_x_mm         = []; 
    endpoint_error_y_mm         = []; 
    endpoint_error_z_mm         = []; 
    calibration_angle_count     = [];
    calibration_angles_deg      = {};
    control_points_theta        = {};
    control_points_radius       = {};
    control_points_accel        = {};
    stiffness_measurements      = {};
    filename                    = string([]);
    filepath                    = string([]);
    T = table(trial_key, user_id, target_id, ...
        target_screen_position_x_mm, target_screen_position_y_mm, ...
        fingertip_offset_x_mm, fingertip_offset_y_mm, fingertip_offset_z_mm, ...
        completion_time_s, endpoint_error_x_mm, endpoint_error_y_mm, endpoint_error_z_mm,...
        calibration_angle_count, calibration_angles_deg, control_points_theta, ...
        control_points_radius, control_points_accel, stiffness_measurements, ...
        filename, filepath);
end

function T = emptySummaryTable()
    %EMPTYSUMMARYTABLE Return a correctly-typed empty summary table.
    User = string([]);
    Data = {};
    T = table(User, Data);
end

function T = emptyUserStatTable(statName)
    %EMPTYUSERSTATTABLE Return a correctly-typed empty userMeans/userMedians
    %   table (column names labeled with STATNAME, e.g. "mean"/"median").
    userID = string([]);
    completion_time = [];
    err_x  = [];
    err_y  = [];
    err_z  = [];
    err_2D = [];
    err_3D = [];
    T = table(userID, completion_time, err_x, err_y, err_z, err_2D, err_3D);
    T.Properties.VariableNames = {'userID', ...
        [statName, '_completion_time'], [statName, '_endpoint_error_x'], ...
        [statName, '_endpoint_error_y'], [statName, '_endpoint_error_z'], ...
        [statName, '_endpoint_error_2D'], [statName, '_endpoint_error_3D']};
end