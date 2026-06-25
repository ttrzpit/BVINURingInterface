function [trials, summary] = ParseNURingTrials(rootDir, varargin)
    %PARSENURINGTRIALS Parse NURing trial CSV files into a MATLAB table.
    %
    %   trials = PARSENURINGTRIALS(rootDir) scans rootDir (the +TrialData
    %   folder) for participant subfolders, each containing trial CSV files
    %   named like:
    %       123-06192026-200806-303.csv
    %       ^pid ^date(MMddyyyy) ^time(HHmmss) ^targetMarker
    %
    %   and returns a table TRIALS with one row per trial file:
    %
    %       participant_id   (string)
    %       target_marker    (string)   - target marker parsed from filename
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
    %       target_id, detected, tx_mm, ty_mm, tz_mm, qx, qy, qz, qw,
    %       pwm_a, pwm_b, pwm_c, touch_x_px, touch_y_px, touch_x_mm, touch_y_mm
    %   Empty touch_* fields are read as NaN.
    %
    %   [trials, summary] = PARSENURINGTRIALS(rootDir) also returns SUMMARY,
    %   a table with one row per participant:
    %       User    (string) - participant_id
    %       Qty     (double) - number of trials parsed for that participant
    %       Trials  (string) - comma-separated list of that participant's
    %                           target markers, e.g. "032, 042, 123, 332"
    %   SUMMARY is also printed to the console automatically.
    %
    %   trials = PARSENURINGTRIALS(rootDir, 'Name', Value, ...) supports:
    %       'UseParallel'   (default true)   - use parfor over trial files if
    %                                           Parallel Computing Toolbox and
    %                                           a pool are available
    %       'Participants'  (default {})     - cell array of participant ID
    %                                           strings/folder names to
    %                                           restrict parsing to. Empty
    %                                           means "all participants".
    %
    %   Example:
    %       [trials, summary] = ParseNURingTrials('+TrialData');
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
        trials = emptyTrialsTable();
        summary = emptySummaryTable();
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
        trials = emptyTrialsTable();
        summary = emptySummaryTable();
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
        trials = emptyTrialsTable();
        summary = emptySummaryTable();
        return;
    end

    % ---------------------------------------------------------------------
    % Assemble into one table
    % ---------------------------------------------------------------------
    participant_id = string(cellfun(@(r) r.participant_id, rows, 'UniformOutput', false));
    target_marker  = string(cellfun(@(r) r.target_marker,  rows, 'UniformOutput', false));
    trial_datetime = cellfun(@(r) r.trial_datetime, rows); % datetime scalars -> datetime array
    filename       = string(cellfun(@(r) r.filename, rows, 'UniformOutput', false));
    filepath       = string(cellfun(@(r) r.filepath, rows, 'UniformOutput', false));
    n_samples      = cellfun(@(r) r.n_samples, rows);
    duration_s     = cellfun(@(r) r.duration_s, rows);
    data           = cellfun(@(r) r.data, rows, 'UniformOutput', false); % cell column of timetables

    trials = table(participant_id, target_marker, trial_datetime, ...
        filename, filepath, n_samples, duration_s, data);

    % Sort for convenience: by participant, then time of trial
    trials = sortrows(trials, {'participant_id', 'trial_datetime'});

    % ---------------------------------------------------------------------
    % Build per-participant summary table: User | Qty | Trials
    % ---------------------------------------------------------------------
    summary = buildSummaryTable(trials);
    disp(summary);

    disp ("ParseNURingTrials: Data parsed.") ;

end

% =====================================================================
% Local helper functions
% =====================================================================

function row = parseOneTrialFile(fpath)
    %PARSEONETRIALFILE Parse a single trial CSV file. Returns [] on failure.
    row = [];
    try
        [~, name, ~] = fileparts(fpath);
        meta = parseTrialFilename(name);

        opts = detectImportOptions(fpath, 'Delimiter', ',');
        opts = setvartype(opts, {'t_secs',...
            'tx_mm','ty_mm','tz_mm', ...
            'dx_mm','dy_mm','dz_mm',...
            'qx','qy','qz','qw',...
            'pwm_a','pwm_b','pwm_c', ...
            'touch_x_px','touch_y_px','touch_x_mm','touch_y_mm'}, 'double');
        opts = setvartype(opts, {'target_id','detected'}, 'int16');

        % Treat blanks in numeric columns as NaN (default behavior for
        % double columns in detectImportOptions, but set explicitly).
        touchVars = {'touch_x_px','touch_y_px','touch_x_mm','touch_y_mm'};
        for v = 1:numel(touchVars)
            if any(strcmp(opts.VariableNames, touchVars{v}))
                opts = setvaropts(opts, touchVars{v}, 'TreatAsMissing', '');
            end
        end

        T = readtable(fpath, opts);

        if ~ismember('t_secs', T.Properties.VariableNames)
            error('Missing required column t_secs in %s', fpath);
        end

        tt = table2timetable(T, 'RowTimes', seconds(T.t_secs));
        tt.t_secs = T.t_secs; % keep original column too, for convenience

        row = struct();
        row.participant_id  = meta.participant_id;
        row.target_marker   = meta.target_marker;
        row.trial_datetime  = meta.trial_datetime;
        row.filename         = [name, '.csv'];
        row.filepath          = fpath;
        row.n_samples         = height(tt);
        if height(tt) >= 2
            row.duration_s = T.t_secs(end) - T.t_secs(1);
        else
            row.duration_s = NaN;
        end
        row.data = tt;
    catch ME
        warning('parseNURingTrials:fileParseFailed', ...
            'Failed to parse %s: %s', fpath, ME.message);
        row = [];
    end

end

function meta = parseTrialFilename(name)
    %PARSETRIALFILENAME Parse "PID-MMddyyyy-HHmmss-TARGET" filename (no ext).
    tok = regexp(name, '^(?<pid>\d+)-(?<date>\d{8})-(?<time>\d{6})-(?<target>\d+)$', 'names');
    if isempty(tok)
        error('Filename "%s" does not match expected PID-MMddyyyy-HHmmss-TARGET pattern', name);
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

function summary = buildSummaryTable(trials)
    %BUILDSUMMARYTABLE Build a User | Qty | Trials summary from the trials
    %table, one row per unique participant_id.
    uniqueUsers = unique(trials.participant_id);
    nUsers = numel(uniqueUsers);

    User   = uniqueUsers;
    Qty    = zeros(nUsers, 1);
    Trials = strings(nUsers, 1);

    for i = 1:nUsers
        isThisUser = trials.participant_id == uniqueUsers(i);
        Qty(i) = sum(isThisUser);
        markers = sort(unique(trials.target_marker(isThisUser)));
        Trials(i) = strjoin(markers, ', ');
    end

    summary = table(User, Qty, Trials);
end

function T = emptyTrialsTable()
    %EMPTYTRIALSTABLE Return a correctly-typed empty trials table.
    participant_id = string([]);
    target_marker  = string([]);
    trial_datetime = datetime([]);
    filename       = string([]);
    filepath       = string([]);
    n_samples      = [];
    duration_s     = [];
    data           = {};
    T = table(participant_id, target_marker, trial_datetime, ...
        filename, filepath, n_samples, duration_s, data);
end

function T = emptySummaryTable()
    %EMPTYSUMMARYTABLE Return a correctly-typed empty summary table.
    User   = string([]);
    Qty    = [];
    Trials = string([]);
    T = table(User, Qty, Trials);
end