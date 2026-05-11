% Run every experiment batch used by the comparison plotters.
%
% The plotters in analysis/comparison load result .mat files directly from
% the experiment folders. Existing matching results are archived before each
% batch so old runs are not mixed with the new run set.

clear; clc;

root_dir = fileparts(mfilename('fullpath'));

n_runs = 5;
scenario_suffixes = {'', '_nd', '_mm', '_mm_nd'};
archive_existing_results = true;
rng_seeds = 1:10;
parallel_pool_size = 5;

batches = struct('label', {}, ...
                 'script', {}, ...
                 'folders', {}, ...
                 'result_pattern', {}, ...
                 'archive_prefix', {});

baseline_folders = scenarioFolders('DDPGpi_main_s5_1', scenario_suffixes);

batches(end+1) = makeBatch( ...
    'No Control', ...
    'run_10_experiments_no_control.m', ...
    baseline_folders, ...
    'No_control_*.mat', ...
    'archived_no_control_results');

batches(end+1) = makeBatch( ...
    'Hierarchical MPC', ...
    'run_10_experiments_MPC.m', ...
    baseline_folders, ...
    'MPC_RM_SR_hier*result_*.mat', ...
    'archived_hier_MPC_results');

batches(end+1) = makeBatch( ...
    'PI-ALINEA MPC', ...
    'run_10_experiments.m', ...
    piAlineaFolders(n_runs, scenario_suffixes), ...
    'PI_ALINEA_MPC_SR_RM*result_*.mat', ...
    'archived_PI_ALINEA_results');

batches(end+1) = makeBatch( ...
    'DDPG-MPC', ...
    'run_10_experiments.m', ...
    agentFolders('DDPGpi_main_s5_', n_runs, scenario_suffixes), ...
    'RL_MPC_SR_RM_result_*.mat', ...
    'archived_RL_MPC_results');

batches(end+1) = makeBatch( ...
    'SAC-MPC', ...
    'run_10_experiments.m', ...
    agentFolders('SACpi_main_s5_', n_runs, scenario_suffixes), ...
    'RL_MPC_SR_RM_result_*.mat', ...
    'archived_RL_MPC_results');

batches(end+1) = makeBatch( ...
    'SAC(D)-MPC', ...
    'run_10_experiments.m', ...
    agentFolders('SACDpi_main_s5_', n_runs, scenario_suffixes), ...
    'RL_MPC_SR_RM_result_*.mat', ...
    'archived_RL_MPC_results');

start_dir = pwd;
cleanup_obj = onCleanup(@() cd(start_dir));

fprintf('Starting all experiment batches from %s\n', root_dir);
fprintf('Using RNG seeds %s for every experiment folder.\n', mat2str(rng_seeds));
prepareParallelPool(parallel_pool_size);

for b = 1:numel(batches)
    batch = batches(b);
    fprintf('\n=== %s: %d folder(s) ===\n', batch.label, numel(batch.folders));

    for f = 1:numel(batch.folders)
        exp_dir = batch.folders{f};
        exp_path = fullfile(root_dir, exp_dir);

        if ~exist(exp_path, 'dir')
            error('Experiment folder not found: %s', exp_path);
        end

        script_path = fullfile(exp_path, batch.script);
        if ~exist(script_path, 'file')
            error('Experiment script not found: %s', script_path);
        end

        fprintf('\nRunning %s in %s...\n', batch.label, exp_dir);

        if archive_existing_results
            archiveResults(exp_path, batch.result_pattern, batch.archive_prefix);
        end

        runExperimentScript(exp_path, batch.script, rng_seeds);
        cd(root_dir);
    end
end

fprintf('\nAll experiment batches completed.\n');

function batch = makeBatch(label, script, folders, result_pattern, archive_prefix)
    batch = struct('label', label, ...
                   'script', script, ...
                   'folders', {folders}, ...
                   'result_pattern', result_pattern, ...
                   'archive_prefix', archive_prefix);
end

function folders = scenarioFolders(base_name, scenario_suffixes)
    folders = cell(numel(scenario_suffixes), 1);
    for s = 1:numel(scenario_suffixes)
        folders{s} = [base_name scenario_suffixes{s}];
    end
end

function folders = agentFolders(prefix, n_runs, scenario_suffixes)
    folders = cell(n_runs * numel(scenario_suffixes), 1);
    idx = 1;
    for s = 1:numel(scenario_suffixes)
        sfx = scenario_suffixes{s};
        for r = 1:n_runs
            folders{idx} = [prefix num2str(r) sfx];
            idx = idx + 1;
        end
    end
end

function folders = piAlineaFolders(n_runs, scenario_suffixes)
    folders = cell(n_runs * numel(scenario_suffixes), 1);
    idx = 1;
    for s = 1:numel(scenario_suffixes)
        sfx = scenario_suffixes{s};
        for r = 1:n_runs
            folders{idx} = ['PI_ALINEA_MPC' sfx '_' num2str(r)];
            idx = idx + 1;
        end
    end
end

function archiveResults(exp_path, result_pattern, archive_prefix)
    old_results = dir(fullfile(exp_path, result_pattern));
    if isempty(old_results)
        fprintf('  No existing result files matching %s\n', result_pattern);
        return;
    end

    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    archive_dir = fullfile(exp_path, [archive_prefix '_' timestamp]);
    suffix = 1;
    while exist(archive_dir, 'dir')
        suffix = suffix + 1;
        archive_dir = fullfile(exp_path, ...
            [archive_prefix '_' timestamp '_' num2str(suffix)]);
    end

    mkdir(archive_dir);
    for f = 1:numel(old_results)
        movefile(fullfile(exp_path, old_results(f).name), ...
                 fullfile(archive_dir, old_results(f).name));
    end

    fprintf('  Archived %d existing result file(s) to %s\n', ...
            numel(old_results), archive_dir);
end

function prepareParallelPool(pool_size)
    if exist('gcp', 'file') == 0 && exist('gcp', 'builtin') == 0
        error('Parallel Computing Toolbox is required to start a parallel pool.');
    end

    pool = gcp('nocreate');
    if isempty(pool)
        fprintf('Starting parallel pool with %d workers...\n', pool_size);
        parpool('local', pool_size);
        warmUpParallelPool(pool_size);
        return;
    end

    if pool.NumWorkers == pool_size
        fprintf('Using existing parallel pool with %d workers.\n', pool.NumWorkers);
        warmUpParallelPool(pool_size);
        return;
    end

    fprintf(['Restarting parallel pool with %d workers ', ...
             '(current pool has %d workers)...\n'], pool_size, pool.NumWorkers);
    delete(pool);
    parpool('local', pool_size);
    warmUpParallelPool(pool_size);
end

function warmUpParallelPool(pool_size)
    fprintf('Warming up parallel pool dispatch...\n');
    warmup_values = zeros(pool_size, 1);
    parfor worker_idx = 1:pool_size
        warmup_values(worker_idx) = worker_idx;
    end
end

function runExperimentScript(exp_path, runner_script_name, rng_seeds)
    previous_dir = pwd;
    cleanup_obj = onCleanup(@() cd(previous_dir));
    cd(exp_path);

    runner_script_path = fullfile(exp_path, runner_script_name);
    [experiment_script, diary_name] = parseExperimentRunner(runner_script_path);

    if ~exist(fullfile(exp_path, experiment_script), 'file')
        error('Benchmark script not found: %s', ...
              fullfile(exp_path, experiment_script));
    end

    fprintf('  Benchmark script: %s\n', experiment_script);
    fprintf('  Diary file: %s\n', diary_name);
    fprintf('  RNG seeds: %s\n', mat2str(rng_seeds));

    diary(diary_name);
    diary_cleanup = onCleanup(@() diary('off'));

    for seed_idx = 1:numel(rng_seeds)
        i = rng_seeds(seed_idx);
        fprintf('  Seed %d of %d: rng(%d)\n', ...
                seed_idx, numel(rng_seeds), i);
        rng(i);
        run(experiment_script);
        pause(1);
    end
end

function [experiment_script, diary_name] = parseExperimentRunner(runner_script_path)
    runner_text = fileread(runner_script_path);
    runner_lines = regexp(runner_text, '\r\n|\n|\r', 'split');
    diary_name = '';
    experiment_scripts = {};

    for line_idx = 1:numel(runner_lines)
        line = strtrim(runner_lines{line_idx});
        if isempty(line) || line(1) == '%'
            continue;
        end

        comment_idx = strfind(line, '%');
        if ~isempty(comment_idx)
            line = strtrim(line(1:comment_idx(1) - 1));
            if isempty(line)
                continue;
            end
        end

        diary_tokens = regexp(line, '^diary\s+(.+?)\s*;?$', ...
                              'tokens', 'once');
        if ~isempty(diary_tokens)
            candidate_diary_name = stripQuotes(strtrim(diary_tokens{1}));
            if ~strcmp(candidate_diary_name, 'off')
                diary_name = candidate_diary_name;
            end
            continue;
        end

        run_tokens = regexp(line, '^run\s+(.+?\.m)\s*;?$', ...
                            'tokens', 'once');
        if ~isempty(run_tokens)
            experiment_scripts{end+1} = stripQuotes(strtrim(run_tokens{1})); %#ok<AGROW>
        end
    end

    if isempty(diary_name)
        error('No diary file found in experiment runner: %s', runner_script_path);
    end

    if numel(experiment_scripts) ~= 1
        error(['Expected exactly one active benchmark script in %s, ', ...
               'but found %d.'], runner_script_path, numel(experiment_scripts));
    end

    experiment_script = experiment_scripts{1};
end

function value = stripQuotes(value)
    if numel(value) < 2
        return;
    end

    is_single_quoted = value(1) == '''' && value(end) == '''';
    is_double_quoted = value(1) == '"' && value(end) == '"';
    if is_single_quoted || is_double_quoted
        value = value(2:end-1);
    end
end
