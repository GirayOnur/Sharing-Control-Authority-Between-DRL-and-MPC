%% test_sac_policy_mode_objective.m
%
% Runs one saved SAC-MPC agent twice on the same experiment seed:
%   1) UseExplorationPolicy = true   -> stochastic SAC actions
%   2) UseExplorationPolicy = false  -> deterministic/max-likelihood SAC actions
%
% The objective is decomposed with the same calculation used in
% comparison/multi_agents/plot_obj_multi.m.

clearvars -except agent_folder_name experiment_seed; clc; close all;

%% User settings
script_dir = fileparts(mfilename('fullpath'));
agents_dir = fullfile(script_dir, '..', '..', '..');
out_dir    = script_dir;

% Default: Scenario 3 (_mm), SAC run 3. Change this folder to test another
% SAC agent, for example 'SACpi_main_s5_3_mm_nd' for Scenario 4.
% You can also override this before calling run(...):
%   agent_folder_name = 'DDPGpi_main_s5_3_mm';
%   experiment_seed = 1;
%   run('comparison/multi_agents/test_sac_policy_mode_objective.m')
if ~exist('agent_folder_name', 'var')
    agent_folder_name = 'SACpi_main_s5_3_mm';
end
if ~exist('experiment_seed', 'var')
    experiment_seed = 1;
end

agent_folder = fullfile(agents_dir, agent_folder_name);
if ~exist(agent_folder, 'dir')
    error('Agent folder not found: %s', agent_folder);
end

%% Run stochastic vs deterministic SAC policy modes
fprintf('Testing policy modes for folder:\n  %s\n\n', agent_folder);

modes = [true, false];
mode_labels = {'stochastic / exploration=true', 'deterministic / exploration=false'};
results = struct([]);

for i = 1:numel(modes)
    fprintf('[%d/%d] Running %s with rng(%d)...\n', ...
        i, numel(modes), mode_labels{i}, experiment_seed);
    results(i).mode_label = mode_labels{i}; %#ok<SAGROW>
    results(i).use_exploration_policy = modes(i);
    results(i).metrics = run_one_sac_experiment(agent_folder, modes(i), experiment_seed);
end

%% Print summary
fprintf('\nObjective decomposition (same J definition as plot_obj_multi.m):\n');
fprintf('%-36s %14s %14s %14s %14s %12s\n', ...
    'Policy mode', 'J', 'TTS', 'q_pen', 'u_pen', 'comp_time');
fprintf('%s\n', repmat('-', 1, 112));
for i = 1:numel(results)
    m = results(i).metrics;
    fprintf('%-36s %14.2f %14.2f %14.2f %14.6f %12.2f\n', ...
        results(i).mode_label, m.J, m.tts, m.q_pen, m.u_pen, m.total_comp_time);
end

delta = results(2).metrics.J - results(1).metrics.J;
fprintf('\nDelta J (deterministic - stochastic): %.2f\n', delta);
fprintf('Negative delta means deterministic/exploration=false produced a lower objective.\n');

out_file = fullfile(out_dir, sprintf('policy_mode_test_%s_seed%d.mat', ...
    agent_folder_name, experiment_seed));
save(out_file, 'results', 'agent_folder_name', 'experiment_seed');
fprintf('\nSaved result data -> %s\n', out_file);

%% Local functions
function metrics = run_one_sac_experiment(agent_folder, use_exploration_policy, seed)
    original_dir = pwd;
    cleanup = onCleanup(@() restore_path_and_dir(original_dir, agent_folder));
    addpath(agent_folder);
    cd(agent_folder);

    rng(seed);

    [~, folder_name] = fileparts(agent_folder);
    is_nd = endsWith(folder_name, '_nd');
    is_mm = contains(folder_name, '_mm');

    scenario = 3;
    N = 900;

    param_sim = param_get;
    param_MPC_low = param_MPC_get(1);
    param_MPC_high = param_MPC_get(0);
    x_norm = calc_x_norm;

    if is_mm
        param_mm = param_get_mm;
    end

    if is_nd
        base_demands = load('base_demands.mat');
        demands.o1c1 = calc_noisy_demands('o1', 'c1', base_demands.base_demand_o1c1);
        demands.o1c2 = calc_noisy_demands('o1', 'c2', base_demands.base_demand_o1c2);
        demands.o2c1 = calc_noisy_demands('o2', 'c1', base_demands.base_demand_o2c1);
        demands.o2c2 = calc_noisy_demands('o2', 'c2', base_demands.base_demand_o2c2);
        demands.o3c1 = calc_noisy_demands('o3', 'c1', base_demands.base_demand_o3c1);
        demands.o3c2 = calc_noisy_demands('o3', 'c2', base_demands.base_demand_o3c2);
    end

    agent_files = dir(fullfile(agent_folder, 'agent_*.mat'));
    if isempty(agent_files)
        error('No agent_*.mat file found in %s', agent_folder);
    end
    [~, sort_idx] = sort({agent_files.name});
    agent_files = agent_files(sort_idx);

    loaded = load(fullfile(agent_folder, agent_files(1).name), 'agent');
    agent = loaded.agent;
    agent.UseExplorationPolicy = use_exploration_policy;

    x = zeros(75, 1);
    xx = zeros(numel(x), N);
    u = [0.5; 1; 1];
    uu = zeros(numel(u), N);
    total_comp_time = 0;

    k = 0;
    for i = 1:60
        if is_nd
            x = fun_benchmark_RM_nd(x, u, k, param_sim, scenario, demands);
        else
            x = fun_benchmark_RM(x, u, k, param_sim, scenario);
        end
        k = k + 1;
    end

    u_mpc_sr = repmat(0.5, 1, param_MPC_high.Nc);
    u_mpc_sr_0 = u_mpc_sr;
    u_mpc_sr_prev = u_mpc_sr(:, 1);
    u_rl = [1; 1];
    k_c = 0;

    for i = 1:N
        if mod(k_c, param_MPC_low.M) == 0
            if is_nd
                demando1c1 = demands.o1c1(k + 1);
                demando1c2 = demands.o1c2(k + 1);
                demando2c1 = demands.o2c1(k + 1);
                demando2c2 = demands.o2c2(k + 1);
                demando3c1 = demands.o3c1(k + 1);
                demando3c2 = demands.o3c2(k + 1);
            else
                [demando1c1, demando1c2] = demando1(k - 1, scenario);
                [demando2c1, demando2c2] = demando2(k - 1, scenario);
                [demando3c1, demando3c2] = demando3(k - 1, scenario);
            end

            agent_obs = [x ./ x_norm; ...
                [demando1c1, demando1c2]' ./ 1000; ...
                [demando2c1, demando2c2]' ./ 1000; ...
                [demando3c1, demando3c2]' ./ 1000];

            tic;
            u_low = getAction(agent, agent_obs);
            total_comp_time = total_comp_time + toc;
            u_rl = u_low{1};
        end

        if mod(k_c, param_MPC_high.M) == 0
            tic;
            if is_mm
                u_mpc_sr = MPC_solve_SR_w_RL_h_mm(x, u_mpc_sr_0, u_mpc_sr_prev, ...
                    u_rl, k, param_mm, param_MPC_low, param_MPC_high, scenario, agent);
            elseif is_nd
                u_mpc_sr = MPC_solve_SR_w_RL_h(x, u_mpc_sr_0, u_mpc_sr_prev, ...
                    u_rl, k, param_sim, param_MPC_low, param_MPC_high, scenario, agent, demands);
            else
                u_mpc_sr = MPC_solve_SR_w_RL_h(x, u_mpc_sr_0, u_mpc_sr_prev, ...
                    u_rl, k, param_sim, param_MPC_low, param_MPC_high, scenario, agent);
            end
            total_comp_time = total_comp_time + toc;
            u_mpc_sr_0 = [u_mpc_sr(:, 2:end), u_mpc_sr(:, end)];
            u_mpc_sr_prev = u_mpc_sr(:, 1);
        end

        if is_nd
            x = fun_benchmark_RM_nd(x, [u_mpc_sr(:, 1); u_rl], k, param_sim, scenario, demands);
        else
            x = fun_benchmark_RM(x, [u_mpc_sr(:, 1); u_rl], k, param_sim, scenario);
        end

        xx(:, i) = x;
        uu(:, i) = [u_mpc_sr(:, 1); u_rl];
        k = k + 1;
        k_c = k_c + 1;
    end

    metrics = compute_obj_decomposition(xx, uu, param_sim, total_comp_time);
    metrics.agent_file = agent_files(1).name;
    metrics.use_exploration_policy = use_exploration_policy;
    metrics.seed = seed;
end

function metrics = compute_obj_decomposition(xx, uu, param_sim, total_comp_time)
    r_cost = 0.4;
    M_low = 6;

    du = uu(1:3, 2:end) - uu(1:3, 1:end-1);
    u_pen = sum(r_cost .* sum(du.^2, 1)) ./ M_low;

    idx_c1 = [3, 10, 17, 24, 31, 38, 45, 52, 59];
    idx_c2 = [4, 11, 18, 25, 32, 39, 46, 53, 60];
    lambda = [param_sim.lambda.l1, param_sim.lambda.l2, param_sim.lambda.l3, ...
        param_sim.lambda.l4, param_sim.lambda.l5, param_sim.lambda.l6, ...
        param_sim.lambda.l7, param_sim.lambda.l8, param_sim.lambda.l9];

    tts_c1 = param_sim.T .* (sum(xx(idx_c1, :) .* lambda', 1) .* param_sim.L_m ...
        + xx(64, :) + xx(68, :) + xx(72, :));
    tts_c2 = param_sim.T .* (sum(xx(idx_c2, :) .* lambda', 1) .* param_sim.L_m ...
        + xx(65, :) + xx(69, :) + xx(73, :));
    tts = sum(tts_c1 + tts_c2);

    w_o1 = xx(64, :) + xx(65, :);
    w_o2 = xx(68, :) + xx(69, :);
    w_o3 = xx(72, :) + xx(73, :);
    q_pen = sum(max(0, w_o1 - param_sim.w_con(1)).^2 ...
        + max(0, w_o2 - param_sim.w_con(2)).^2 ...
        + max(0, w_o3 - param_sim.w_con(3)).^2);

    metrics.J = u_pen + tts + q_pen;
    metrics.tts = tts;
    metrics.q_pen = q_pen;
    metrics.u_pen = u_pen;
    metrics.total_comp_time = total_comp_time;
    metrics.max_queue_excess = [max(max(0, w_o1 - param_sim.w_con(1))), ...
        max(max(0, w_o2 - param_sim.w_con(2))), ...
        max(max(0, w_o3 - param_sim.w_con(3)))];
end

function restore_path_and_dir(original_dir, agent_folder)
    cd(original_dir);
    rmpath(agent_folder);
end
