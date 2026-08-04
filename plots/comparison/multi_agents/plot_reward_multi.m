%% plot_reward_multi.m
%
% Compares PI-ALINEA MPC, DDPG-MPC, SAC-MPC, and SAC(D)-MPC across all 5 training runs
% (50 experiments each) for all 4 scenarios.
% No Control and Hierarchical MPC are excluded.
%
% Computes total episode reward using the exact formula from rlStepFunc.m:
%   reward_k = -(tts_k + u_pen_k + q_pen_k) / 30   (per RL step)
%   u_pen_k  = r_cost * sum((rl_actions_{k-1} - rl_actions_k).^2)
%              [only the 2 RL-controlled inputs, uu rows 1:2]
%   q_pen_k  = (10 + max(w_o1)/100)*(max(w_o1)>w_con(1))
%            + (10 + max(w_o2)/100)*(max(w_o2)>w_con(2))
%            + (10 + max(w_o3)/100)*(max(w_o3)>w_con(3))
%   Total reward = sum(reward_k) over all 160 RL steps
%
% Higher (less negative) = better performance.
% Best agent identification uses max (highest reward).
%
% Saves: reward_multi.svg  +  reward_multi_table.tex

clear; clc; close all;

%% ── Paths ─────────────────────────────────────────────────────────────────
script_dir = fileparts(mfilename('fullpath'));
agents_dir = fullfile(script_dir, '..', '..', '..');
out_dir    = script_dir;

%% ── Definitions ───────────────────────────────────────────────────────────
scen_labels = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4'};
scen_sfx    = {'', '_nd', '_mm', '_mm_nd'};
n_scen      = 4;

ctrl_labels       = {'PI-ALINEA', 'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
ctrl_labels_plain = {'PI-ALINEA', 'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
n_ctrl            = 4;
plot_ctrl_order   = [1, 2, 4, 3];
n_runs            = 5;

r_cost = 0.4;   % MPC_param.r_cost
M_low  = 6;     % low-level timesteps per RL step

ctrl_clr = [
    0.9290, 0.6940, 0.1250;   % amber  - PI-ALINEA
    0.0000, 0.4470, 0.7410;   % blue   - DDPG-MPC
    0.8350, 0.0980, 0.1020;   % red    - SAC-MPC
    0.4940, 0.1840, 0.5560;   % purple - SAC(D)-MPC
];

%% ── Helper ─────────────────────────────────────────────────────────────────
function total_reward = compute_reward(xx, uu, param_sim, r_cost, M_low)
    % Replicates rlStepFunc reward, summed over all RL steps.
    % xx : (75, N_sim)  - state trajectory at every low-level timestep
    % uu : (3, N_rl)    - control actions at every RL step
    %                     rows 1:2 = RL actions, row 3 = MPC action

    n_rl = size(xx, 2) / M_low;   % 160 RL steps for a full episode

    idx_c1 = [3, 10, 17, 24, 31, 38, 45, 52, 59];
    idx_c2 = [4, 11, 18, 25, 32, 39, 46, 53, 60];
    lambda  = [param_sim.lambda.l1, param_sim.lambda.l2, param_sim.lambda.l3, ...
               param_sim.lambda.l4, param_sim.lambda.l5, param_sim.lambda.l6, ...
               param_sim.lambda.l7, param_sim.lambda.l8, param_sim.lambda.l9];

    total_reward = 0;

    for k = 1:n_rl
        k1   = (k-1)*M_low + 1;
        k2   = k*M_low;
        xx_k = xx(:, k1:k2);

        % TTS over M_low substeps
        tts_c1 = param_sim.T .* ( sum(xx_k(idx_c1,:) .* lambda', 1) .* param_sim.L_m ...
                                 + xx_k(64,:) + xx_k(68,:) + xx_k(72,:) );
        tts_c2 = param_sim.T .* ( sum(xx_k(idx_c2,:) .* lambda', 1) .* param_sim.L_m ...
                                 + xx_k(65,:) + xx_k(69,:) + xx_k(73,:) );
        tts_k  = sum(tts_c1 + tts_c2);

        % u_pen: squared change in RL actions only (rows 1:2 of uu)
        if k == 1
            du = zeros(2, 1);
        else
            du = uu(1:2, k) - uu(1:2, k-1);
        end
        u_pen_k = r_cost * sum(du.^2);

        % q_pen: step penalty when waiting queue exceeds constraint
        w_o1 = xx_k(64,:) + xx_k(65,:);
        w_o2 = xx_k(68,:) + xx_k(69,:);
        w_o3 = xx_k(72,:) + xx_k(73,:);
        q_pen_k = (10 + max(w_o1)./100) .* (max(w_o1) > param_sim.w_con(1)) ...
                + (10 + max(w_o2)./100) .* (max(w_o2) > param_sim.w_con(2)) ...
                + (10 + max(w_o3)./100) .* (max(w_o3) > param_sim.w_con(3));

        total_reward = total_reward + (-(tts_k + u_pen_k + q_pen_k) ./ 30);
    end
end

%% ── Load data ──────────────────────────────────────────────────────────────
rew_data     = cell(n_ctrl, n_scen);
run_idx_data = cell(n_ctrl, n_scen);

fprintf('Loading experiment data...\n');

for s = 1:n_scen
    sfx = scen_sfx{s};
    fprintf('\n  Scenario %d  (sfx = "%s")\n', s, sfx);

    %% 1. PI-ALINEA MPC
    rew_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['PI_ALINEA_MPC' sfx '_' num2str(r)]);
        files  = dir(fullfile(folder, 'PI_ALINEA_MPC_SR_RM*result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'uu', 'param_sim');
            rew_v(end+1, 1) = compute_reward(d.xx, d.uu, d.param_sim, r_cost, M_low);
            ridx(end+1, 1)  = r;
        end
    end
    rew_data{1,s}     = rew_v;
    run_idx_data{1,s} = ridx;
    fprintf('    [1] PI-ALINEA MPC  : %2d experiment(s)\n', numel(rew_v));

    %% 2. DDPG-MPC
    rew_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['DDPGpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'uu', 'param_sim');
            rew_v(end+1, 1) = compute_reward(d.xx, d.uu, d.param_sim, r_cost, M_low);
            ridx(end+1, 1)  = r;
        end
    end
    rew_data{2,s}     = rew_v;
    run_idx_data{2,s} = ridx;
    fprintf('    [2] DDPG-MPC       : %2d experiment(s)\n', numel(rew_v));

    %% 3. SAC-MPC
    rew_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'uu', 'param_sim');
            rew_v(end+1, 1) = compute_reward(d.xx, d.uu, d.param_sim, r_cost, M_low);
            ridx(end+1, 1)  = r;
        end
    end
    rew_data{3,s}     = rew_v;
    run_idx_data{3,s} = ridx;
    fprintf('    [3] SAC-MPC        : %2d experiment(s)\n', numel(rew_v));

    %% 4. SAC(D)-MPC
    rew_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACDpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'uu', 'param_sim');
            rew_v(end+1, 1) = compute_reward(d.xx, d.uu, d.param_sim, r_cost, M_low);
            ridx(end+1, 1)  = r;
        end
    end
    rew_data{4,s}     = rew_v;
    run_idx_data{4,s} = ridx;
    fprintf('    [4] SAC(D)-MPC   : %2d experiment(s)\n', numel(rew_v));
end
fprintf('\nAll data loaded.\n\n');

%% ── Console summary ────────────────────────────────────────────────────────
fprintf('Total episode reward summary (mean ± std):\n');
fprintf('%-18s', '');
for s = 1:n_scen, fprintf('  %-28s', scen_labels{s}); end
fprintf('\n%s\n', repmat('-', 1, 18 + 30*n_scen));
for c = 1:n_ctrl
    fprintf('%-18s', ctrl_labels_plain{c});
    for s = 1:n_scen
        v = rew_data{c,s};
        fprintf('  %-28s', sprintf('%.2f ± %.2f', mean(v), std(v)));
    end
    fprintf('\n');
end
fprintf('\n');

%% ── Figure ─────────────────────────────────────────────────────────────────
ax_fsize  = 13;
ttl_fsize = 14;

all_vals = cell2mat(rew_data(:));
y_lo = floor( min(all_vals) * 1.08);   % reward is negative: extend downward
y_hi = -0.03 * abs(y_lo);              % small margin above the least-negative

fig = figure('Units', 'centimeters', 'Position', [2 2 22 16], 'Color', 'w');
tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for s = 1:n_scen
    ax = nexttile(tl);
    hold(ax, 'on');
    for ci = 1:n_ctrl
        c   = plot_ctrl_order(ci);
        v   = rew_data{c, s};
        clr = ctrl_clr(c, :);
        if numel(v) < 4, v = repmat(v, 4, 1); end
        bc = boxchart(ax, ci * ones(size(v)), v);
        bc.BoxFaceColor     = clr;
        bc.BoxFaceAlpha     = 0.70;
        bc.WhiskerLineColor = clr;
        bc.WhiskerLineStyle = '-';
        bc.LineWidth        = 1.8;
        bc.MarkerStyle      = '+';
        bc.MarkerColor      = clr * 0.75;
        bc.MarkerSize       = 5;
        bc.DisplayName      = ctrl_labels{c};
    end
    hold(ax, 'off');
    ylim(ax, [y_lo, y_hi]);
    ax.XTick              = 1:n_ctrl;
    ax.XTickLabel         = ctrl_labels(plot_ctrl_order);
    ax.XTickLabelRotation = 22;
    ylabel(ax, 'Total Episode Reward', 'FontSize', ax_fsize, 'Interpreter', 'latex');
    title(ax, scen_labels{s}, 'FontSize', ttl_fsize, 'FontWeight', 'bold', 'Interpreter', 'latex');
    set(ax, 'Box', 'on', 'FontSize', ax_fsize, 'LineWidth', 0.8, ...
            'TickDir', 'out', 'TickLabelInterpreter', 'latex');
    grid(ax, 'on');
    ax.GridAlpha     = 0.12;
    ax.GridLineStyle = ':';
end

set(fig, 'Renderer', 'painters');
print(fig, fullfile(out_dir, 'reward_multi.svg'), '-dsvg', '-vector');
fprintf('Saved figure → reward_multi.svg\n');

%% ── LaTeX table ────────────────────────────────────────────────────────────
fid = fopen(fullfile(out_dir, 'reward_multi_table.tex'), 'w');
fprintf(fid, '\\begin{table}[htbp]\n\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of total episode reward ' ...
    '($R = \\sum_k r_k$, $r_k = -(\\mathrm{TTS}_k + u_{\\mathrm{pen},k} + q_{\\mathrm{pen},k})/30$) ' ...
    'for PI-ALINEA, DDPG-MPC, SAC-MPC, and SAC(D)-MPC across all 5 training runs. ' ...
    'Higher (less negative) values indicate better performance.}\n']);
fprintf(fid, '\\label{tab:reward_multi}\n');
fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\n', repmat('c', 1, n_scen));
fprintf(fid, 'Controller');
for s = 1:n_scen, fprintf(fid, ' & %s', scen_labels{s}); end
fprintf(fid, ' \\\\\n\\midrule\n');
for c = 1:n_ctrl
    fprintf(fid, '%s', strrep(ctrl_labels_plain{c}, ' ', '~'));
    for s = 1:n_scen
        v = rew_data{c,s};
        fprintf(fid, ' & $%.2f \\pm %.2f$', mean(v), std(v));
    end
    fprintf(fid, ' \\\\\n');
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n\\end{table}\n');
fclose(fid);
fprintf('Saved table   → reward_multi_table.tex\n');

%% ── Median and best agent identification ───────────────────────────────────
% Note: reward is negative, best agent = maximum (least negative) reward.
fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('Median and best agent per algorithm and scenario\n');
fprintf('(best = highest reward = least negative)\n');
fprintf('%s\n', repmat('=', 1, 70));
fprintf('%-12s  %-12s  %-28s  %-28s\n', ...
        'Algorithm', 'Scenario', 'Median agent (reward)', 'Best agent (reward)');
fprintf('%s\n', repmat('-', 1, 70));
for c = 1:n_ctrl
    for s = 1:n_scen
        v    = rew_data{c,s};
        ridx = run_idx_data{c,s};
        med  = median(v);
        [~, idx]      = min(abs(v - med));
        [~, best_idx] = max(v);          % best = highest reward
        fprintf('%-12s  %-12s  run %-2d (%.2f)          run %-2d (%.2f)\n', ...
                ctrl_labels_plain{c}, scen_labels{s}, ...
                ridx(idx), v(idx), ridx(best_idx), v(best_idx));
    end
    fprintf('\n');
end
