%% plot_tts_comparison_agent.m
%
% Agent-indexed version of plot_tts_comparison.m.
% Set ddpg_agent, sac_agent, and sacd_agent to select a single run (10 experiments each)
% for a fair comparison across all 6 controllers.
%
% Saves: tts_comparison_agent.svg  +  tts_table_agent.tex

clear; clc; close all;

%% ── Paths ─────────────────────────────────────────────────────────────────
script_dir = fileparts(mfilename('fullpath'));
agents_dir = fullfile(script_dir, '..', '..', '..');
out_dir    = script_dir;

if ~exist(out_dir, 'dir'), mkdir(out_dir); end

%% ── Definitions ───────────────────────────────────────────────────────────
scen_labels = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4'};
scen_sfx    = {'', '_nd', '_mm', '_mm_nd'};
n_scen      = 4;

ctrl_labels       = {'No Control', 'PI-ALINEA', 'Hier.\ MPC', 'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
ctrl_labels_plain = {'No Control', 'PI-ALINEA', 'Hier. MPC',  'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
n_ctrl            = 6;
plot_ctrl_order   = [1, 2, 3, 4, 6, 5];
n_runs       = 5;

% ── Agent selection (set to desired agent run indices) ─────────────────────
pi_alinea_agent = [3, 5, 3, 2];   % PI-ALINEA agent run index per scenario (1-5)
ddpg_agent = [3, 2, 5, 5];   % DDPG agent run index per scenario (1-5)
sac_agent  = [4, 1, 5, 3];   % SAC  agent run index per scenario (1-5)
sacd_agent = sac_agent;        % deterministic SAC run index per scenario (1-5)

%% ── Helper: recompute TTS and queue violations from raw xx + param_sim ────
function [tts_total, q1_viol, q2_viol, q3_viol] = extract_metrics(xx, param_sim)
    idx_c1 = [3, 10, 17, 24, 31, 38, 45, 52, 59];
    idx_c2 = [4, 11, 18, 25, 32, 39, 46, 53, 60];
    idx_wo_c1 = [64, 68, 72];
    idx_wo_c2 = [65, 69, 73];

    lambda = [param_sim.lambda.l1, param_sim.lambda.l2, param_sim.lambda.l3, ...
              param_sim.lambda.l4, param_sim.lambda.l5, param_sim.lambda.l6, ...
              param_sim.lambda.l7, param_sim.lambda.l8, param_sim.lambda.l9];

    tts_c1 = param_sim.T .* (sum(xx(idx_c1,:) .* lambda', 1) .* param_sim.L_m ...
                              + sum(xx(idx_wo_c1,:), 1));
    tts_c2 = param_sim.T .* (sum(xx(idx_c2,:) .* lambda', 1) .* param_sim.L_m ...
                              + sum(xx(idx_wo_c2,:), 1));
    tts_total = sum(tts_c1 + tts_c2);

    q1_viol = (max(xx(64,:) + xx(65,:)) - 200) / 2;
    q2_viol =  max(xx(68,:) + xx(69,:)) - 100;
    q3_viol =  max(xx(72,:) + xx(73,:)) - 100;
end

%% ── Load all experiment data ───────────────────────────────────────────────
tts_data = cell(n_ctrl, n_scen);
qv_data  = cell(n_ctrl, n_scen);

fprintf('Loading experiment data (PI-ALINEA agents=%s, DDPG agents=%s, SAC agents=%s, SAC(D) agents=%s)...\n', mat2str(pi_alinea_agent), mat2str(ddpg_agent), mat2str(sac_agent), mat2str(sacd_agent));

for s = 1:n_scen
    sfx = scen_sfx{s};
    fprintf('\n  Scenario %d  (sfx = "%s")\n', s, sfx);

    % ── 1. No Control ──────────────────────────────────────────────────────
    nc_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    nc_files  = dir(fullfile(nc_folder, 'No_control_*.mat'));
    tts_v = zeros(numel(nc_files), 1);
    qv    = zeros(numel(nc_files), 3);
    for f = 1:numel(nc_files)
        d = load(fullfile(nc_folder, nc_files(f).name), 'xx', 'param_sim');
        [tts_v(f), qv(f,1), qv(f,2), qv(f,3)] = extract_metrics(d.xx, d.param_sim);
    end
    tts_data{1,s} = tts_v;
    qv_data{1,s}  = qv;
    fprintf('    [1] No Control     : %2d experiment(s)\n', numel(tts_v));

    % ── 2. PI-ALINEA MPC (single selected agent folder) ────────────────────
    tts_v = []; qv = [];
    folder = fullfile(agents_dir, ['PI_ALINEA_MPC' sfx '_' num2str(pi_alinea_agent(s))]);
    files  = dir(fullfile(folder, 'PI_ALINEA_MPC_SR_RM*result_*.mat'));
    for f = 1:numel(files)
        d = load(fullfile(folder, files(f).name), 'xx', 'param_sim');
        [t, q1, q2, q3] = extract_metrics(d.xx, d.param_sim);
        tts_v(end+1, 1) = t;
        qv(end+1, :)    = [q1, q2, q3];
    end
    tts_data{2,s} = tts_v;
    qv_data{2,s}  = qv;
    fprintf('    [2] PI-ALINEA MPC  : %2d experiment(s)  (agent %d)\n', numel(tts_v), pi_alinea_agent(s));

    % ── 3. Hierarchical MPC (10 experiments in DDPGpi_main_s5_1 folder) ────
    hier_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    hier_files  = dir(fullfile(hier_folder, 'MPC_RM_SR_hier*result_*.mat'));
    tts_v = zeros(numel(hier_files), 1);
    qv    = zeros(numel(hier_files), 3);
    for f = 1:numel(hier_files)
        d = load(fullfile(hier_folder, hier_files(f).name), 'xx', 'param_sim');
        [tts_v(f), qv(f,1), qv(f,2), qv(f,3)] = extract_metrics(d.xx, d.param_sim);
    end
    tts_data{3,s} = tts_v;
    qv_data{3,s}  = qv;
    fprintf('    [3] Hier. MPC      : %2d experiment(s)\n', numel(tts_v));

    % ── 4. DDPG-MPC (single selected agent folder) ────────────────────────
    tts_v = []; qv = [];
    folder = fullfile(agents_dir, ['DDPGpi_main_s5_' num2str(ddpg_agent(s)) sfx]);
    files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
    for f = 1:numel(files)
        d = load(fullfile(folder, files(f).name), 'xx', 'param_sim');
        [t, q1, q2, q3] = extract_metrics(d.xx, d.param_sim);
        tts_v(end+1, 1) = t;
        qv(end+1, :)    = [q1, q2, q3];
    end
    tts_data{4,s} = tts_v;
    qv_data{4,s}  = qv;
    fprintf('    [4] DDPG-MPC       : %2d experiment(s)  (agent %d)\n', numel(tts_v), ddpg_agent(s));

    % ── 5. SAC-MPC (single selected agent folder) ─────────────────────────
    tts_v = []; qv = [];
    folder = fullfile(agents_dir, ['SACpi_main_s5_' num2str(sac_agent(s)) sfx]);
    files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
    for f = 1:numel(files)
        d = load(fullfile(folder, files(f).name), 'xx', 'param_sim');
        [t, q1, q2, q3] = extract_metrics(d.xx, d.param_sim);
        tts_v(end+1, 1) = t;
        qv(end+1, :)    = [q1, q2, q3];
    end
    tts_data{5,s} = tts_v;
    qv_data{5,s}  = qv;
    fprintf('    [5] SAC-MPC        : %2d experiment(s)  (agent %d)\n', numel(tts_v), sac_agent(s));

    % ── 6. SAC(D)-MPC (single selected agent folder) ─────────────────────────
    tts_v = []; qv = [];
    folder = fullfile(agents_dir, ['SACDpi_main_s5_' num2str(sacd_agent(s)) sfx]);
    files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
    for f = 1:numel(files)
        d = load(fullfile(folder, files(f).name), 'xx', 'param_sim');
        [t, q1, q2, q3] = extract_metrics(d.xx, d.param_sim);
        tts_v(end+1, 1) = t;
        qv(end+1, :)    = [q1, q2, q3];
    end
    tts_data{6,s} = tts_v;
    qv_data{6,s}  = qv;
    fprintf('    [6] SAC(D)-MPC   : %2d experiment(s)  (agent %d)\n', numel(tts_v), sacd_agent(s));
end
fprintf('\nAll data loaded.\n\n');

%% ── Console summary: TTS mean ± std + % improvement over No Control ───────
fprintf('%-18s', '');
for s = 1:n_scen, fprintf('  %-26s', scen_labels{s}); end
fprintf('\n%s\n', repmat('-', 1, 18 + 29*n_scen));

for c = 1:n_ctrl
    fprintf('%-18s', ctrl_labels_plain{c});
    for s = 1:n_scen
        v  = tts_data{c,s};
        nc = mean(tts_data{1,s});
        if numel(v) == 1
            pct = 100*(v - nc)/nc;
            fprintf('  %-26s', sprintf('%.1f  (%+.1f%%)', v, pct));
        else
            pct = 100*(mean(v) - nc)/nc;
            fprintf('  %-26s', sprintf('%.1f±%.1f  (%+.1f%%)', mean(v), std(v), pct));
        end
    end
    fprintf('\n');
end
fprintf('\n');

fprintf('Peak queue violations (max over simulation, positive = violated):\n');
fprintf('%-18s  %-10s  %s\n', 'Controller', 'Scenario', 'Q1 | Q2 | Q3');
fprintf('%s\n', repmat('-', 1, 70));
for c = 1:n_ctrl
    for s = 1:n_scen
        qv = qv_data{c,s};
        if isempty(qv)
            fprintf('%-18s  %-10s  %s\n', ctrl_labels_plain{c}, scen_labels{s}, 'no data');
        else
            fprintf('%-18s  %-10s  %6.2f | %6.2f | %6.2f\n', ...
                    ctrl_labels_plain{c}, scen_labels{s}, ...
                    mean(qv(:,1)), mean(qv(:,2)), mean(qv(:,3)));
        end
    end
end
fprintf('\n');

%% ── Visual style ───────────────────────────────────────────────────────────
ax_fsize  = 13;
ttl_fsize = 14;

ctrl_clr = [
    0.5000, 0.5000, 0.5000;   % gray   - No Control
    0.9290, 0.6940, 0.1250;   % amber  - PI-ALINEA
    0.4660, 0.6740, 0.1880;   % green  - Hier. MPC
    0.0000, 0.4470, 0.7410;   % blue   - DDPG-MPC
    0.8350, 0.0980, 0.1020;   % red    - SAC-MPC
    0.4940, 0.1840, 0.5560;   % purple - SAC(D)-MPC
];

all_tts_vals = cell2mat(tts_data(:));
y_lo = floor(min(all_tts_vals) / 10) * 10 - 10;
y_hi = ceil( max(all_tts_vals) / 10) * 10 + 10;

%% ── Box-plot figure (2×2) ──────────────────────────────────────────────────
function fig = make_boxplot_figure(tts_data, ctrl_clr, ctrl_labels, plot_ctrl_order, ...
                                   scen_labels, y_lo, y_hi, ax_fsize, ttl_fsize)
    n_ctrl = size(ctrl_clr, 1);
    n_scen = numel(scen_labels);
    nc_clr = ctrl_clr(1, :);

    fig = figure('Units', 'centimeters', 'Position', [2 2 22 16], 'Color', 'w');
    tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for s = 1:n_scen
        ax = nexttile(tl);
        hold(ax, 'on');

        det_nc = (s == 1 || s == 3);

        if det_nc
            nc_mean = mean(tts_data{1, s});
            hl = yline(ax, nc_mean, '--', 'Color', nc_clr, 'LineWidth', 1.5, ...
                       'DisplayName', 'No Control');
            ctrl_idx = plot_ctrl_order(plot_ctrl_order ~= 1);
        else
            hl       = [];
            ctrl_idx = plot_ctrl_order;
        end

        n_boxes  = numel(ctrl_idx);
        h_boxes  = gobjects(n_boxes, 1);

        for ci = 1:n_boxes
            c   = ctrl_idx(ci);
            v   = tts_data{c, s};
            clr = ctrl_clr(c, :);

            if numel(v) < 4
                v = repmat(v, 4, 1);
            end

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
            h_boxes(ci)         = bc;
        end

        hold(ax, 'off');
        ylim(ax, [y_lo, y_hi]);
        ax.XTick              = 1:n_boxes;
        ax.XTickLabel         = ctrl_labels(ctrl_idx);
        ax.XTickLabelRotation = 22;
        ylabel(ax, 'TTS (veh$\cdot$h)', 'FontSize', ax_fsize, 'Interpreter', 'latex');
        title(ax,  scen_labels{s}, 'FontSize', ttl_fsize, 'FontWeight', 'bold', ...
              'Interpreter', 'latex');
        set(ax, 'Box', 'on', 'FontSize', ax_fsize, 'LineWidth', 0.8, ...
                'TickDir', 'out', 'TickLabelInterpreter', 'latex');
        grid(ax, 'on');
        ax.GridAlpha     = 0.12;
        ax.GridLineStyle = ':';

        if det_nc
            set(h_boxes, 'HandleVisibility', 'off');
            legend(ax, hl, 'Location', 'southeast', ...
                   'FontSize', ax_fsize - 1, 'Interpreter', 'latex', 'Box', 'off');
        end
    end
end

%% ── Generate figure + LaTeX table ─────────────────────────────────────────
fig = make_boxplot_figure(tts_data, ctrl_clr, ctrl_labels, plot_ctrl_order, ...
                          scen_labels, y_lo, y_hi, ax_fsize, ttl_fsize);
set(fig, 'Renderer', 'painters');
out_fig = fullfile(out_dir, 'tts_comparison_agent.svg');
print(fig, out_fig, '-dsvg', '-vector');
fprintf('Saved figure → %s\n', out_fig);

% ── LaTeX table ─────────────────────────────────────────────────────────────
tex_file = fullfile(out_dir, 'tts_table_agent.tex');
fid      = fopen(tex_file, 'w');

fprintf(fid, '\\begin{table}[htbp]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of Total Time Spent ' ...
    '(TTS, veh$\\cdot$h) across all experimental runs for each controller ' ...
    'and scenario. PI-ALINEA uses agent~%s; DDPG-MPC uses agent~%s; SAC-MPC uses agent~%s; SAC(D)-MPC uses agent~%s. ' ...
    'Values in parentheses show the percentage change relative ' ...
    'to the No Control baseline.}\n'], mat2str(pi_alinea_agent), mat2str(ddpg_agent), mat2str(sac_agent), mat2str(sacd_agent));
fprintf(fid, '\\label{tab:tts_comparison_agent}\n');
fprintf(fid, '\\begin{tabular}{l%s}\n', repmat('c', 1, n_scen));
fprintf(fid, '\\toprule\n');

fprintf(fid, 'Controller');
for s = 1:n_scen
    fprintf(fid, ' & %s', scen_labels{s});
end
fprintf(fid, ' \\\\\n\\midrule\n');

for c = 1:n_ctrl
    fprintf(fid, '%s', strrep(ctrl_labels_plain{c}, ' ', '~'));
    for s = 1:n_scen
        v  = tts_data{c,s};
        nc = mean(tts_data{1,s});
        if numel(unique(v)) == 1
            pct = 100*(v(1) - nc)/nc;
            if c == 1
                fprintf(fid, ' & $%.1f$', v(1));
            else
                fprintf(fid, ' & $%.1f$ ($%+.1f\\%%$)', v(1), pct);
            end
        else
            pct = 100*(mean(v) - nc)/nc;
            if c == 1
                fprintf(fid, ' & $%.1f \\pm %.1f$', mean(v), std(v));
            else
                fprintf(fid, ' & $%.1f \\pm %.1f$ ($%+.1f\\%%$)', ...
                        mean(v), std(v), pct);
            end
        end
    end
    fprintf(fid, ' \\\\\n');
end

fprintf(fid, '\\bottomrule\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n');
fclose(fid);
fprintf('Saved LaTeX table     → %s\n', tex_file);
