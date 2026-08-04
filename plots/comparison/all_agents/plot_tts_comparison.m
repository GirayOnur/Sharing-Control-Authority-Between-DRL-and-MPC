%% plot_tts_comparison.m
%
% Loads raw simulation result .mat files for all 6 controllers × 4 scenarios,
% recomputes TTS (Total Time Spent, veh·h) and peak queue constraint
% violations from the saved state matrix xx and param_sim, then generates:
%
%   Option A: 2×2 box plot figure  +  LaTeX summary table (tts_table.tex)
%   Option B: 2×2 box plot figure  (stand-alone, no table)
%
% Controllers (6):
%   1. No Control       - deterministic baseline (1 run for S1/S3, 10 for S2/S4)
%   2. PI-ALINEA MPC    - feedback controller  (5 runs × 10 experiments)
%   3. Hier. MPC        - hierarchical MPC      (10 experiments, 1 run)
%   4. DDPG-MPC         - proposed DRL-MPC      (5 runs × 10 experiments)
%   5. SAC-MPC          - proposed DRL-MPC      (5 runs × 10 experiments)
%   6. SACD-MPC       - deterministic SAC-MPC (5 runs x 10 experiments)
%
% Scenarios (4):
%   S1 - base (no suffix)   S2 - demand noise (_nd)
%   S3 - model mismatch (_mm)   S4 - both (_mm_nd)
%
% Discussion context embedded in figure design:
%   • Absolute TTS enables direct comparison and % improvement over No Control
%   • Common Y-axis across all 4 subplots → cross-scenario degradation visible
%   • DDPG/SAC/SACD pooled across 5 seeds → spread reflects both training and
%     test variability; wide spread = low robustness
%   • S1→S2, S3→S4 reveal demand-noise sensitivity per controller
%   • S1→S3, S2→S4 reveal model-mismatch sensitivity per controller

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

ctrl_labels       = {'No Control', 'SF-MPC', 'Hier.\ MPC', 'DDPG-MPC', 'SAC-MPC', 'SACD-MPC'};
ctrl_labels_plain = {'No Control', 'SF-MPC', 'Hier. MPC',  'DDPG-MPC', 'SAC-MPC', 'SACD-MPC'};
n_ctrl            = 6;
plot_ctrl_order   = [1, 2, 3, 4, 6, 5];
n_runs       = 5;

%% ── Helper: recompute TTS and queue violations from raw xx + param_sim ────
% NOTE: benchmark scripts call save() BEFORE computing TTS, so TTS is NOT
%       stored in the .mat file, it must be recomputed here.
%
% TTS formula (matches all benchmark scripts exactly):
%   TTS(t) = T * [ (Σ rho_seg_c1 * lambda_seg) * L_m  +  Σ w_o_c1 ]
%           +T * [ (Σ rho_seg_c2 * lambda_seg) * L_m  +  Σ w_o_c2 ]
%   Total TTS = sum(TTS) over all time steps.
%
% Queue violation (from network_analyzer.m):
%   Q1 = (max(xx(64,:)+xx(65,:)) - 200) / 2
%   Q2 =  max(xx(68,:)+xx(69,:)) - 100
%   Q3 =  max(xx(72,:)+xx(73,:)) - 100

function [tts_total, q1_viol, q2_viol, q3_viol] = extract_metrics(xx, param_sim)
    % Row indices for class-1 densities (rho_seg_c1): 9 mainline segments
    idx_c1 = [3, 10, 17, 24, 31, 38, 45, 52, 59];
    % Row indices for class-2 densities (rho_seg_c2)
    idx_c2 = [4, 11, 18, 25, 32, 39, 46, 53, 60];
    % On-ramp queue lengths
    idx_wo_c1 = [64, 68, 72];
    idx_wo_c2 = [65, 69, 73];

    lambda = [param_sim.lambda.l1, param_sim.lambda.l2, param_sim.lambda.l3, ...
              param_sim.lambda.l4, param_sim.lambda.l5, param_sim.lambda.l6, ...
              param_sim.lambda.l7, param_sim.lambda.l8, param_sim.lambda.l9];

    % Time-series TTS contributions
    tts_c1 = param_sim.T .* (sum(xx(idx_c1,:) .* lambda', 1) .* param_sim.L_m ...
                              + sum(xx(idx_wo_c1,:), 1));
    tts_c2 = param_sim.T .* (sum(xx(idx_c2,:) .* lambda', 1) .* param_sim.L_m ...
                              + sum(xx(idx_wo_c2,:), 1));
    tts_total = sum(tts_c1 + tts_c2);

    % Peak queue violations (positive = constraint violated)
    q1_viol = (max(xx(64,:) + xx(65,:)) - 200) / 2;
    q2_viol =  max(xx(68,:) + xx(69,:)) - 100;
    q3_viol =  max(xx(72,:) + xx(73,:)) - 100;
end

%% ── Load all experiment data ───────────────────────────────────────────────
% tts_data{ctrl, scen}  = column vector of total TTS values  [veh*h]
% qv_data{ctrl, scen}   = [n_exp × 3] matrix: [Q1_viol, Q2_viol, Q3_viol]
tts_data     = cell(n_ctrl, n_scen);
qv_data      = cell(n_ctrl, n_scen);
run_idx_data = cell(n_ctrl, n_scen);  % run index (1-5) for each experiment

fprintf('Loading experiment data...\n');

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

    % ── 2. PI-ALINEA MPC (5 independent run folders) ───────────────────────
    tts_v = []; qv = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['PI_ALINEA_MPC' sfx '_' num2str(r)]);
        files  = dir(fullfile(folder, 'PI_ALINEA_MPC_SR_RM*result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'param_sim');
            [t, q1, q2, q3] = extract_metrics(d.xx, d.param_sim);
            tts_v(end+1, 1) = t;
            qv(end+1, :)    = [q1, q2, q3];
            ridx(end+1, 1)  = r;
        end
    end
    tts_data{2,s}     = tts_v;
    qv_data{2,s}      = qv;
    run_idx_data{2,s} = ridx;
    fprintf('    [2] PI-ALINEA MPC  : %2d experiment(s)\n', numel(tts_v));

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

    % ── 4. DDPG-MPC (5 run folders × 10 experiments) ──────────────────────
    tts_v = []; qv = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['DDPGpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'param_sim');
            [t, q1, q2, q3] = extract_metrics(d.xx, d.param_sim);
            tts_v(end+1, 1) = t;
            qv(end+1, :)    = [q1, q2, q3];
            ridx(end+1, 1)  = r;
        end
    end
    tts_data{4,s}     = tts_v;
    qv_data{4,s}      = qv;
    run_idx_data{4,s} = ridx;
    fprintf('    [4] DDPG-MPC       : %2d experiment(s)\n', numel(tts_v));

    % ── 5. SAC-MPC (5 run folders × 10 experiments) ───────────────────────
    tts_v = []; qv = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'param_sim');
            [t, q1, q2, q3] = extract_metrics(d.xx, d.param_sim);
            tts_v(end+1, 1) = t;
            qv(end+1, :)    = [q1, q2, q3];
            ridx(end+1, 1)  = r;
        end
    end
    tts_data{5,s}     = tts_v;
    qv_data{5,s}      = qv;
    run_idx_data{5,s} = ridx;
    fprintf('    [5] SAC-MPC        : %2d experiment(s)\n', numel(tts_v));

    % ── 6. SACD-MPC (5 run folders × 10 experiments) ───────────────────────
    tts_v = []; qv = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACDpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'param_sim');
            [t, q1, q2, q3] = extract_metrics(d.xx, d.param_sim);
            tts_v(end+1, 1) = t;
            qv(end+1, :)    = [q1, q2, q3];
            ridx(end+1, 1)  = r;
        end
    end
    tts_data{6,s}     = tts_v;
    qv_data{6,s}      = qv;
    run_idx_data{6,s} = ridx;
    fprintf('    [6] SACD-MPC   : %2d experiment(s)\n', numel(tts_v));
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

% Controller colours (same blue/red for DDPG/SAC as in learning curves)
ctrl_clr = [
    0.5000, 0.5000, 0.5000;   % gray   - No Control
    0.9290, 0.6940, 0.1250;   % amber  - PI-ALINEA
    0.4660, 0.6740, 0.1880;   % green  - Hier. MPC
    0.0000, 0.4470, 0.7410;   % blue   - DDPG-MPC
    0.8350, 0.0980, 0.1020;   % red    - SAC-MPC
    0.4940, 0.1840, 0.5560;   % purple - SACD-MPC
];

% Compute global TTS range for a shared Y-axis across all subplots
all_tts_vals = cell2mat(tts_data(:));
y_lo = floor(min(all_tts_vals) / 10) * 10 - 10;
y_hi = ceil( max(all_tts_vals) / 10) * 10 + 10;

%% ── Box-plot figure (2×2) ──────────────────────────────────────────────────
% S1/S3 (deterministic No Control): show dashed reference line + legend entry,
%                                    no No Control box.
% S2/S4 (stochastic No Control):    show No Control box, no dashed line.
function fig = make_boxplot_figure(tts_data, ctrl_clr, ctrl_labels, plot_ctrl_order, ...
                                   scen_labels, y_lo, y_hi, ax_fsize, ttl_fsize)
    n_ctrl = size(ctrl_clr, 1);
    n_scen = numel(scen_labels);
    nc_clr = ctrl_clr(1, :);   % No Control colour (gray)

    fig = figure('Units', 'centimeters', 'Position', [2 2 22 16], 'Color', 'w');
    tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for s = 1:n_scen
        ax = nexttile(tl);
        hold(ax, 'on');

        det_nc = (s == 1 || s == 3);   % true for deterministic No Control scenarios

        if det_nc
            % Dashed reference line only, No Control box is omitted
            nc_mean = mean(tts_data{1, s});
            hl = yline(ax, nc_mean, '--', 'Color', nc_clr, 'LineWidth', 1.5, ...
                       'DisplayName', 'No Control');
            ctrl_idx = plot_ctrl_order(plot_ctrl_order ~= 1);
        else
            % No Control box included, no dashed line
            hl       = [];
            ctrl_idx = plot_ctrl_order;
        end

        n_boxes  = numel(ctrl_idx);
        h_boxes  = gobjects(n_boxes, 1);

        for ci = 1:n_boxes
            c   = ctrl_idx(ci);
            v   = tts_data{c, s};
            clr = ctrl_clr(c, :);

            % Replicate scalar values so boxchart renders a visible median line
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
        ylabel(ax, 'TTS (veh$\cdot$h)', 'FontSize', ax_fsize, 'Interpreter', 'latex', 'Color', 'k');
        title(ax,  scen_labels{s}, 'FontSize', ttl_fsize, 'FontWeight', 'bold', ...
              'Interpreter', 'latex', 'Color', 'k');
        set(ax, 'Box', 'on', 'FontSize', ax_fsize, 'LineWidth', 0.8, ...
                'TickDir', 'out', 'TickLabelInterpreter', 'latex', ...
                'Color', 'w', 'XColor', 'k', 'YColor', 'k');
        grid(ax, 'on');
        ax.GridAlpha     = 0.12;
        ax.GridLineStyle = ':';

        % Show only the dashed No Control line in the legend for S1/S3
        % (box labels are already on the x-axis)
        if det_nc
            set(h_boxes, 'HandleVisibility', 'off');
            lgd = legend(ax, hl, 'Location', 'none', ...
                   'Orientation', 'horizontal', ...
                   'FontSize', ax_fsize - 1, 'Interpreter', 'latex', 'Box', 'off', ...
                   'TextColor', 'k');
            lgd.ItemTokenSize = [18, 8];
            drawnow limitrate;
            ax.Units = 'normalized';
            lgd.Units = 'normalized';
            legend_center_gap = 0.012;
            legend_x_shift = -0.004;
            if s == 3
                legend_x_shift = 0.009;
            end
            ax_pos = ax.Position;
            lgd_pos = lgd.Position;
            lgd.Position = [ax_pos(1) + ax_pos(3) - lgd_pos(3) + legend_x_shift, ...
                            ax_pos(2) + ax_pos(4) + legend_center_gap - 0.5 * lgd_pos(4), ...
                            lgd_pos(3), lgd_pos(4)];
        end
    end
end

%% ── Generate figure + LaTeX table ─────────────────────────────────────────
fig = make_boxplot_figure(tts_data, ctrl_clr, ctrl_labels, plot_ctrl_order, ...
                          scen_labels, y_lo, y_hi, ax_fsize, ttl_fsize);
set(fig, 'Renderer', 'painters');
fig.Units = 'centimeters';
fig_pos = fig.Position;
set(fig, 'PaperUnits', 'centimeters', ...
         'PaperSize', fig_pos(3:4), ...
         'PaperPosition', [0 0 fig_pos(3) fig_pos(4)], ...
         'PaperPositionMode', 'manual');
out_fig = fullfile(out_dir, 'tts_comparison.svg');
print(fig, out_fig, '-dsvg', '-vector');
fprintf('Saved figure → %s\n', out_fig);
out_pdf = fullfile(out_dir, 'tts_comparison.pdf');
print(fig, out_pdf, '-dpdf', '-vector');
fprintf('Saved PDF figure -> %s\n', out_pdf);

% ── LaTeX table ─────────────────────────────────────────────────────────────
tex_file = fullfile(out_dir, 'tts_table.tex');
fid      = fopen(tex_file, 'w');

fprintf(fid, '\\begin{table}[htbp]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of Total Time Spent ' ...
    '(TTS, veh$\\cdot$h) across all experimental runs for each controller ' ...
    'and scenario. Values in parentheses show the percentage change relative ' ...
    'to the No Control baseline.}\n']);
fprintf(fid, '\\label{tab:tts_comparison}\n');
fprintf(fid, '\\begin{tabular}{l%s}\n', repmat('c', 1, n_scen));
fprintf(fid, '\\toprule\n');

% Header row
fprintf(fid, 'Controller');
for s = 1:n_scen
    fprintf(fid, ' & %s', scen_labels{s});
end
fprintf(fid, ' \\\\\n\\midrule\n');

% Data rows
for c = 1:n_ctrl
    fprintf(fid, '%s', strrep(ctrl_labels_plain{c}, ' ', '~'));
    for s = 1:n_scen
        v  = tts_data{c,s};
        nc = mean(tts_data{1,s});
        if numel(unique(v)) == 1
            % Deterministic case (No Control S1/S3): single unique value
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

%% ── Median-agent identification for DDPG and SAC ──────────────────────────
% For each algorithm × scenario, find the experiment whose TTS is closest
% to the overall median, and report which agent run (1-5) it belongs to.
fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('Median-representative agent per algorithm and scenario\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('%-10s  %-12s  %-8s  %-10s  %s\n', ...
        'Algorithm', 'Scenario', 'Agent', 'TTS (med)', 'Median of all');
fprintf('%s\n', repmat('-', 1, 60));

drl_ctrl  = {2, 4, 5, 6};
drl_names = {'SF-MPC', 'DDPG-MPC', 'SAC-MPC', 'SACD-MPC'};

for di = 1:numel(drl_ctrl)
    c = drl_ctrl{di};
    for s = 1:n_scen
        tts_v = tts_data{c, s};
        ridx  = run_idx_data{c, s};
        med   = median(tts_v);
        [~, idx]      = min(abs(tts_v - med));
        [~, best_idx] = min(tts_v);
        fprintf('%-10s  %-12s  med: run %-2d (%8.2f)   best: run %-2d (%8.2f)\n', ...
                drl_names{di}, scen_labels{s}, ridx(idx), tts_v(idx), ridx(best_idx), tts_v(best_idx));
    end
    fprintf('\n');
end
