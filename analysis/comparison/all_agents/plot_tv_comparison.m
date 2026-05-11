%% plot_tv_comparison.m
%
% Loads raw simulation result .mat files for all 6 controllers × 4 scenarios
% and computes the total input variation (TV) as a scalar:
%
%   TV = Σ_{k=2}^{N} ||u(k) − u(k−1)||_2
%
% where u(k) ∈ R^3 is the control input vector at time step k,
% and || · ||_2 denotes the Euclidean (L2) norm.
%
% Generates a 2×2 box-plot figure (one subplot per scenario):
%   S1/S3 – No Control shown as dashed reference line (deterministic, TV = 0)
%   S2/S4 – No Control shown as a box (stochastic, TV ≈ 0)
%
% Saves: tv_comparison.svg  +  tv_table.tex

clear; clc; close all;

%% ── Paths ─────────────────────────────────────────────────────────────────
script_dir = fileparts(mfilename('fullpath'));
agents_dir = fullfile(script_dir, '..', '..', '..');
out_dir    = script_dir;

%% ── Definitions ───────────────────────────────────────────────────────────
scen_labels = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4'};
scen_sfx    = {'', '_nd', '_mm', '_mm_nd'};
n_scen      = 4;

ctrl_labels       = {'No Control', 'PI-ALINEA', 'Hier.\ MPC', 'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
ctrl_labels_plain = {'No Control', 'PI-ALINEA', 'Hier. MPC',  'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
n_ctrl            = 6;
n_runs            = 5;

%% ── Helper: compute TV from raw uu ────────────────────────────────────────
function tv = compute_tv(uu)
    % uu : [n_inputs × N]
    % TV = sum of L2 norms of consecutive differences
    tv = sum(vecnorm(diff(uu, 1, 2), 2, 1));
end

%% ── Load data ──────────────────────────────────────────────────────────────
tv_data      = cell(n_ctrl, n_scen);
run_idx_data = cell(n_ctrl, n_scen);

fprintf('Loading experiment data...\n');

for s = 1:n_scen
    sfx = scen_sfx{s};
    fprintf('\n  Scenario %d  (sfx = "%s")\n', s, sfx);

    %% 1. No Control
    nc_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    nc_files  = dir(fullfile(nc_folder, 'No_control_*.mat'));
    tv_v = zeros(numel(nc_files), 1);
    for f = 1:numel(nc_files)
        d = load(fullfile(nc_folder, nc_files(f).name), 'uu');
        tv_v(f) = compute_tv(d.uu);
    end
    tv_data{1,s} = tv_v;
    fprintf('    [1] No Control     : %2d experiment(s)\n', numel(tv_v));

    %% 2. PI-ALINEA MPC
    tv_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['PI_ALINEA_MPC' sfx '_' num2str(r)]);
        files  = dir(fullfile(folder, 'PI_ALINEA_MPC_SR_RM*result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'uu');
            tv_v(end+1, 1) = compute_tv(d.uu);
            ridx(end+1, 1) = r;
        end
    end
    tv_data{2,s}      = tv_v;
    run_idx_data{2,s} = ridx;
    fprintf('    [2] PI-ALINEA MPC  : %2d experiment(s)\n', numel(tv_v));

    %% 3. Hierarchical MPC
    hier_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    hier_files  = dir(fullfile(hier_folder, 'MPC_RM_SR_hier*result_*.mat'));
    tv_v = zeros(numel(hier_files), 1);
    for f = 1:numel(hier_files)
        d = load(fullfile(hier_folder, hier_files(f).name), 'uu');
        tv_v(f) = compute_tv(d.uu);
    end
    tv_data{3,s} = tv_v;
    fprintf('    [3] Hier. MPC      : %2d experiment(s)\n', numel(tv_v));

    %% 4. DDPG-MPC
    tv_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['DDPGpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'uu');
            tv_v(end+1, 1) = compute_tv(d.uu);
            ridx(end+1, 1) = r;
        end
    end
    tv_data{4,s}      = tv_v;
    run_idx_data{4,s} = ridx;
    fprintf('    [4] DDPG-MPC       : %2d experiment(s)\n', numel(tv_v));

    %% 5. SAC-MPC
    tv_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'uu');
            tv_v(end+1, 1) = compute_tv(d.uu);
            ridx(end+1, 1) = r;
        end
    end
    tv_data{5,s}      = tv_v;
    run_idx_data{5,s} = ridx;
    fprintf('    [5] SAC-MPC        : %2d experiment(s)\n', numel(tv_v));

    %% 6. SAC(D)-MPC
    tv_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACDpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'uu');
            tv_v(end+1, 1) = compute_tv(d.uu);
            ridx(end+1, 1) = r;
        end
    end
    tv_data{6,s}      = tv_v;
    run_idx_data{6,s} = ridx;
    fprintf('    [6] SAC(D)-MPC   : %2d experiment(s)\n', numel(tv_v));
end
fprintf('\nAll data loaded.\n\n');

%% ── Console summary ────────────────────────────────────────────────────────
fprintf('TV summary (mean ± std):\n');
fprintf('%-18s', '');
for s = 1:n_scen, fprintf('  %-22s', scen_labels{s}); end
fprintf('\n%s\n', repmat('-', 1, 18 + 24*n_scen));
for c = 1:n_ctrl
    fprintf('%-18s', ctrl_labels_plain{c});
    for s = 1:n_scen
        v = tv_data{c,s};
        if numel(unique(v)) == 1
            fprintf('  %-22s', sprintf('%.4f (det.)', v(1)));
        else
            fprintf('  %-22s', sprintf('%.4f ± %.4f', mean(v), std(v)));
        end
    end
    fprintf('\n');
end
fprintf('\n');

%% ── Visual style ───────────────────────────────────────────────────────────
ax_fsize  = 13;
ttl_fsize = 14;

ctrl_clr = [
    0.5000, 0.5000, 0.5000;   % gray   – No Control
    0.9290, 0.6940, 0.1250;   % amber  – PI-ALINEA
    0.4660, 0.6740, 0.1880;   % green  – Hier. MPC
    0.0000, 0.4470, 0.7410;   % blue   – DDPG-MPC
    0.8350, 0.0980, 0.1020;   % red    – SAC-MPC
    0.4940, 0.1840, 0.5560;   % purple - SAC(D)-MPC
];

all_vals = cell2mat(tv_data(:));
y_hi = ceil( max(all_vals) * 1.08);
y_lo = -0.03 * y_hi;

%% ── 2×2 box-plot figure ───────────────────────────────────────────────────
fig = figure('Units', 'centimeters', 'Position', [2 2 22 16], 'Color', 'w');
tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for s = 1:n_scen
    ax = nexttile(tl);
    hold(ax, 'on');

    det_nc = (s == 1 || s == 3);

    if det_nc
        nc_val = mean(tv_data{1,s});
        hl = yline(ax, nc_val, '--', 'Color', ctrl_clr(1,:), 'LineWidth', 1.5, ...
                   'DisplayName', 'No Control');
        ctrl_idx = 2:n_ctrl;
    else
        hl       = [];
        ctrl_idx = 1:n_ctrl;
    end

    n_boxes = numel(ctrl_idx);
    h_boxes = gobjects(n_boxes, 1);

    for ci = 1:n_boxes
        c   = ctrl_idx(ci);
        v   = tv_data{c, s};
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
    ylabel(ax, '$\mathrm{TV}$', 'FontSize', ax_fsize, 'Interpreter', 'latex');
    title(ax, scen_labels{s}, 'FontSize', ttl_fsize, ...
          'FontWeight', 'bold', 'Interpreter', 'latex');
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

set(fig, 'Renderer', 'painters');
out_fig = fullfile(out_dir, 'tv_comparison.svg');
print(fig, out_fig, '-dsvg', '-vector');
fprintf('Saved figure → %s\n', out_fig);

%% ── LaTeX table ────────────────────────────────────────────────────────────
tex_file = fullfile(out_dir, 'tv_table.tex');
fid      = fopen(tex_file, 'w');
fprintf(fid, '\\begin{table}[htbp]\n\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of total input ' ...
    'variation (TV) across all experimental runs.}\n']);
fprintf(fid, '\\label{tab:tv_comparison}\n');
fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\n', repmat('c', 1, n_scen));
fprintf(fid, 'Controller');
for s = 1:n_scen, fprintf(fid, ' & %s', scen_labels{s}); end
fprintf(fid, ' \\\\\n\\midrule\n');
for c = 1:n_ctrl
    fprintf(fid, '%s', strrep(ctrl_labels_plain{c}, ' ', '~'));
    for s = 1:n_scen
        v = tv_data{c,s};
        if numel(unique(v)) == 1
            fprintf(fid, ' & $%.4f$', v(1));
        else
            fprintf(fid, ' & $%.4f \\pm %.4f$', mean(v), std(v));
        end
    end
    fprintf(fid, ' \\\\\n');
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n\\end{table}\n');
fclose(fid);
fprintf('Saved LaTeX table → %s\n', tex_file);

%% ── Median-agent identification for DDPG and SAC ──────────────────────────
fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('Median-representative agent (TV)\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('%-10s  %-12s  %-8s  %-12s  %s\n', ...
        'Algorithm', 'Scenario', 'Agent', 'TV(med)', 'Median of all');
fprintf('%s\n', repmat('-', 1, 60));
drl_ctrl  = {2, 4, 5, 6};
drl_names = {'PI-ALINEA', 'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};

for di = 1:numel(drl_ctrl)
    c     = drl_ctrl{di};
    for s = 1:n_scen
        v    = tv_data{c,s};
        ridx = run_idx_data{c,s};
        med  = median(v);
        [~, idx]      = min(abs(v - med));
        [~, best_idx] = min(v);
        fprintf('%-10s  %-12s  med: run %-2d (%.4f)   best: run %-2d (%.4f)\n', ...
                drl_names{di}, scen_labels{s}, ridx(idx), v(idx), ridx(best_idx), v(best_idx));
    end
    fprintf('\n');
end
