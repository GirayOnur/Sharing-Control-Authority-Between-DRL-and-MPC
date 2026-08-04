%% plot_tv_multi.m
%
% Compares PI-ALINEA MPC, DDPG-MPC, SAC-MPC, and SAC(D)-MPC across all 5 training runs
% (50 experiments each) for all 4 scenarios.
% No Control and Hierarchical MPC are excluded.
%
% Saves: tv_multi.svg  +  tv_multi_table.tex

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

ctrl_clr = [
    0.9290, 0.6940, 0.1250;
    0.0000, 0.4470, 0.7410;
    0.8350, 0.0980, 0.1020;
    0.4940, 0.1840, 0.5560;
];

%% ── Helper ─────────────────────────────────────────────────────────────────
function tv = compute_tv(uu)
    tv = sum(vecnorm(diff(uu, 1, 2), 2, 1));
end

%% ── Load data ──────────────────────────────────────────────────────────────
tv_data      = cell(n_ctrl, n_scen);
run_idx_data = cell(n_ctrl, n_scen);

fprintf('Loading experiment data...\n');

for s = 1:n_scen
    sfx = scen_sfx{s};
    fprintf('\n  Scenario %d  (sfx = "%s")\n', s, sfx);

    %% 1. PI-ALINEA MPC
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
    tv_data{1,s}      = tv_v;
    run_idx_data{1,s} = ridx;
    fprintf('    [1] PI-ALINEA MPC  : %2d experiment(s)\n', numel(tv_v));

    %% 2. DDPG-MPC
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
    tv_data{2,s}      = tv_v;
    run_idx_data{2,s} = ridx;
    fprintf('    [2] DDPG-MPC       : %2d experiment(s)\n', numel(tv_v));

    %% 3. SAC-MPC
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
    tv_data{3,s}      = tv_v;
    run_idx_data{3,s} = ridx;
    fprintf('    [3] SAC-MPC        : %2d experiment(s)\n', numel(tv_v));

    %% 4. SAC(D)-MPC
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
    tv_data{4,s}      = tv_v;
    run_idx_data{4,s} = ridx;
    fprintf('    [4] SAC(D)-MPC   : %2d experiment(s)\n', numel(tv_v));
end
fprintf('\nAll data loaded.\n\n');

%% ── Console summary ────────────────────────────────────────────────────────
fprintf('TV summary (mean ± std):\n');
fprintf('%-18s', '');
for s = 1:n_scen, fprintf('  %-24s', scen_labels{s}); end
fprintf('\n%s\n', repmat('-', 1, 18 + 26*n_scen));
for c = 1:n_ctrl
    fprintf('%-18s', ctrl_labels_plain{c});
    for s = 1:n_scen
        v = tv_data{c,s};
        fprintf('  %-24s', sprintf('%.4f ± %.4f', mean(v), std(v)));
    end
    fprintf('\n');
end
fprintf('\n');

%% ── Figure ─────────────────────────────────────────────────────────────────
ax_fsize  = 13;
ttl_fsize = 14;

all_vals = cell2mat(tv_data(:));
y_hi = ceil( max(all_vals) * 1.08);
y_lo = -0.03 * y_hi;

fig = figure('Units', 'centimeters', 'Position', [2 2 22 16], 'Color', 'w');
tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for s = 1:n_scen
    ax = nexttile(tl);
    hold(ax, 'on');
    for ci = 1:n_ctrl
        c   = plot_ctrl_order(ci);
        v   = tv_data{c, s};
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
    ylabel(ax, '$\mathrm{TV}$', 'FontSize', ax_fsize, 'Interpreter', 'latex');
    title(ax, scen_labels{s}, 'FontSize', ttl_fsize, 'FontWeight', 'bold', 'Interpreter', 'latex');
    set(ax, 'Box', 'on', 'FontSize', ax_fsize, 'LineWidth', 0.8, ...
            'TickDir', 'out', 'TickLabelInterpreter', 'latex');
    grid(ax, 'on');
    ax.GridAlpha     = 0.12;
    ax.GridLineStyle = ':';
end

set(fig, 'Renderer', 'painters');
print(fig, fullfile(out_dir, 'tv_multi.svg'), '-dsvg', '-vector');
fprintf('Saved figure → tv_multi.svg\n');

%% ── LaTeX table ────────────────────────────────────────────────────────────
fid = fopen(fullfile(out_dir, 'tv_multi_table.tex'), 'w');
fprintf(fid, '\\begin{table}[htbp]\n\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of total input variation (TV) ' ...
    'for PI-ALINEA, DDPG-MPC, SAC-MPC, and SAC(D)-MPC across all 5 training runs.}\n']);
fprintf(fid, '\\label{tab:tv_multi}\n');
fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\n', repmat('c', 1, n_scen));
fprintf(fid, 'Controller');
for s = 1:n_scen, fprintf(fid, ' & %s', scen_labels{s}); end
fprintf(fid, ' \\\\\n\\midrule\n');
for c = 1:n_ctrl
    fprintf(fid, '%s', strrep(ctrl_labels_plain{c}, ' ', '~'));
    for s = 1:n_scen
        v = tv_data{c,s};
        fprintf(fid, ' & $%.4f \\pm %.4f$', mean(v), std(v));
    end
    fprintf(fid, ' \\\\\n');
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n\\end{table}\n');
fclose(fid);
fprintf('Saved table   → tv_multi_table.tex\n');

%% ── Median and best agent identification ───────────────────────────────────
fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('Median and best agent per algorithm and scenario\n');
fprintf('%s\n', repmat('=', 1, 70));
fprintf('%-12s  %-12s  %-26s  %-26s\n', ...
        'Algorithm', 'Scenario', 'Median agent (TV)', 'Best agent (TV)');
fprintf('%s\n', repmat('-', 1, 70));
for c = 1:n_ctrl
    for s = 1:n_scen
        v    = tv_data{c,s};
        ridx = run_idx_data{c,s};
        med  = median(v);
        [~, idx]      = min(abs(v - med));
        [~, best_idx] = min(v);
        fprintf('%-12s  %-12s  run %-2d (%.4f)        run %-2d (%.4f)\n', ...
                ctrl_labels_plain{c}, scen_labels{s}, ...
                ridx(idx), v(idx), ridx(best_idx), v(best_idx));
    end
    fprintf('\n');
end
