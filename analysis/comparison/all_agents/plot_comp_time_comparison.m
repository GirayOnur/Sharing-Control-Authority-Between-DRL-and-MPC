%% plot_comp_time_comparison.m
%
% Loads total controller computation time from saved experiment .mat files
% for all 6 controllers and 4 scenarios, then generates:
%
%   comp_time_comparison.svg
%   comp_time_table.tex
%
% Computation time is read from the saved variable total_comp_time.
% No Control runs do not perform controller optimization in the current
% benchmark scripts, so they are shown as 0 s.

clear; clc; close all;

%% Paths
script_dir = fileparts(mfilename('fullpath'));
agents_dir = fullfile(script_dir, '..', '..', '..');
out_dir    = script_dir;

if ~exist(out_dir, 'dir'), mkdir(out_dir); end

%% Definitions
scen_labels = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4'};
scen_sfx    = {'', '_nd', '_mm', '_mm_nd'};
n_scen      = 4;

ctrl_labels       = {'No Control', 'PI-ALINEA', 'Hier.\ MPC', 'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
ctrl_labels_plain = {'No Control', 'PI-ALINEA', 'Hier. MPC',  'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
n_ctrl            = 6;
n_runs            = 5;

%% Load all experiment data
time_data    = cell(n_ctrl, n_scen);
run_idx_data = cell(n_ctrl, n_scen);

fprintf('Loading computation-time data...\n');

for s = 1:n_scen
    sfx = scen_sfx{s};
    fprintf('\n  Scenario %d  (sfx = "%s")\n', s, sfx);

    % 1. No Control
    nc_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    time_data{1,s} = read_no_control_times(nc_folder, 'No_control_*.mat');
    fprintf('    [1] No Control     : %2d experiment(s)\n', numel(time_data{1,s}));

    % 2. PI-ALINEA MPC
    t_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['PI_ALINEA_MPC' sfx '_' num2str(r)]);
        v = read_computation_times(folder, 'PI_ALINEA_MPC_SR_RM*result_*.mat');
        t_v = [t_v; v];
        ridx = [ridx; r * ones(numel(v), 1)];
    end
    time_data{2,s}    = t_v;
    run_idx_data{2,s} = ridx;
    fprintf('    [2] PI-ALINEA MPC  : %2d experiment(s)\n', numel(t_v));

    % 3. Hierarchical MPC
    hier_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    time_data{3,s} = read_computation_times(hier_folder, 'MPC_RM_SR_hier*result_*.mat');
    fprintf('    [3] Hier. MPC      : %2d experiment(s)\n', numel(time_data{3,s}));

    % 4. DDPG-MPC
    t_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['DDPGpi_main_s5_' num2str(r) sfx]);
        v = read_computation_times(folder, 'RL_MPC_SR_RM_result_*.mat');
        t_v = [t_v; v];
        ridx = [ridx; r * ones(numel(v), 1)];
    end
    time_data{4,s}    = t_v;
    run_idx_data{4,s} = ridx;
    fprintf('    [4] DDPG-MPC       : %2d experiment(s)\n', numel(t_v));

    % 5. SAC-MPC
    t_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACpi_main_s5_' num2str(r) sfx]);
        v = read_computation_times(folder, 'RL_MPC_SR_RM_result_*.mat');
        t_v = [t_v; v];
        ridx = [ridx; r * ones(numel(v), 1)];
    end
    time_data{5,s}    = t_v;
    run_idx_data{5,s} = ridx;
    fprintf('    [5] SAC-MPC        : %2d experiment(s)\n', numel(t_v));

    % 6. SAC(D)-MPC
    t_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACDpi_main_s5_' num2str(r) sfx]);
        v = read_computation_times(folder, 'RL_MPC_SR_RM_result_*.mat');
        t_v = [t_v; v];
        ridx = [ridx; r * ones(numel(v), 1)];
    end
    time_data{6,s}    = t_v;
    run_idx_data{6,s} = ridx;
    fprintf('    [6] SAC(D)-MPC     : %2d experiment(s)\n', numel(t_v));
end
fprintf('\nAll computation-time data loaded.\n\n');

%% Console summary
fprintf('Computation-time summary (mean +/- std, s):\n');
fprintf('%-18s', '');
for s = 1:n_scen, fprintf('  %-24s', scen_labels{s}); end
fprintf('\n%s\n', repmat('-', 1, 18 + 26*n_scen));

for c = 1:n_ctrl
    fprintf('%-18s', ctrl_labels_plain{c});
    for s = 1:n_scen
        fprintf('  %-24s', summary_text(time_data{c,s}));
    end
    fprintf('\n');
end
fprintf('\n');

%% Visual style
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

mask     = ~cellfun(@isempty, time_data);
all_vals = vertcat(time_data{mask});
y_hi     = upper_axis_limit(all_vals);

%% Figure
fig = make_boxplot_figure(time_data, ctrl_clr, ctrl_labels, ...
                          scen_labels, y_hi, ax_fsize, ttl_fsize);
set(fig, 'Renderer', 'painters');
out_fig = fullfile(out_dir, 'comp_time_comparison.svg');
print(fig, out_fig, '-dsvg', '-vector');
fprintf('Saved figure -> %s\n', out_fig);

%% LaTeX table
tex_file = fullfile(out_dir, 'comp_time_table.tex');
fid = fopen(tex_file, 'w');

fprintf(fid, '\\begin{table}[htbp]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of total controller ' ...
    'computation time (s) across all experimental runs for each controller ' ...
    'and scenario.}\n']);
fprintf(fid, '\\label{tab:comp_time_comparison}\n');
fprintf(fid, '\\begin{tabular}{l%s}\n', repmat('c', 1, n_scen));
fprintf(fid, '\\toprule\n');
fprintf(fid, 'Controller');
for s = 1:n_scen, fprintf(fid, ' & %s', scen_labels{s}); end
fprintf(fid, ' \\\\\n\\midrule\n');

for c = 1:n_ctrl
    fprintf(fid, '%s', strrep(ctrl_labels_plain{c}, ' ', '~'));
    for s = 1:n_scen
        write_table_value(fid, time_data{c,s});
    end
    fprintf(fid, ' \\\\\n');
end

fprintf(fid, '\\bottomrule\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n');
fclose(fid);
fprintf('Saved LaTeX table -> %s\n', tex_file);

%% Median and fastest agent identification
fprintf('\n%s\n', repmat('=', 1, 72));
fprintf('Median-representative and fastest agent per algorithm and scenario\n');
fprintf('%s\n', repmat('=', 1, 72));
fprintf('%-12s  %-12s  %-24s  %-24s\n', ...
        'Algorithm', 'Scenario', 'Median agent (s)', 'Fastest agent (s)');
fprintf('%s\n', repmat('-', 1, 72));

drl_ctrl  = [2, 4, 5, 6];
drl_names = {'PI-ALINEA', 'DDPG-MPC', 'SAC-MPC', 'SAC(D)-MPC'};
for di = 1:numel(drl_ctrl)
    c = drl_ctrl(di);
    for s = 1:n_scen
        v = time_data{c,s};
        ridx = run_idx_data{c,s};
        if isempty(v)
            fprintf('%-12s  %-12s  %-24s  %-24s\n', drl_names{di}, scen_labels{s}, 'no data', 'no data');
            continue;
        end
        med = median(v);
        [~, idx] = min(abs(v - med));
        [~, fast_idx] = min(v);
        fprintf('%-12s  %-12s  run %-2d (%8.2f)       run %-2d (%8.2f)\n', ...
                drl_names{di}, scen_labels{s}, ...
                ridx(idx), v(idx), ridx(fast_idx), v(fast_idx));
    end
    fprintf('\n');
end

%% Local functions
function times = read_computation_times(folder, pattern)
    files = sorted_dir(fullfile(folder, pattern));
    times = zeros(numel(files), 1);
    for f = 1:numel(files)
        times(f) = read_computation_time(fullfile(folder, files(f).name));
    end
end

function times = read_no_control_times(folder, pattern)
    files = sorted_dir(fullfile(folder, pattern));
    times = zeros(numel(files), 1);
end

function t = read_computation_time(mat_file)
    d = load(mat_file, 'total_comp_time');
    if ~isfield(d, 'total_comp_time')
        error('No computation-time variable found in "%s".', mat_file);
    end
    t = double(d.total_comp_time);
    t = t(:);
    if isempty(t)
        error('Computation-time variable total_comp_time is empty in "%s".', mat_file);
    end
    t = t(end);
end

function files = sorted_dir(pattern)
    files = dir(pattern);
    if isempty(files), return; end
    [~, idx] = sort({files.name});
    files = files(idx);
end

function txt = summary_text(v)
    if isempty(v)
        txt = 'no data';
    elseif numel(unique(v)) == 1
        txt = sprintf('%.1f', v(1));
    else
        txt = sprintf('%.1f +/- %.1f', mean(v), std(v));
    end
end

function write_table_value(fid, v)
    if isempty(v)
        fprintf(fid, ' & --');
    elseif numel(unique(v)) == 1
        fprintf(fid, ' & $%.1f$', v(1));
    else
        fprintf(fid, ' & $%.1f \\pm %.1f$', mean(v), std(v));
    end
end

function y_hi = upper_axis_limit(vals)
    max_val = max(vals);
    if isempty(max_val) || max_val <= 0
        y_hi = 1;
        return;
    end
    base = 10 ^ floor(log10(max_val));
    step = base / 2;
    y_hi = ceil(1.08 * max_val / step) * step;
end

function fig = make_boxplot_figure(time_data, ctrl_clr, ctrl_labels, ...
                                   scen_labels, y_hi, ax_fsize, ttl_fsize)
    n_ctrl = size(ctrl_clr, 1);
    n_scen = numel(scen_labels);
    nc_clr = ctrl_clr(1, :);

    fig = figure('Units', 'centimeters', 'Position', [2 2 22 16], 'Color', 'w');
    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for s = 1:n_scen
        ax = nexttile(tl);
        hold(ax, 'on');

        det_nc = (s == 1 || s == 3);
        if det_nc
            hl = yline(ax, mean(time_data{1,s}), '--', 'Color', nc_clr, ...
                       'LineWidth', 1.5, 'DisplayName', 'No Control');
            ctrl_idx = 2:n_ctrl;
        else
            hl = [];
            ctrl_idx = 1:n_ctrl;
        end

        h_boxes = gobjects(0);
        for ci = 1:numel(ctrl_idx)
            c = ctrl_idx(ci);
            v = time_data{c,s};
            if isempty(v), continue; end

            if numel(v) < 4
                v = repmat(v, 4, 1);
            end

            clr = ctrl_clr(c, :);
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
            h_boxes(end+1)      = bc; %#ok<AGROW>
        end

        hold(ax, 'off');
        ylim(ax, [0, y_hi]);
        ax.XTick = 1:numel(ctrl_idx);
        ax.XTickLabel = ctrl_labels(ctrl_idx);
        ax.XTickLabelRotation = 22;
        ylabel(ax, 'Computation time (s)', 'FontSize', ax_fsize, 'Interpreter', 'latex');
        title(ax, scen_labels{s}, 'FontSize', ttl_fsize, 'FontWeight', 'bold', 'Interpreter', 'latex');
        set(ax, 'Box', 'on', 'FontSize', ax_fsize, 'LineWidth', 0.8, ...
                'TickDir', 'out', 'TickLabelInterpreter', 'latex');
        grid(ax, 'on');
        ax.GridAlpha = 0.12;
        ax.GridLineStyle = ':';

        if det_nc
            set(h_boxes, 'HandleVisibility', 'off');
            legend(ax, hl, 'Location', 'northeast', ...
                   'FontSize', ax_fsize - 1, 'Interpreter', 'latex', 'Box', 'off');
        end
    end
end
