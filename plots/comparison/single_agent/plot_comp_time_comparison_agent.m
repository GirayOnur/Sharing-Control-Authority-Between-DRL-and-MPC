%% plot_comp_time_comparison_agent.m
%
% Agent-indexed computation-time version of the comparison plotters.
% Set pi_alinea_agent, ddpg_agent, sac_agent, and sacd_agent to select a
% single run folder per scenario.
%
% Saves:
%   comp_time_comparison_agent.svg
%   comp_time_table_agent.tex

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
plot_ctrl_order   = [1, 2, 3, 4, 6, 5];

% Agent selection, aligned with the existing single-agent comparison scripts.
pi_alinea_agent = [3, 5, 3, 2];
ddpg_agent      = [3, 2, 5, 5];
sac_agent       = [4, 1, 5, 3];
sacd_agent      = sac_agent;

%% Load all experiment data
time_data = cell(n_ctrl, n_scen);

fprintf(['Loading computation-time data (PI-ALINEA agents=%s, DDPG agents=%s, ' ...
    'SAC agents=%s, SAC(D) agents=%s)...\n'], ...
    mat2str(pi_alinea_agent), mat2str(ddpg_agent), mat2str(sac_agent), mat2str(sacd_agent));

for s = 1:n_scen
    sfx = scen_sfx{s};
    fprintf('\n  Scenario %d  (sfx = "%s")\n', s, sfx);

    % 1. No Control
    nc_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    time_data{1,s} = read_no_control_times(nc_folder, 'No_control_*.mat');
    fprintf('    [1] No Control     : %2d experiment(s)\n', numel(time_data{1,s}));

    % 2. PI-ALINEA MPC
    folder = fullfile(agents_dir, ['PI_ALINEA_MPC' sfx '_' num2str(pi_alinea_agent(s))]);
    time_data{2,s} = read_computation_times(folder, 'PI_ALINEA_MPC_SR_RM*result_*.mat');
    fprintf('    [2] PI-ALINEA MPC  : %2d experiment(s)  (agent %d)\n', ...
            numel(time_data{2,s}), pi_alinea_agent(s));

    % 3. Hierarchical MPC
    hier_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    time_data{3,s} = read_computation_times(hier_folder, 'MPC_RM_SR_hier*result_*.mat');
    fprintf('    [3] Hier. MPC      : %2d experiment(s)\n', numel(time_data{3,s}));

    % 4. DDPG-MPC
    folder = fullfile(agents_dir, ['DDPGpi_main_s5_' num2str(ddpg_agent(s)) sfx]);
    time_data{4,s} = read_computation_times(folder, 'RL_MPC_SR_RM_result_*.mat');
    fprintf('    [4] DDPG-MPC       : %2d experiment(s)  (agent %d)\n', ...
            numel(time_data{4,s}), ddpg_agent(s));

    % 5. SAC-MPC
    folder = fullfile(agents_dir, ['SACpi_main_s5_' num2str(sac_agent(s)) sfx]);
    time_data{5,s} = read_computation_times(folder, 'RL_MPC_SR_RM_result_*.mat');
    fprintf('    [5] SAC-MPC        : %2d experiment(s)  (agent %d)\n', ...
            numel(time_data{5,s}), sac_agent(s));

    % 6. SAC(D)-MPC
    folder = fullfile(agents_dir, ['SACDpi_main_s5_' num2str(sacd_agent(s)) sfx]);
    time_data{6,s} = read_computation_times(folder, 'RL_MPC_SR_RM_result_*.mat');
    fprintf('    [6] SAC(D)-MPC     : %2d experiment(s)  (agent %d)\n', ...
            numel(time_data{6,s}), sacd_agent(s));
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
fig = make_boxplot_figure(time_data, ctrl_clr, ctrl_labels, plot_ctrl_order, ...
                          scen_labels, y_hi, ax_fsize, ttl_fsize);
set(fig, 'Renderer', 'painters');
out_fig = fullfile(out_dir, 'comp_time_comparison_agent.svg');
print(fig, out_fig, '-dsvg', '-vector');
fprintf('Saved figure -> %s\n', out_fig);

%% LaTeX table
tex_file = fullfile(out_dir, 'comp_time_table_agent.tex');
fid = fopen(tex_file, 'w');

fprintf(fid, '\\begin{table}[htbp]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of total controller ' ...
    'computation time (s) for each controller and scenario. PI-ALINEA uses ' ...
    'agent~%s; DDPG-MPC uses agent~%s; SAC-MPC uses agent~%s; SAC(D)-MPC ' ...
    'uses agent~%s.}\n'], mat2str(pi_alinea_agent), mat2str(ddpg_agent), ...
    mat2str(sac_agent), mat2str(sacd_agent));
fprintf(fid, '\\label{tab:comp_time_comparison_agent}\n');
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

function fig = make_boxplot_figure(time_data, ctrl_clr, ctrl_labels, plot_ctrl_order, ...
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
            ctrl_idx = plot_ctrl_order(plot_ctrl_order ~= 1);
        else
            hl = [];
            ctrl_idx = plot_ctrl_order;
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
