%% plot_dw_tot_comparison_agent.m
%
% Agent-indexed version of plot_dw_tot_comparison.m.
% Set ddpg_agent, sac_agent, and sacd_agent to select a single run (10 experiments each)
% for a fair comparison across all 6 controllers.
%
% Saves: dw_tot_comparison_agent.svg  +  dw_tot_table_agent.tex

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
plot_ctrl_order   = [1, 2, 3, 4, 6, 5];
n_runs            = 5;

T_sim   = 10/3600;           % simulation time step [h]
w_max   = [200, 100, 100];   % queue capacity [veh]: O1, O2, O3

% ── Agent selection (set to desired agent run indices) ─────────────────────
pi_alinea_agent = [3, 5, 3, 2];   % PI-ALINEA agent run index per scenario (1-5)
ddpg_agent = [3, 2, 5, 5];   % DDPG agent run index per scenario (1-5)
sac_agent  = [4, 1, 5, 3];   % SAC  agent run index per scenario (1-5)
sacd_agent = sac_agent;        % deterministic SAC run index per scenario (1-5)

%% ── Helper: compute Δw_tot from raw xx ────────────────────────────────────
function dw_tot = compute_dw_tot(xx, T, w_max)
    w_o1 = xx(64,:) + xx(65,:);
    w_o2 = xx(68,:) + xx(69,:);
    w_o3 = xx(72,:) + xx(73,:);

    dw_tot = T * ( sum(max(0, w_o1 - w_max(1))) ...
                 + sum(max(0, w_o2 - w_max(2))) ...
                 + sum(max(0, w_o3 - w_max(3))) );
end

%% ── Load data ──────────────────────────────────────────────────────────────
dw_data = cell(n_ctrl, n_scen);

fprintf('Loading experiment data (PI-ALINEA agents=%s, DDPG agents=%s, SAC agents=%s, SAC(D) agents=%s)...\n', mat2str(pi_alinea_agent), mat2str(ddpg_agent), mat2str(sac_agent), mat2str(sacd_agent));

for s = 1:n_scen
    sfx = scen_sfx{s};
    fprintf('\n  Scenario %d  (sfx = "%s")\n', s, sfx);

    %% 1. No Control
    nc_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    nc_files  = dir(fullfile(nc_folder, 'No_control_*.mat'));
    dw_v = zeros(numel(nc_files), 1);
    for f = 1:numel(nc_files)
        d = load(fullfile(nc_folder, nc_files(f).name), 'xx');
        dw_v(f) = compute_dw_tot(d.xx, T_sim, w_max);
    end
    dw_data{1,s} = dw_v;
    fprintf('    [1] No Control     : %2d experiment(s)\n', numel(dw_v));

    %% 2. PI-ALINEA MPC (single selected agent folder)
    dw_v = [];
    folder = fullfile(agents_dir, ['PI_ALINEA_MPC' sfx '_' num2str(pi_alinea_agent(s))]);
    files  = dir(fullfile(folder, 'PI_ALINEA_MPC_SR_RM*result_*.mat'));
    for f = 1:numel(files)
        d = load(fullfile(folder, files(f).name), 'xx');
        dw_v(end+1, 1) = compute_dw_tot(d.xx, T_sim, w_max);
    end
    dw_data{2,s} = dw_v;
    fprintf('    [2] PI-ALINEA MPC  : %2d experiment(s)  (agent %d)\n', numel(dw_v), pi_alinea_agent(s));

    %% 3. Hierarchical MPC
    hier_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    hier_files  = dir(fullfile(hier_folder, 'MPC_RM_SR_hier*result_*.mat'));
    dw_v = zeros(numel(hier_files), 1);
    for f = 1:numel(hier_files)
        d = load(fullfile(hier_folder, hier_files(f).name), 'xx');
        dw_v(f) = compute_dw_tot(d.xx, T_sim, w_max);
    end
    dw_data{3,s} = dw_v;
    fprintf('    [3] Hier. MPC      : %2d experiment(s)\n', numel(dw_v));

    %% 4. DDPG-MPC (single selected agent folder)
    dw_v = [];
    folder = fullfile(agents_dir, ['DDPGpi_main_s5_' num2str(ddpg_agent(s)) sfx]);
    files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
    for f = 1:numel(files)
        d = load(fullfile(folder, files(f).name), 'xx');
        dw_v(end+1, 1) = compute_dw_tot(d.xx, T_sim, w_max);
    end
    dw_data{4,s} = dw_v;
    fprintf('    [4] DDPG-MPC       : %2d experiment(s)  (agent %d)\n', numel(dw_v), ddpg_agent(s));

    %% 5. SAC-MPC (single selected agent folder)
    dw_v = [];
    folder = fullfile(agents_dir, ['SACpi_main_s5_' num2str(sac_agent(s)) sfx]);
    files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
    for f = 1:numel(files)
        d = load(fullfile(folder, files(f).name), 'xx');
        dw_v(end+1, 1) = compute_dw_tot(d.xx, T_sim, w_max);
    end
    dw_data{5,s} = dw_v;
    fprintf('    [5] SAC-MPC        : %2d experiment(s)  (agent %d)\n', numel(dw_v), sac_agent(s));

    %% 6. SAC(D)-MPC (single selected agent folder)
    dw_v = [];
    folder = fullfile(agents_dir, ['SACDpi_main_s5_' num2str(sacd_agent(s)) sfx]);
    files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
    for f = 1:numel(files)
        d = load(fullfile(folder, files(f).name), 'xx');
        dw_v(end+1, 1) = compute_dw_tot(d.xx, T_sim, w_max);
    end
    dw_data{6,s} = dw_v;
    fprintf('    [6] SAC(D)-MPC   : %2d experiment(s)  (agent %d)\n', numel(dw_v), sacd_agent(s));
end
fprintf('\nAll data loaded.\n\n');

%% ── Console summary ────────────────────────────────────────────────────────
fprintf('Δw_tot summary (mean ± std, veh·h):\n');
fprintf('%-18s', '');
for s = 1:n_scen, fprintf('  %-22s', scen_labels{s}); end
fprintf('\n%s\n', repmat('-', 1, 18 + 24*n_scen));
for c = 1:n_ctrl
    fprintf('%-18s', ctrl_labels_plain{c});
    for s = 1:n_scen
        v = dw_data{c,s};
        if numel(unique(v)) == 1
            fprintf('  %-22s', sprintf('%.3f (det.)', v(1)));
        else
            fprintf('  %-22s', sprintf('%.3f ± %.3f', mean(v), std(v)));
        end
    end
    fprintf('\n');
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

all_vals = cell2mat(dw_data(:));
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
        nc_val = mean(dw_data{1,s});
        hl = yline(ax, nc_val, '--', 'Color', ctrl_clr(1,:), 'LineWidth', 1.5, ...
                   'DisplayName', 'No Control');
        ctrl_idx = plot_ctrl_order(plot_ctrl_order ~= 1);
    else
        hl       = [];
        ctrl_idx = plot_ctrl_order;
    end

    n_boxes = numel(ctrl_idx);
    h_boxes = gobjects(n_boxes, 1);

    for ci = 1:n_boxes
        c   = ctrl_idx(ci);
        v   = dw_data{c, s};
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
    ylabel(ax, '$\Delta w_{\mathrm{tot}}$ (veh$\cdot$h)', ...
           'FontSize', ax_fsize, 'Interpreter', 'latex');
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
out_fig = fullfile(out_dir, 'dw_tot_comparison_agent.svg');
print(fig, out_fig, '-dsvg', '-vector');
fprintf('Saved figure → %s\n', out_fig);

%% ── LaTeX table ────────────────────────────────────────────────────────────
tex_file = fullfile(out_dir, 'dw_tot_table_agent.tex');
fid      = fopen(tex_file, 'w');
fprintf(fid, '\\begin{table}[htbp]\n\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of total queue ' ...
    'constraint violation ($\\Delta w_{\\mathrm{tot}}$, veh$\\cdot$h) ' ...
    'across all experimental runs. PI-ALINEA uses agent~%s; DDPG-MPC uses agent~%s; SAC-MPC uses agent~%s; SAC(D)-MPC uses agent~%s.}\n'], ...
    mat2str(pi_alinea_agent), mat2str(ddpg_agent), mat2str(sac_agent), mat2str(sacd_agent));
fprintf(fid, '\\label{tab:dw_tot_comparison_agent}\n');
fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\n', repmat('c', 1, n_scen));
fprintf(fid, 'Controller');
for s = 1:n_scen, fprintf(fid, ' & %s', scen_labels{s}); end
fprintf(fid, ' \\\\\n\\midrule\n');
for c = 1:n_ctrl
    fprintf(fid, '%s', strrep(ctrl_labels_plain{c}, ' ', '~'));
    for s = 1:n_scen
        v = dw_data{c,s};
        if numel(unique(v)) == 1
            fprintf(fid, ' & $%.3f$', v(1));
        else
            fprintf(fid, ' & $%.3f \\pm %.3f$', mean(v), std(v));
        end
    end
    fprintf(fid, ' \\\\\n');
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n\\end{table}\n');
fclose(fid);
fprintf('Saved LaTeX table → %s\n', tex_file);
