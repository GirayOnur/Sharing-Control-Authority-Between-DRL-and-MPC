%% plot_obj_comparison.m
%
% Loads raw simulation result .mat files for all 6 controllers × 4 scenarios
% and computes the soft objective cost (SOC):
%
%   SOC = u_pen + tts + q_pen
%
% where:
%   u_pen = r_cost · Σ_{k=2}^{N} Σ_{j=1}^{3} (u_j(k) − u_j(k−1))²  /  M_low
%           (action smoothness penalty over ALL 3 inputs, scaled by low-level
%            control period M_low; r_cost = 0.4, M_low = 6)
%
%   tts   = T · Σ_k [ Σ_seg (ρ_seg_c1 · λ_seg) · L_m + Σ_o w_o_c1 ]
%         + T · Σ_k [ Σ_seg (ρ_seg_c2 · λ_seg) · L_m + Σ_o w_o_c2 ]
%
%   q_pen = Σ_k [ max(0, w_O1(k) − w_O1^max)²
%               + max(0, w_O2(k) − w_O2^max)²
%               + max(0, w_O3(k) − w_O3^max)² ]
%           (soft quadratic queue constraint penalty; w_max = [200,100,100])
%
% Cost formula matches bayes_cost_PI_ALINEA_MPC.m with the extension that
% u_pen now includes input 1 (uu(1,:)) in addition to inputs 2 and 3.
%
% Generates a 2×2 box-plot figure (one subplot per scenario):
%   S1/S3 - No Control shown as dashed reference line (deterministic)
%   S2/S4 - No Control shown as a box (stochastic)
%
% Saves: obj_comparison.svg  +  obj_table.tex

clear; clc; close all;

%% ── Paths ─────────────────────────────────────────────────────────────────
script_dir = fileparts(mfilename('fullpath'));
agents_dir = fullfile(script_dir, '..', '..', '..');
out_dir    = script_dir;

%% ── Definitions ───────────────────────────────────────────────────────────
scen_labels = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4'};
scen_sfx    = {'', '_nd', '_mm', '_mm_nd'};
n_scen      = 4;

ctrl_labels       = {'No Control', 'SF-MPC', 'Hier.\ MPC', 'DDPG-MPC', 'SAC-MPC', 'SACD-MPC'};
ctrl_labels_plain = {'No Control', 'SF-MPC', 'Hier. MPC',  'DDPG-MPC', 'SAC-MPC', 'SACD-MPC'};
n_ctrl            = 6;
plot_ctrl_order   = [1, 2, 3, 4, 6, 5];
n_runs            = 5;

% Cost parameters (from param_MPC_get(1) and param_get)
r_cost = 0.4;    % action penalty weight
M_low  = 6;      % low-level control period [sim steps], scales u_pen

%% ── Helper: compute SOC from raw xx, uu, param_sim ────────────────────────
function obj = compute_obj(xx, uu, param_sim, r_cost, M_low)
    % ── u_pen: action smoothness penalty over ALL 3 inputs ─────────────────
    % Scaled by M_low because uu stores the action at every simulation step,
    % not only at each low-level control step.
    du   = uu(1:3, 2:end) - uu(1:3, 1:end-1);   % [3 × (N-1)]
    u_pen = sum(r_cost .* sum(du.^2, 1)) ./ M_low;

    % ── tts: Total Time Spent ───────────────────────────────────────────────
    idx_c1    = [3, 10, 17, 24, 31, 38, 45, 52, 59];
    idx_c2    = [4, 11, 18, 25, 32, 39, 46, 53, 60];
    lambda    = [param_sim.lambda.l1, param_sim.lambda.l2, param_sim.lambda.l3, ...
                 param_sim.lambda.l4, param_sim.lambda.l5, param_sim.lambda.l6, ...
                 param_sim.lambda.l7, param_sim.lambda.l8, param_sim.lambda.l9];
    tts_c1 = param_sim.T .* (sum(xx(idx_c1,:) .* lambda', 1) .* param_sim.L_m ...
                              + xx(64,:) + xx(68,:) + xx(72,:));
    tts_c2 = param_sim.T .* (sum(xx(idx_c2,:) .* lambda', 1) .* param_sim.L_m ...
                              + xx(65,:) + xx(69,:) + xx(73,:));
    tts = sum(tts_c1 + tts_c2);

    % ── q_pen: soft quadratic queue constraint penalty ──────────────────────
    w_o1  = xx(64,:) + xx(65,:);
    w_o2  = xx(68,:) + xx(69,:);
    w_o3  = xx(72,:) + xx(73,:);
    q_pen = sum( max(0, w_o1 - param_sim.w_con(1)).^2 ...
               + max(0, w_o2 - param_sim.w_con(2)).^2 ...
               + max(0, w_o3 - param_sim.w_con(3)).^2 );

    obj = u_pen + tts + q_pen;
end

%% ── Load data ──────────────────────────────────────────────────────────────
obj_data     = cell(n_ctrl, n_scen);
run_idx_data = cell(n_ctrl, n_scen);

fprintf('Loading experiment data...\n');

for s = 1:n_scen
    sfx = scen_sfx{s};
    fprintf('\n  Scenario %d  (sfx = "%s")\n', s, sfx);

    %% 1. No Control
    nc_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    nc_files  = dir(fullfile(nc_folder, 'No_control_*.mat'));
    obj_v = zeros(numel(nc_files), 1);
    for f = 1:numel(nc_files)
        d = load(fullfile(nc_folder, nc_files(f).name), 'xx', 'uu', 'param_sim');
        obj_v(f) = compute_obj(d.xx, d.uu, d.param_sim, r_cost, M_low);
    end
    obj_data{1,s} = obj_v;
    fprintf('    [1] No Control     : %2d experiment(s)\n', numel(obj_v));

    %% 2. PI-ALINEA MPC
    obj_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['PI_ALINEA_MPC' sfx '_' num2str(r)]);
        files  = dir(fullfile(folder, 'PI_ALINEA_MPC_SR_RM*result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'uu', 'param_sim');
            obj_v(end+1, 1) = compute_obj(d.xx, d.uu, d.param_sim, r_cost, M_low);
            ridx(end+1, 1)  = r;
        end
    end
    obj_data{2,s}     = obj_v;
    run_idx_data{2,s} = ridx;
    fprintf('    [2] PI-ALINEA MPC  : %2d experiment(s)\n', numel(obj_v));

    %% 3. Hierarchical MPC
    hier_folder = fullfile(agents_dir, ['DDPGpi_main_s5_1' sfx]);
    hier_files  = dir(fullfile(hier_folder, 'MPC_RM_SR_hier*result_*.mat'));
    obj_v = zeros(numel(hier_files), 1);
    for f = 1:numel(hier_files)
        d = load(fullfile(hier_folder, hier_files(f).name), 'xx', 'uu', 'param_sim');
        obj_v(f) = compute_obj(d.xx, d.uu, d.param_sim, r_cost, M_low);
    end
    obj_data{3,s} = obj_v;
    fprintf('    [3] Hier. MPC      : %2d experiment(s)\n', numel(obj_v));

    %% 4. DDPG-MPC
    obj_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['DDPGpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'uu', 'param_sim');
            obj_v(end+1, 1) = compute_obj(d.xx, d.uu, d.param_sim, r_cost, M_low);
            ridx(end+1, 1)  = r;
        end
    end
    obj_data{4,s}     = obj_v;
    run_idx_data{4,s} = ridx;
    fprintf('    [4] DDPG-MPC       : %2d experiment(s)\n', numel(obj_v));

    %% 5. SAC-MPC
    obj_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'uu', 'param_sim');
            obj_v(end+1, 1) = compute_obj(d.xx, d.uu, d.param_sim, r_cost, M_low);
            ridx(end+1, 1)  = r;
        end
    end
    obj_data{5,s}     = obj_v;
    run_idx_data{5,s} = ridx;
    fprintf('    [5] SAC-MPC        : %2d experiment(s)\n', numel(obj_v));

    %% 6. SACD-MPC
    obj_v = []; ridx = [];
    for r = 1:n_runs
        folder = fullfile(agents_dir, ['SACDpi_main_s5_' num2str(r) sfx]);
        files  = dir(fullfile(folder, 'RL_MPC_SR_RM_result_*.mat'));
        for f = 1:numel(files)
            d = load(fullfile(folder, files(f).name), 'xx', 'uu', 'param_sim');
            obj_v(end+1, 1) = compute_obj(d.xx, d.uu, d.param_sim, r_cost, M_low);
            ridx(end+1, 1)  = r;
        end
    end
    obj_data{6,s}     = obj_v;
    run_idx_data{6,s} = ridx;
    fprintf('    [6] SACD-MPC   : %2d experiment(s)\n', numel(obj_v));
end
fprintf('\nAll data loaded.\n\n');

%% ── Console summary ────────────────────────────────────────────────────────
fprintf('SOC summary (mean ± std):\n');
fprintf('%-18s', '');
for s = 1:n_scen, fprintf('  %-24s', scen_labels{s}); end
fprintf('\n%s\n', repmat('-', 1, 18 + 26*n_scen));
for c = 1:n_ctrl
    fprintf('%-18s', ctrl_labels_plain{c});
    for s = 1:n_scen
        v = obj_data{c,s};
        if numel(unique(v)) == 1
            fprintf('  %-24s', sprintf('%.2f (det.)', v(1)));
        else
            fprintf('  %-24s', sprintf('%.2f ± %.2f', mean(v), std(v)));
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
    0.4940, 0.1840, 0.5560;   % purple - SACD-MPC
];

all_vals = cell2mat(obj_data(:));
y_hi = ceil( max(all_vals) * 1.08);
y_lo = -0.03 * y_hi;

%% ── 2×2 box-plot figure ───────────────────────────────────────────────────
fig = figure('Units', 'centimeters', 'Position', [2 2 22 16], 'Color', 'w');
tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Scenario-specific zoom insets. S1/S2 show all displayed controllers because
% the shared y-axis is driven by larger multi-merge costs; S3 omits SAC-MPC
% in the inset so the remaining methods can be compared.
zoom_ctrl_groups = cell(1, n_scen);
zoom_ctrl_groups{1} = plot_ctrl_order(plot_ctrl_order ~= 1);
zoom_ctrl_groups{2} = plot_ctrl_order;
zoom_ctrl_groups{3} = plot_ctrl_order(~ismember(plot_ctrl_order, [1, 5]));
zoom_ctrl_groups{4} = [];
zoom_include_nc_line = [true, false, true, false];

for s = 1:n_scen
    ax = nexttile(tl);
    hold(ax, 'on');

    det_nc = (s == 1 || s == 3);

    if det_nc
        nc_val = mean(obj_data{1,s});
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
        v   = obj_data{c, s};
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
    ylabel(ax, 'SOC', 'FontSize', ax_fsize, 'Interpreter', 'latex', 'Color', 'k');
    title(ax, scen_labels{s}, 'FontSize', ttl_fsize, ...
          'FontWeight', 'bold', 'Interpreter', 'latex', 'Color', 'k');
    set(ax, 'Box', 'on', 'FontSize', ax_fsize, 'LineWidth', 0.8, ...
            'TickDir', 'out', 'TickLabelInterpreter', 'latex', ...
            'Color', 'w', 'XColor', 'k', 'YColor', 'k');
    grid(ax, 'on');
    ax.GridAlpha     = 0.12;
    ax.GridLineStyle = ':';

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

    add_obj_zoom_inset(ax, obj_data, s, ctrl_idx, zoom_ctrl_groups{s}, ...
                       zoom_include_nc_line(s), ctrl_clr, ctrl_labels, ...
                       ax_fsize);
end

set(fig, 'Renderer', 'painters');
fig.Units = 'centimeters';
fig_pos = fig.Position;
set(fig, 'PaperUnits', 'centimeters', ...
         'PaperSize', fig_pos(3:4), ...
         'PaperPosition', [0 0 fig_pos(3) fig_pos(4)], ...
         'PaperPositionMode', 'manual');
out_fig = fullfile(out_dir, 'obj_comparison.svg');
print(fig, out_fig, '-dsvg', '-vector');
fprintf('Saved figure → %s\n', out_fig);
out_pdf = fullfile(out_dir, 'obj_comparison.pdf');
print(fig, out_pdf, '-dpdf', '-vector');
fprintf('Saved PDF figure -> %s\n', out_pdf);

%% ── LaTeX table ────────────────────────────────────────────────────────────
tex_file = fullfile(out_dir, 'obj_table.tex');
fid      = fopen(tex_file, 'w');
fprintf(fid, '\\begin{table}[htbp]\n\\centering\n');
fprintf(fid, ['\\caption{Mean $\\pm$ standard deviation of the soft objective ' ...
    'cost (SOC = $u_{\\mathrm{pen}} + \\mathrm{TTS} + q_{\\mathrm{pen}}$) ' ...
    'across all experimental runs.}\n']);
fprintf(fid, '\\label{tab:obj_comparison}\n');
fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\n', repmat('c', 1, n_scen));
fprintf(fid, 'Controller');
for s = 1:n_scen, fprintf(fid, ' & %s', scen_labels{s}); end
fprintf(fid, ' \\\\\n\\midrule\n');
for c = 1:n_ctrl
    fprintf(fid, '%s', strrep(ctrl_labels_plain{c}, ' ', '~'));
    for s = 1:n_scen
        v = obj_data{c,s};
        if numel(unique(v)) == 1
            fprintf(fid, ' & $%.2f$', v(1));
        else
            fprintf(fid, ' & $%.2f \\pm %.2f$', mean(v), std(v));
        end
    end
    fprintf(fid, ' \\\\\n');
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n\\end{table}\n');
fclose(fid);
fprintf('Saved LaTeX table → %s\n', tex_file);

%% ── Median-agent identification for DDPG and SAC ──────────────────────────
fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('Median-representative agent (SOC)\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('%-10s  %-12s  %-8s  %-12s  %s\n', ...
        'Algorithm', 'Scenario', 'Agent', 'SOC(med)', 'Median of all');
fprintf('%s\n', repmat('-', 1, 60));
drl_ctrl  = {2, 4, 5, 6};
drl_names = {'SF-MPC', 'DDPG-MPC', 'SAC-MPC', 'SACD-MPC'};

for di = 1:numel(drl_ctrl)
    c     = drl_ctrl{di};
    for s = 1:n_scen
        v    = obj_data{c,s};
        ridx = run_idx_data{c,s};
        med  = median(v);
        [~, idx]      = min(abs(v - med));
        [~, best_idx] = min(v);
        fprintf('%-10s  %-12s  med: run %-2d (%.2f)   best: run %-2d (%.2f)\n', ...
                drl_names{di}, scen_labels{s}, ridx(idx), v(idx), ridx(best_idx), v(best_idx));
    end
    fprintf('\n');
end

function add_obj_zoom_inset(ax, obj_data, scenario_idx, ctrl_idx, zoom_ctrl_idx, ...
                            include_nc_line, ctrl_clr, ctrl_labels, ax_fsize)
    zoom_ctrl_idx = zoom_ctrl_idx(:)';
    zoom_ctrl_idx = zoom_ctrl_idx(ismember(zoom_ctrl_idx, ctrl_idx));
    if isempty(zoom_ctrl_idx)
        return;
    end

    zoom_vals = [];
    for c = zoom_ctrl_idx
        zoom_vals = [zoom_vals; finite_column(obj_data{c, scenario_idx})]; %#ok<AGROW>
    end
    if include_nc_line
        zoom_vals = [zoom_vals; finite_column(obj_data{1, scenario_idx})]; %#ok<AGROW>
    end
    if isempty(zoom_vals)
        return;
    end

    zoom_lo = min(zoom_vals);
    zoom_hi = max(zoom_vals);
    span = zoom_hi - zoom_lo;
    if span <= 0
        pad = max(0.08 * max(abs(zoom_hi), 1), eps);
    else
        pad = 0.08 * span;
    end
    zoom_ylim = [max(0, zoom_lo - pad), zoom_hi + pad];
    if zoom_ylim(1) >= zoom_ylim(2)
        zoom_ylim = [0, max(1, zoom_hi * 1.08)];
    end
    if scenario_idx == 1 || scenario_idx == 3
        zoom_ylim(2) = 1e7;
    end

    drawnow limitrate;
    old_units = ax.Units;
    ax.Units = 'normalized';
    ax_pos = ax.Position;
    ax.Units = old_units;

    if scenario_idx == 3
        inset_pos = [ax_pos(1) + 0.08 * ax_pos(3), ...
                     ax_pos(2) + 0.34 * ax_pos(4), ...
                     0.68 * ax_pos(3), ...
                     0.54 * ax_pos(4)];
    elseif scenario_idx == 4
        inset_pos = [ax_pos(1) + 0.08 * ax_pos(3), ...
                     ax_pos(2) + 0.54 * ax_pos(4), ...
                     0.68 * ax_pos(3), ...
                     0.34 * ax_pos(4)];
    else
        inset_pos = [ax_pos(1) + 0.12 * ax_pos(3), ...
                     ax_pos(2) + 0.48 * ax_pos(4), ...
                     0.76 * ax_pos(3), ...
                     0.46 * ax_pos(4)];
    end
    inset_ax = axes('Parent', ancestor(ax, 'figure'), ...
                    'Units', 'normalized', ...
                    'Position', inset_pos);
    hold(inset_ax, 'on');

    zoom_pos = zeros(1, numel(zoom_ctrl_idx));
    for i = 1:numel(zoom_ctrl_idx)
        c = zoom_ctrl_idx(i);
        x_pos = find(ctrl_idx == c, 1);
        zoom_pos(i) = x_pos;

        v = finite_column(obj_data{c, scenario_idx});
        if numel(v) < 4
            v = repmat(v, 4, 1);
        end

        bc = boxchart(inset_ax, x_pos * ones(size(v)), v);
        bc.BoxFaceColor     = ctrl_clr(c, :);
        bc.BoxFaceAlpha     = 0.70;
        bc.WhiskerLineColor = ctrl_clr(c, :);
        bc.WhiskerLineStyle = '-';
        bc.LineWidth        = 1.1;
        bc.MarkerStyle      = '+';
        bc.MarkerColor      = ctrl_clr(c, :) * 0.75;
        bc.MarkerSize       = 3;
        bc.DisplayName      = ctrl_labels{c};
        bc.HandleVisibility = 'off';
    end

    if include_nc_line
        nc_vals = finite_column(obj_data{1, scenario_idx});
        if ~isempty(nc_vals)
            yline(inset_ax, mean(nc_vals), '--', 'Color', ctrl_clr(1, :), ...
                  'LineWidth', 0.9, 'HandleVisibility', 'off');
        end
    end

    hold(inset_ax, 'off');
    xlim(inset_ax, [min(zoom_pos) - 0.5, max(zoom_pos) + 0.5]);
    ylim(inset_ax, zoom_ylim);
    inset_ax.XTick = zoom_pos;
    inset_ax.XTickLabel = [];
    set(inset_ax, 'Box', 'on', ...
                  'FontSize', max(ax_fsize - 5, 6), ...
                  'LineWidth', 0.6, ...
                  'TickDir', 'out', ...
                  'TickLabelInterpreter', 'latex', ...
                  'Color', 'w', 'XColor', 'k', 'YColor', 'k');
    grid(inset_ax, 'on');
    inset_ax.GridAlpha = 0.12;
    inset_ax.GridLineStyle = ':';
end

function v = finite_column(v)
    v = v(:);
    v = v(isfinite(v));
end
