function metric_data = plot_extra_experiment_metric(metric_name)
%PLOT_EXTRA_EXPERIMENT_METRIC Plot nominal-trained DDPG robustness metrics.
%
% Compares the same nominal-demand-trained DDPG agents in their nominal
% demand environment and in the corresponding noisy-demand environment.

if nargin ~= 1
    error('Call plot_extra_experiment_metric with one metric name.');
end

metric_name = lower(strtrim(metric_name));
cfg = metricConfig(metric_name);

script_dir = fileparts(mfilename('fullpath'));
agents_dir = fullfile(script_dir, '..', '..', '..');
out_dir = script_dir;
n_runs = 5;

case_defs = experimentCases();
method_labels = {'Nominal demand', 'Noisy demand'};
method_labels_plain = {'Nominal demand', 'Noisy demand'};
n_cases = numel(case_defs);
n_methods = numel(method_labels);

values = cell(n_methods, n_cases);
run_idx = cell(n_methods, n_cases);

fprintf('Loading %s data for extra DDPG experiments...\n', cfg.display_name);

for case_idx = 1:n_cases
    fprintf('\n  %s\n', case_defs(case_idx).label);

    for method_idx = 1:n_methods
        folder_pattern = case_defs(case_idx).folder_patterns{method_idx};
        [values{method_idx, case_idx}, run_idx{method_idx, case_idx}] = ...
            loadMetricGroup(agents_dir, folder_pattern, n_runs, cfg);

        fprintf('    %-16s: %2d experiment(s)\n', ...
            method_labels_plain{method_idx}, numel(values{method_idx, case_idx}));
    end
end

fprintf('\n%s summary:\n', cfg.display_name);
for case_idx = 1:n_cases
    nominal_v = values{1, case_idx};
    noisy_v = values{2, case_idx};
    delta_pct = 100 * (mean(noisy_v) - mean(nominal_v)) / mean(nominal_v);

    fprintf('  %s\n', case_defs(case_idx).label);
    fprintf('    %-16s %s\n', method_labels_plain{1}, summaryText(nominal_v, cfg.value_format));
    fprintf('    %-16s %s  (%+.2f%% vs nominal demand)\n', ...
        method_labels_plain{2}, summaryText(noisy_v, cfg.value_format), delta_pct);
end
fprintf('\n');

fig = plotMetricFigure(values, cfg, case_defs, method_labels);
set(fig, 'Renderer', 'painters');

svg_path = fullfile(out_dir, [cfg.file_prefix '_comparison.svg']);
pdf_path = fullfile(out_dir, [cfg.file_prefix '_comparison.pdf']);
print(fig, svg_path, '-dsvg', '-vector');
print(fig, pdf_path, '-dpdf', '-vector');
fprintf('Saved figure -> %s\n', svg_path);
fprintf('Saved PDF figure -> %s\n', pdf_path);

table_path = fullfile(out_dir, [cfg.file_prefix '_table.tex']);
writeLatexTable(table_path, values, cfg, case_defs, method_labels_plain);
fprintf('Saved LaTeX table -> %s\n', table_path);

metric_data = struct();
metric_data.metric = metric_name;
metric_data.values = values;
metric_data.run_idx = run_idx;
metric_data.case_defs = case_defs;
metric_data.method_labels = method_labels_plain;
end

function case_defs = experimentCases()
    case_defs = struct('label', {}, 'folder_patterns', {});

    case_defs(1).label = 'Main nominal agent';
    case_defs(1).folder_patterns = { ...
        'DDPGpi_main_s5_%d', ...
        'DDPGpi_main_s5_%d_nominal_on_nd'};

    case_defs(2).label = 'Mismatch nominal agent';
    case_defs(2).folder_patterns = { ...
        'DDPGpi_main_s5_%d_mm', ...
        'DDPGpi_main_s5_%d_mm_nominal_on_mm_nd'};
end

function cfg = metricConfig(metric_name)
    switch metric_name
        case 'tts'
            cfg = baseConfig(metric_name, 'TTS', ...
                'TTS (veh h)', 'tts', {'xx', 'param_sim'}, '%.1f');
        case 'obj'
            cfg = baseConfig(metric_name, 'Objective cost', ...
                'Objective cost', 'obj', {'xx', 'uu', 'param_sim'}, '%.1f');
        case 'tv'
            cfg = baseConfig(metric_name, 'Total input variation', ...
                'TV', 'tv', {'uu'}, '%.4f');
        case 'dw_tot'
            cfg = baseConfig(metric_name, 'Total queue violation', ...
                '$\Delta w_{\mathrm{tot}}$ (veh h)', 'dw_tot', {'xx'}, '%.3f');
        case 'dw_max'
            cfg = baseConfig(metric_name, 'Maximum queue violation', ...
                '$\Delta w_{\max}$ (veh)', 'dw_max', {'xx'}, '%.3f');
        case 'comp_time'
            cfg = baseConfig(metric_name, 'Computation time', ...
                'Computation time (s)', 'comp_time', {'total_comp_time'}, '%.1f');
        otherwise
            error('Unknown metric "%s".', metric_name);
    end
end

function cfg = baseConfig(metric_name, display_name, y_label, file_prefix, required_vars, value_format)
    cfg = struct('metric_name', metric_name, ...
                 'display_name', display_name, ...
                 'y_label', y_label, ...
                 'file_prefix', file_prefix, ...
                 'required_vars', {required_vars}, ...
                 'value_format', value_format);
end

function [metric_values, run_idx] = loadMetricGroup(agents_dir, folder_pattern, n_runs, cfg)
    metric_values = [];
    run_idx = [];

    for run_id = 1:n_runs
        folder_name = sprintf(folder_pattern, run_id);
        folder_path = fullfile(agents_dir, folder_name);

        if ~exist(folder_path, 'dir')
            error('Experiment folder not found: %s', folder_path);
        end

        files = sortedDir(fullfile(folder_path, 'RL_MPC_SR_RM_result_*.mat'));
        if isempty(files)
            error('No RL_MPC_SR_RM_result_*.mat files found in %s', folder_path);
        end

        for file_idx = 1:numel(files)
            mat_path = fullfile(folder_path, files(file_idx).name);
            data = load(mat_path, cfg.required_vars{:});
            metric_values(end+1, 1) = computeMetric(data, cfg.metric_name); %#ok<AGROW>
            run_idx(end+1, 1) = run_id; %#ok<AGROW>
        end
    end
end

function files = sortedDir(pattern)
    files = dir(pattern);
    if isempty(files)
        return;
    end

    [~, order] = sort({files.name});
    files = files(order);
end

function value = computeMetric(data, metric_name)
    switch metric_name
        case 'tts'
            value = computeTts(data.xx, data.param_sim);
        case 'obj'
            value = computeObjective(data.xx, data.uu, data.param_sim);
        case 'tv'
            value = computeTv(data.uu);
        case 'dw_tot'
            value = computeDwTot(data.xx);
        case 'dw_max'
            value = computeDwMax(data.xx);
        case 'comp_time'
            value = double(data.total_comp_time);
            value = value(:);
            if isempty(value)
                error('total_comp_time is empty.');
            end
            value = value(end);
        otherwise
            error('Unknown metric "%s".', metric_name);
    end
end

function tts_total = computeTts(xx, param_sim)
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
end

function obj = computeObjective(xx, uu, param_sim)
    r_cost = 0.4;
    m_low = 6;

    du = uu(1:3, 2:end) - uu(1:3, 1:end-1);
    u_pen = sum(r_cost .* sum(du.^2, 1)) ./ m_low;

    tts = computeTts(xx, param_sim);

    w_o1 = xx(64,:) + xx(65,:);
    w_o2 = xx(68,:) + xx(69,:);
    w_o3 = xx(72,:) + xx(73,:);
    q_pen = sum(max(0, w_o1 - param_sim.w_con(1)).^2 ...
              + max(0, w_o2 - param_sim.w_con(2)).^2 ...
              + max(0, w_o3 - param_sim.w_con(3)).^2);

    obj = u_pen + tts + q_pen;
end

function tv = computeTv(uu)
    du = diff(uu, 1, 2);
    tv = sum(sqrt(sum(du.^2, 1)));
end

function dw_tot = computeDwTot(xx)
    t_sim = 10 / 3600;
    w_max = [200, 100, 100];
    [dw_o1, dw_o2, dw_o3] = queueViolations(xx, w_max);
    dw_tot = t_sim * (sum(dw_o1) + sum(dw_o2) + sum(dw_o3));
end

function dw_max = computeDwMax(xx)
    w_max = [200, 100, 100];
    [dw_o1, dw_o2, dw_o3] = queueViolations(xx, w_max);
    dw_max = max([max(dw_o1), max(dw_o2), max(dw_o3)]);
end

function [dw_o1, dw_o2, dw_o3] = queueViolations(xx, w_max)
    w_o1 = xx(64,:) + xx(65,:);
    w_o2 = xx(68,:) + xx(69,:);
    w_o3 = xx(72,:) + xx(73,:);

    dw_o1 = max(0, w_o1 - w_max(1));
    dw_o2 = max(0, w_o2 - w_max(2));
    dw_o3 = max(0, w_o3 - w_max(3));
end

function fig = plotMetricFigure(values, cfg, case_defs, method_labels)
    method_colors = [
        0.0000, 0.4470, 0.7410;   % blue
        0.8500, 0.3250, 0.0980;   % orange
    ];

    [y_lo, y_hi] = axisLimits(values);
    ax_fsize = 12;
    ttl_fsize = 13;

    fig = figure('Units', 'centimeters', 'Position', [2 2 18 9], 'Color', 'w');
    tl = tiledlayout(fig, 1, numel(case_defs), 'TileSpacing', 'compact', ...
                     'Padding', 'compact');

    for case_idx = 1:numel(case_defs)
        ax = nexttile(tl);
        hold(ax, 'on');

        box_handles = gobjects(numel(method_labels), 1);
        for method_idx = 1:numel(method_labels)
            v = values{method_idx, case_idx};
            if numel(v) < 4
                v = repmat(v, 4, 1);
            end

            bc = boxchart(ax, method_idx * ones(size(v)), v);
            bc.BoxFaceColor = method_colors(method_idx, :);
            bc.BoxFaceAlpha = 0.72;
            bc.WhiskerLineColor = method_colors(method_idx, :);
            bc.LineWidth = 1.7;
            bc.MarkerStyle = '+';
            bc.MarkerColor = method_colors(method_idx, :) * 0.75;
            bc.MarkerSize = 5;
            box_handles(method_idx) = bc;
        end

        hold(ax, 'off');
        ylim(ax, [y_lo, y_hi]);
        xlim(ax, [0.45, numel(method_labels) + 0.55]);
        ax.XTick = 1:numel(method_labels);
        ax.XTickLabel = method_labels;
        ax.XTickLabelRotation = 15;
        ylabel(ax, cfg.y_label, 'FontSize', ax_fsize, 'Interpreter', 'latex');
        title(ax, case_defs(case_idx).label, 'FontSize', ttl_fsize, ...
              'FontWeight', 'bold', 'Interpreter', 'none');
        set(ax, 'Box', 'on', 'FontSize', ax_fsize, 'LineWidth', 0.8, ...
                'TickDir', 'out', 'TickLabelInterpreter', 'none');
        grid(ax, 'on');
        ax.GridAlpha = 0.12;
        ax.GridLineStyle = ':';

        if case_idx == numel(case_defs)
            lgd = legend(ax, box_handles, method_labels, 'Location', 'southoutside', ...
                         'Orientation', 'horizontal', 'Box', 'off', ...
                         'Interpreter', 'none', 'FontSize', ax_fsize - 1);
            lgd.ItemTokenSize = [16, 8];
        end
    end
end

function [y_lo, y_hi] = axisLimits(values)
    all_values = [];
    for idx = 1:numel(values)
        all_values = [all_values; values{idx}(:)]; %#ok<AGROW>
    end

    if isempty(all_values)
        error('No metric values were loaded.');
    end

    min_val = min(all_values);
    max_val = max(all_values);

    if max_val == min_val
        margin = max(abs(max_val) * 0.10, 1);
    else
        margin = 0.08 * (max_val - min_val);
    end

    y_lo = min(0, min_val - margin);
    y_hi = max_val + margin;
end

function writeLatexTable(table_path, values, cfg, case_defs, method_labels)
    fid = fopen(table_path, 'w');
    if fid < 0
        error('Could not open table file for writing: %s', table_path);
    end
    cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, '\\begin{table}[htbp]\n');
    fprintf(fid, '\\centering\n');
    fprintf(fid, ['\\caption{Extra DDPG noisy-demand comparison for %s. ' ...
        'Values are mean $\\pm$ standard deviation across the five ' ...
        'nominal-demand-trained agents and ten evaluations per agent.}\n'], ...
        cfg.display_name);
    fprintf(fid, '\\label{tab:extra_%s_comparison}\n', cfg.file_prefix);
    fprintf(fid, '\\begin{tabular}{l%s}\n', repmat('c', 1, numel(case_defs)));
    fprintf(fid, '\\toprule\n');
    fprintf(fid, 'Training setup');
    for case_idx = 1:numel(case_defs)
        fprintf(fid, ' & %s', latexText(case_defs(case_idx).label));
    end
    fprintf(fid, ' \\\\\n\\midrule\n');

    for method_idx = 1:numel(method_labels)
        fprintf(fid, '%s', latexText(method_labels{method_idx}));
        for case_idx = 1:numel(case_defs)
            fprintf(fid, ' & %s', summaryLatex(values{method_idx, case_idx}, cfg.value_format));
        end
        fprintf(fid, ' \\\\\n');
    end

    fprintf(fid, '\\midrule\n');
    fprintf(fid, 'Noisy vs nominal');
    for case_idx = 1:numel(case_defs)
        nominal_mean = mean(values{1, case_idx});
        noisy_mean = mean(values{2, case_idx});
        delta_pct = 100 * (noisy_mean - nominal_mean) / nominal_mean;
        fprintf(fid, ' & $%+.1f\\%%$', delta_pct);
    end
    fprintf(fid, ' \\\\\n');
    fprintf(fid, '\\bottomrule\n');
    fprintf(fid, '\\end{tabular}\n');
    fprintf(fid, '\\end{table}\n');
end

function txt = summaryText(v, value_format)
    if isempty(v)
        txt = 'no data';
        return;
    end

    if numel(v) == 1 || std(v) == 0
        txt = sprintf(value_format, mean(v));
    else
        txt = [sprintf(value_format, mean(v)) ' +/- ' sprintf(value_format, std(v))];
    end
end

function txt = summaryLatex(v, value_format)
    if isempty(v)
        txt = '--';
        return;
    end

    if numel(v) == 1 || std(v) == 0
        txt = ['$' sprintf(value_format, mean(v)) '$'];
    else
        txt = ['$' sprintf(value_format, mean(v)) ' \pm ' ...
               sprintf(value_format, std(v)) '$'];
    end
end

function out = latexText(in)
    out = strrep(in, '_', '\_');
end
