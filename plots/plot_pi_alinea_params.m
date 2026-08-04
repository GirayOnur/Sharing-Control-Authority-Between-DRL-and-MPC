function plot_pi_alinea_params()
%PLOT_PI_ALINEA_PARAMS Plot Bayesian-optimised PI-ALINEA parameters per scenario.
%
% For each of the four scenarios (S1 nominal, S2 noisy demand, S3 model
% mismatch, S4 both), this script loads the five tuned parameter sets from
% the corresponding PI_ALINEA_MPC_* folders and renders a 2x3 panel figure
% (one panel per parameter) with five jittered markers per scenario and a
% horizontal tick at the per-scenario mean. The Bayesian-opt search bounds
% are drawn as dotted horizontal lines.
%
% Output:
%   pi_alinea_params.pdf next to this script.

clear_locals = onCleanup(@() []); %#ok<NASGU>

script_dir = fileparts(mfilename('fullpath'));
agents_dir = fullfile(script_dir, '..');

scenarios = struct( ...
    'label',          {'S1', 'S2', 'S3', 'S4'}, ...
    'folder_pattern', {'PI_ALINEA_MPC_%d', ...
                       'PI_ALINEA_MPC_nd_%d', ...
                       'PI_ALINEA_MPC_mm_%d', ...
                       'PI_ALINEA_MPC_mm_nd_%d'}, ...
    'result_marker',  {'SR_RM_result', ...
                       'SR_RM_nd_result', ...
                       'SR_RM_mm_result', ...
                       'SR_RM_mm_nd_result'});

% Parameter mapping from the Matlab code names to the paper notation:
%   K_I (gain on rho_c - rho_b)        -> K_R (paper),  search bound [0,4]
%   K_P (gain on rho_b - rho_b_prev)   -> K_A (paper),  search bound [0,15]
%   rho_c                              -> bar(rho),     search bound [30,100]
params = struct( ...
    'code_field', {'K_I1', 'K_I2', 'K_P1', 'K_P2', 'rho_c1', 'rho_c2'}, ...
    'label_tex',  {'$K_{\mathrm{R},1}$', '$K_{\mathrm{R},2}$', ...
                   '$K_{\mathrm{A},1}$', '$K_{\mathrm{A},2}$', ...
                   '$\bar{\rho}_{1}$ [veh/km/lane]', ...
                   '$\bar{\rho}_{2}$ [veh/km/lane]'}, ...
    'bounds',     {[0,4], [0,4], [0,15], [0,15], [30,100], [30,100]});

n_runs = 5;
n_scen = numel(scenarios);
n_par  = numel(params);

% Container: values{p_idx} is an [n_runs x n_scen] matrix.
values = cell(n_par, 1);
for p_idx = 1:n_par
    values{p_idx} = nan(n_runs, n_scen);
end

fprintf('Loading PI-ALINEA parameter sets...\n');
for s_idx = 1:n_scen
    for run_id = 1:n_runs
        folder_name = sprintf(scenarios(s_idx).folder_pattern, run_id);
        folder_path = fullfile(agents_dir, folder_name);
        if ~exist(folder_path, 'dir')
            error('Folder not found: %s', folder_path);
        end

        pat = ['PI_ALINEA_MPC_' scenarios(s_idx).result_marker '_*.mat'];
        files = dir(fullfile(folder_path, pat));
        if isempty(files)
            % Fall back to any file containing the marker string.
            all_mats = dir(fullfile(folder_path, '*.mat'));
            keep = false(numel(all_mats), 1);
            for j = 1:numel(all_mats)
                keep(j) = contains(all_mats(j).name, scenarios(s_idx).result_marker);
            end
            files = all_mats(keep);
        end
        if isempty(files)
            error('No result file matching "%s" in %s', ...
                  scenarios(s_idx).result_marker, folder_path);
        end

        mat_path = fullfile(folder_path, files(1).name);
        loaded = load(mat_path, 'pi_alinea_params');
        if ~isfield(loaded, 'pi_alinea_params')
            error('pi_alinea_params not present in %s', mat_path);
        end
        p = loaded.pi_alinea_params;

        for p_idx = 1:n_par
            values{p_idx}(run_id, s_idx) = double(p.(params(p_idx).code_field));
        end

        fprintf('  %s run %d -> %s\n', scenarios(s_idx).label, run_id, ...
                files(1).name);
    end
end

% Print summary for sanity-checking.
fprintf('\nPer-scenario means:\n');
header = sprintf('  %-8s', 'param');
for s_idx = 1:n_scen
    header = [header sprintf('%10s', scenarios(s_idx).label)]; %#ok<AGROW>
end
fprintf('%s\n', header);
for p_idx = 1:n_par
    row = sprintf('  %-8s', params(p_idx).code_field);
    for s_idx = 1:n_scen
        row = [row sprintf('%10.3f', mean(values{p_idx}(:, s_idx)))]; %#ok<AGROW>
    end
    fprintf('%s\n', row);
end

% --- Plot --------------------------------------------------------------
fig = figure('Units', 'centimeters', 'Position', [2 2 18.3 8.6], 'Color', 'w');
tl = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

marker_color = [0.121, 0.466, 0.705];   % matches matplotlib tab:blue
mean_color   = [0, 0, 0];
ax_fsize  = 9;
lbl_fsize = 10;

rng_state = rng();           % save current RNG state
cleanup_rng = onCleanup(@() rng(rng_state)); %#ok<NASGU>

for p_idx = 1:n_par
    ax = nexttile(tl);
    hold(ax, 'on');

    rng(7 + p_idx, 'twister');  % reproducible jitter per panel
    for s_idx = 1:n_scen
        v = values{p_idx}(:, s_idx);
        x = s_idx + (rand(numel(v), 1) - 0.5) * 0.22;
        scatter(ax, x, v, 22, 'MarkerFaceColor', marker_color, ...
                'MarkerEdgeColor', 'k', 'LineWidth', 0.4, ...
                'MarkerFaceAlpha', 0.85);

        m = mean(v);
        plot(ax, [s_idx - 0.25, s_idx + 0.25], [m, m], ...
             'Color', mean_color, 'LineWidth', 1.3);
    end

    b = params(p_idx).bounds;
    pad = 0.05 * (b(2) - b(1));
    ylim(ax, [b(1) - pad, b(2) + pad]);
    xlim(ax, [0.5, n_scen + 0.5]);

    plot(ax, [0.5, n_scen + 0.5], [b(1), b(1)], ':', ...
         'Color', [0.6 0.6 0.6], 'LineWidth', 0.6);
    plot(ax, [0.5, n_scen + 0.5], [b(2), b(2)], ':', ...
         'Color', [0.6 0.6 0.6], 'LineWidth', 0.6);

    ax.XTick = 1:n_scen;
    ax.XTickLabel = {scenarios.label};
    if p_idx > 3
        xlabel(ax, 'Scenario', 'FontSize', lbl_fsize, 'Interpreter', 'tex');
    else
        ax.XTickLabel = repmat({''}, 1, n_scen);
    end
    ylabel(ax, params(p_idx).label_tex, ...
           'FontSize', lbl_fsize, 'Interpreter', 'latex');

    grid(ax, 'on');
    ax.GridAlpha = 0.12;
    ax.GridLineStyle = ':';
    set(ax, 'Box', 'on', 'FontSize', ax_fsize, 'LineWidth', 0.8, ...
            'TickDir', 'out', 'TickLabelInterpreter', 'none', ...
            'Layer', 'top');
    hold(ax, 'off');
end

set(fig, 'Renderer', 'painters');

% Make the PDF page exactly the size of the figure (kills the default
% letter-paper white margins around the axes).
drawnow;
set(fig, 'Units', 'centimeters');
fig_pos = get(fig, 'Position');
fig_size = fig_pos(3:4);
set(fig, 'PaperUnits', 'centimeters', ...
         'PaperSize', fig_size, ...
         'PaperPosition', [0 0 fig_size], ...
         'PaperPositionMode', 'manual');

% Save next to this script and also to the paper's figures dir if present.
out_pdf_local = fullfile(script_dir, 'pi_alinea_params.pdf');
print(fig, out_pdf_local, '-dpdf', '-vector');
fprintf('\nSaved PDF -> %s\n', out_pdf_local);

paper_fig_dir = fullfile('/home', 'giray', 'projects', ...
    'A_Novel_RL_MPC_Framework_for_Multi_class_Transportation_Networks', ...
    'figures');
if exist(paper_fig_dir, 'dir')
    out_pdf_paper = fullfile(paper_fig_dir, 'pi_alinea_params.pdf');
    print(fig, out_pdf_paper, '-dpdf', '-vector');
    fprintf('Saved PDF -> %s\n', out_pdf_paper);
end

end
