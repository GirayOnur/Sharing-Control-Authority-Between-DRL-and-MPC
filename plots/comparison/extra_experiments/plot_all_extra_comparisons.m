clear; clc; close all;

metrics = {'tts', 'obj', 'tv', 'dw_tot', 'dw_max', 'comp_time'};

for metric_idx = 1:numel(metrics)
    plot_extra_experiment_metric(metrics{metric_idx});
end
