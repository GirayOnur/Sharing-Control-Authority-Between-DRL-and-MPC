
tot_episode = 3000;
step_per_episode = 150;
tot_sampling_step = tot_episode*step_per_episode;

StandardDeviationDecayRate = 5e-6;
StandardDeviation = 0.3;

stddev_list = [];
for i=1:tot_sampling_step
    StandardDeviation = StandardDeviation.*(1 - StandardDeviationDecayRate);
    stddev_list(end+1) = StandardDeviation;
end

disp(StandardDeviation)

plot(stddev_list)