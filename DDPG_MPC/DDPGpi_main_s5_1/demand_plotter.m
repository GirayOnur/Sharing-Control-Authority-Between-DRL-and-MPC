%Plots the demand of one origin over the simulation. The filter only smooths
%the one minute steps for the figure, it does not change the demand used in
%the experiments.


kf = 960;
scenario = 1;
t = linspace(0,900,901).*10./3600;
demands = nan(1,901);
for i=60:kf
    [demandc1,demandc2] = demando2(i,scenario);
    demand = demandc1 + demandc2;
    demands(i-59) = demand;
    % plot(i,demand,'o')
    % hold on
end
[b,a] = butter(1,0.1);
demands = filtfilt(b,a,demands);
plot(t,demands)
title("Vehicle demand vs time")
xlabel('time [h]') 
ylabel('vehicle demand [veh/h]') 