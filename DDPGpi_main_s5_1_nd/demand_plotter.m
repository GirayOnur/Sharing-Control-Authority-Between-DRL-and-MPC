


kf = 960;
scenario = 3;
t = linspace(0,900,901).*10./3600;


base_demands = load('base_demands.mat');


Demands.o2c1 = calc_noisy_demands('o2','c1',base_demands.base_demand_o2c1);
Demands.o2c2 = calc_noisy_demands('o2','c2',base_demands.base_demand_o2c2);

demands = Demands.o2c1(63:963) + Demands.o2c2(63:963);

plot(t,demands)
title("Vehicle demand vs time")
xlabel('time [h]') 
ylabel('vehicle demand [veh/h]')