%Plots a nominal demand profile against its noisy version, to check the
%size of the noise used in scenarios 2 and 4.


base_demands = load('base_demands.mat');




plot(base_demands.base_demand_o2c2, 'b')
hold on

Demands.o2c2 = calc_noisy_demands('o2','c2',base_demands.base_demand_o2c2);

plot(Demands.o2c2, 'r')



