

base_demands = load('base_demands.mat');




plot(base_demands.base_demand_o2c2, 'b')
hold on

Demands.o2c2 = calc_noisy_demands('o2','c2',base_demands.base_demand_o2c2);

plot(Demands.o2c2, 'r')



