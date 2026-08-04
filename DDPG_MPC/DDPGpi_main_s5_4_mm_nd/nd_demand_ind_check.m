%Sanity check that base_demands.mat lines up with the nominal demand
%functions: the stored profile is read at k+1 and the functions at k-1, so
%both curves should land on top of each other.
clear
clc
scenario = 3;

base_demands = load('base_demands.mat');

for k=1:960
    [demando1c1,demando1c2]  = demando1(k-1,scenario);
    [demando2c1,demando2c2]  = demando2(k-1,scenario);
    [demando3c1,demando3c2]  = demando3(k-1,scenario);

    demand1 = demando2c1 + demando2c2;
    demand2 = base_demands.base_demand_o2c1(k+1) + base_demands.base_demand_o2c2(k+1);
    plot(k,demand1,'o')
    hold on
    plot(k,demand2,'*')
end
