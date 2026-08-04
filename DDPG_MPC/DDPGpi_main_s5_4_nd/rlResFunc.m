function [initialObservation, initialState] = rlResFunc
% Episode reset. Every episode starts from a network that has already run
% 60 steps without control, so the agent always begins in congested traffic
% rather than on an empty road.

%Noisy demand runs draw a new noise realisation for every episode, so the
%warm-up has to be redone here instead of being loaded from file.
base_demands = load('base_demands.mat');

Demands.o1c1 = calc_noisy_demands('o1','c1',base_demands.base_demand_o1c1);
Demands.o1c2 = calc_noisy_demands('o1','c2',base_demands.base_demand_o1c2);
Demands.o2c1 = calc_noisy_demands('o2','c1',base_demands.base_demand_o2c1);
Demands.o2c2 = calc_noisy_demands('o2','c2',base_demands.base_demand_o2c2);
Demands.o3c1 = calc_noisy_demands('o3','c1',base_demands.base_demand_o3c1);
Demands.o3c2 = calc_noisy_demands('o3','c2',base_demands.base_demand_o3c2);

param_sim = param_get;
scenario = 3;

x=zeros(75,1);

N_init = 60;

u = [0.5;1;1];

k = 0;
for i=1:N_init
    x = fun_benchmark_RM_nd(x,u,k,param_sim,scenario,Demands);
    k = k + 1;
end

xx = x;
uu = u;
kk = k;


x_norm = calc_x_norm();


demando1c1 = Demands.o1c1(kk+1);
demando1c2 = Demands.o1c2(kk+1);
demando2c1 = Demands.o2c1(kk+1);
demando2c2 = Demands.o2c2(kk+1);
demando3c1 = Demands.o3c1(kk+1);
demando3c2 = Demands.o3c2(kk+1);

initialState.states = [xx;...
    [demando1c1,demando1c2]';...
    [demando2c1,demando2c2]';...
    [demando3c1,demando3c2]';...
    uu(2:3); uu(1); kk; scenario; uu(1); uu(1)]; %last terms are for the previous MPC action trajectory to be used as an initial guess for the next MPC computation

%The demand profile is passed along with the state because the step
%function has to keep using the same realisation.
initialState.demands = Demands;

% initialObservation = [xx; uu(1:2); uu(9:12);...
%     [demando1c1_1,demando1c1_2,demando1c1_3]';...
%     [demando1c2_1,demando1c2_2,demando1c2_3]';...
%     [demando2c1_1,demando2c1_2,demando2c1_3]'; 
%     [demando2c2_1,demando2c2_2,demando2c2_3]';...
%     [demando3c1_1,demando3c1_2,demando3c1_3]';...
%     [demando3c2_1,demando3c2_2,demando3c2_3]';...
%     k_shift];

initialObservation = [xx./x_norm;...
                     [demando1c1,demando1c2]'./1000;...
                     [demando2c1,demando2c2]'./1000;...
                     [demando3c1,demando3c2]'./1000];

end

