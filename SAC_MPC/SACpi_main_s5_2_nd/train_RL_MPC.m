
%Prepares what the training needs and then starts it.
%
%In the plain folders that means running the network 60 steps without
%control and storing the resulting state as net_init.mat, which is where
%every training episode begins. In the noisy demand folders it means
%sampling the demand functions into base_demands.mat instead.
clear
clc

rng(2)

scenario = 3;
N_demand = 1030;


%Noisy demand runs: sample the demand functions once and store the result
%as base_demands.mat. rlResFunc adds fresh noise to it every episode.
%The warm-up state is not stored here, it is redone inside rlResFunc.
base_demand_o1c1 = nan(1,N_demand);
base_demand_o1c2 = nan(1,N_demand);
base_demand_o2c1 = nan(1,N_demand);
base_demand_o2c2 = nan(1,N_demand);
base_demand_o3c1 = nan(1,N_demand);
base_demand_o3c2 = nan(1,N_demand);

for k = 1:N_demand
    [base_demand_o1c1(1,k),base_demand_o1c2(1,k)] = demando1(k-2, scenario);
    [base_demand_o2c1(1,k),base_demand_o2c2(1,k)] = demando2(k-2, scenario);
    [base_demand_o3c1(1,k),base_demand_o3c2(1,k)] = demando3(k-2, scenario);
end

save("base_demands",'base_demand_o1c1', 'base_demand_o1c2', 'base_demand_o2c1','base_demand_o2c2',...
    'base_demand_o3c1', 'base_demand_o3c2');

%RL training:
%Everything else (agent, environment, training) is in const_RL.m.
run const_RL.m