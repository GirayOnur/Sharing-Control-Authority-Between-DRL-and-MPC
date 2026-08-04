
%Prepares what the training needs and then starts it.
%
%For the nominal demand scenarios that means running the initialization
%period and storing the resulting state as net_init.mat, the point every
%training episode starts from. For the noisy demand scenarios it means
%sampling the nominal demand profiles into base_demands.mat instead.
clear
clc

rng(2)

scenario = 3;
N_demand = 1030;


%Noisy demand runs: sample the demand functions once and store the result
%as base_demands.mat. rlResFunc adds fresh noise to it every episode.
%The initialization state is not stored here, it is redone inside rlResFunc.
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