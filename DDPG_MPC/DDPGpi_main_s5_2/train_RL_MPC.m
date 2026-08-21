
%Prepares what the training needs and then starts it.
%
%For the nominal demand scenarios that means running the initialization
%period and storing the resulting state as net_init.mat, the point every
%training episode starts from. For the noisy demand scenarios it means
%sampling the nominal demand profiles into base_demands.mat instead.
clear
clc

rng(2)

param_sim = param_get;
param_MPC = param_MPC_get(1);

scenario = 3;
N = 900;

x=zeros(75,1);

%Initialization period: 10 min under the no-control setting, with the split
%rate at 0.5 and both ramps unrestricted. The resulting congested state is
%the starting point of every training episode.
N_init = 60;
xx_init = zeros(size(x,1),N_init);


xx = zeros(size(x,1),N);

u = [0.5;1;1];
uu = zeros(size(u,1),N);

k = 0;
for i=1:N_init
    x = fun_benchmark_RM(x,u,k,param_sim,scenario);
    xx_init(:,i) = x;
    k = k + 1;
end


save("net_init",'x', 'u', 'scenario','k');

%RL training:
%Everything else (agent, environment, training) is in const_RL.m.
run const_RL.m