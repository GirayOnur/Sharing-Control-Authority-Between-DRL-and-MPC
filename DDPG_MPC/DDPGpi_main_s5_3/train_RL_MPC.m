
%Prepares what the training needs and then starts it.
%
%In the plain folders that means running the network 60 steps without
%control and storing the resulting state as net_init.mat, which is where
%every training episode begins. In the noisy demand folders it means
%sampling the demand functions into base_demands.mat instead.
clear
clc

rng(3)

param_sim = param_get;
param_MPC = param_MPC_get(1);

scenario = 3;
N = 900;

x=zeros(75,1);

%Warm-up: run the network with no control until congestion has built up,
%then store that state as the starting point of every training episode.
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