
%Simulates the benchmark network with an integrated RL-MPC controller
clear
clc

rng(5)

param_sim = param_get;
param_MPC = param_MPC_get(1);

scenario = 3;
N = 900;

x=zeros(75,1);

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
run const_RL.m