
diary experiments_RL_MPC
for i=1:10
    rng(i)
    run benchmark_RL_MPC_SR_RM_mm.m
    %run benchmark_MPC_SR_RM_hier.m
    %run benchmark_nocontrol_SR_RM.m
    pause(1)
end
diary off
