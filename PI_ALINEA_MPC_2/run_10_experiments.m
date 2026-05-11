
diary experiments_PI_ALINEA
for i=1:10
    rng(i)
    run benchmark_PI_ALINEA_MPC_SR_RM.m
    %run benchmark_MPC_SR_RM_hier.m
    %run benchmark_nocontrol_SR_RM.m
    pause(1)
end
diary off
