%Runs the benchmark in this folder 10 times, with seed 1 to 10, and writes
%the console output to a diary file. The lines that are commented out pick a
%different controller; keep only one of them active.

diary experiments_PI_ALINEA
for i=1:10
    rng(i)
    run benchmark_PI_ALINEA_MPC_SR_RM_nd.m
    %run benchmark_MPC_SR_RM_hier.m
    %run benchmark_nocontrol_SR_RM.m
    pause(1)
end
diary off
