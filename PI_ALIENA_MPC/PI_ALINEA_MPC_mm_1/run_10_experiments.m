%Runs the 10 evaluation simulations of this folder, with seed 1 to 10, and
%writes the console output to a diary file. The commented out lines pick a
%different control framework; keep only one of them active.

diary experiments_PI_ALINEA
for i=1:10
    rng(i)
    run benchmark_PI_ALINEA_MPC_SR_RM_mm.m
    %run benchmark_MPC_SR_RM_hier.m
    %run benchmark_nocontrol_SR_RM.m
    pause(1)
end
diary off
