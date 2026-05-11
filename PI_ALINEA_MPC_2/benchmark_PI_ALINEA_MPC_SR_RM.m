
%Simulates the benchmark network with an MPC controller
%clear
%clc

%rng(10)

load("Bayes_opt_PI_ALINEA_result_2025-09-05 09_07_25.mat")

optimVals = bestPoint(optimResults);
pi_alinea_params.K_P1 = optimVals{1,1};
pi_alinea_params.K_P2 = optimVals{1,2};
pi_alinea_params.K_I1 = optimVals{1,3};
pi_alinea_params.K_I2 = optimVals{1,4};
pi_alinea_params.rho_c1 = optimVals{1,5};
pi_alinea_params.rho_c2 = optimVals{1,6};


scenario = 3;
N = 900; %total simulation steps

param_sim = param_get;
param_MPC_low = param_MPC_get(1);
param_MPC_high = param_MPC_get(0);

x=zeros(75,1);
xx = zeros(size(x,1),N);
u = [0.5;1;1];
uu = zeros(size(u,1),N);
total_comp_time = 0;

%simulate without controller to generate congestion using crit density for
%the output cell
k = 0;
for i=1:60
    if mod(k,param_MPC_low.M) == 0
        x_prev = x;
    end    
    x = fun_benchmark_RM(x,u,k,param_sim,scenario);
    k = k + 1;
end



%% up to now, it is correct

u_mpc_sr = repmat([0.5],1,param_MPC_high.Nc);
u_mpc_sr_0 = u_mpc_sr;
u_mpc_sr_prev = u_mpc_sr(:,1);
u_PI_ALINEA = [1;1];
k_c = 0;

for i=1:N
    if mod(k_c,param_MPC_low.M) == 0
        u_PI_ALINEA_prev = u_PI_ALINEA;
        tic
        u_PI_ALINEA = calc_u_alinea(x,u_PI_ALINEA,pi_alinea_params,x_prev);
        total_comp_time = total_comp_time + toc;
        x_prev = x;
    end

    if mod(k_c,param_MPC_high.M) == 0
        tic
        u_mpc_sr = MPC_solve_SR_w_PI_ALINEA_h(x,u_mpc_sr_0,u_mpc_sr_prev,u_PI_ALINEA,k,param_sim,param_MPC_low,param_MPC_high,scenario,pi_alinea_params,x_prev,u_PI_ALINEA_prev);
        total_comp_time = total_comp_time + toc;
        u_mpc_sr_0 = [u_mpc_sr(:,2:end),u_mpc_sr(:,end)];
        u_mpc_sr_prev = u_mpc_sr(:,1);
    end

    x = fun_benchmark_RM(x,[u_mpc_sr(:,1);u_PI_ALINEA],k,param_sim,scenario);
    xx(:,i) = x;
    uu(:,i) = [u_mpc_sr(:,1);u_PI_ALINEA];
    k = k+1;
    k_c = k_c + 1;
end

mpc_result_doc_name = 'PI_ALINEA_MPC_SR_RM_result_' + string(datetime('now'), 'yyyy-MM-dd hh_mm_ss') + '.mat';
save(mpc_result_doc_name)

%%
v_1_1_c1 = xx(1,:);
v_1_1_c2 = xx(2,:);
rho_1_1_c1 = xx(3,:);
rho_1_1_c2 = xx(4,:);
rho_1_1_tot = xx(5,:);
q_1_1_c1 = xx(6,:);
q_1_1_c2 = xx(7,:);

v_1_2_c1 = xx(8,:);
v_1_2_c2 = xx(9,:);
rho_1_2_c1 = xx(10,:);
rho_1_2_c2 = xx(11,:);
rho_1_2_tot = xx(12,:);
q_1_2_c1 = xx(13,:);
q_1_2_c2 = xx(14,:);

v_1_3_c1 = xx(15,:);
v_1_3_c2 = xx(16,:);
rho_1_3_c1 = xx(17,:);
rho_1_3_c2 = xx(18,:);
rho_1_3_tot = xx(19,:);
q_1_3_c1 = xx(20,:);
q_1_3_c2 = xx(21,:);

v_2_1_c1 = xx(22,:);
v_2_1_c2 = xx(23,:);
rho_2_1_c1 = xx(24,:);
rho_2_1_c2 = xx(25,:);
rho_2_1_tot = xx(26,:);
q_2_1_c1 = xx(27,:);
q_2_1_c2 = xx(28,:);

v_3_1_c1 = xx(29,:);
v_3_1_c2 = xx(30,:);
rho_3_1_c1 = xx(31,:);
rho_3_1_c2 = xx(32,:);
rho_3_1_tot = xx(33,:);
q_3_1_c1 = xx(34,:);
q_3_1_c2 = xx(35,:);

v_3_2_c1 = xx(36,:);
v_3_2_c2 = xx(37,:);
rho_3_2_c1 = xx(38,:);
rho_3_2_c2 = xx(39,:);
rho_3_2_tot = xx(40,:);
q_3_2_c1 = xx(41,:);
q_3_2_c2 = xx(42,:);

v_4_1_c1 = xx(43,:);
v_4_1_c2 = xx(44,:);
rho_4_1_c1 = xx(45,:);
rho_4_1_c2 = xx(46,:);
rho_4_1_tot = xx(47,:);
q_4_1_c1 = xx(48,:);
q_4_1_c2 = xx(49,:);

v_5_1_c1 = xx(50,:);
v_5_1_c2 = xx(51,:);
rho_5_1_c1 = xx(52,:);
rho_5_1_c2 = xx(53,:);
rho_5_1_tot = xx(54,:);
q_5_1_c1 = xx(55,:);
q_5_1_c2 = xx(56,:);

v_5_2_c1 = xx(57,:);
v_5_2_c2 = xx(58,:);
rho_5_2_c1 = xx(59,:);
rho_5_2_c2 = xx(60,:);
rho_5_2_tot = xx(61,:);
q_5_2_c1 = xx(62,:);
q_5_2_c2 = xx(63,:);

w_o_1_c1 = xx(64,:);
w_o_1_c2 = xx(65,:);
q_o_1_c1 = xx(66,:);
q_o_1_c2 = xx(67,:);

w_o_2_c1 = xx(68,:);
w_o_2_c2 = xx(69,:);
q_o_2_c1 = xx(70,:);
q_o_2_c2 = xx(71,:);

w_o_3_c1 = xx(72,:);
w_o_3_c2 = xx(73,:);
q_o_3_c1 = xx(74,:);
q_o_3_c2 = xx(75,:);


TTS=param_sim.T.*((rho_1_1_c1.*param_sim.lambda.l1 + rho_1_2_c1.*param_sim.lambda.l2 + rho_1_3_c1.*param_sim.lambda.l3...
    + rho_2_1_c1.*param_sim.lambda.l4 + rho_3_1_c1.*param_sim.lambda.l5 + rho_3_2_c1.*param_sim.lambda.l6...
    + rho_4_1_c1.*param_sim.lambda.l7 + rho_5_1_c1.*param_sim.lambda.l8 + rho_5_2_c1.*param_sim.lambda.l9).*param_sim.L_m+w_o_1_c1+w_o_2_c1+w_o_3_c1)...
+   param_sim.T.*((rho_1_1_c2.*param_sim.lambda.l1 + rho_1_2_c2.*param_sim.lambda.l2 + rho_1_3_c2.*param_sim.lambda.l3...
    + rho_2_1_c2.*param_sim.lambda.l4 + rho_3_1_c2.*param_sim.lambda.l5 + rho_3_2_c2.*param_sim.lambda.l6...
    + rho_4_1_c2.*param_sim.lambda.l7 + rho_5_1_c2.*param_sim.lambda.l8 + rho_5_2_c2.*param_sim.lambda.l9).*param_sim.L_m+w_o_1_c2+w_o_2_c2+w_o_3_c2);

Rho=[rho_5_2_tot;rho_5_1_tot;rho_4_1_tot;rho_3_2_tot;rho_3_1_tot;rho_2_1_tot;rho_1_3_tot;rho_1_2_tot;rho_1_1_tot];

fprintf('TTS is %.3f veh*h \n', sum(TTS))
fprintf('Total computation time is %.3f s \n', total_comp_time)

run network_analyzer.m

