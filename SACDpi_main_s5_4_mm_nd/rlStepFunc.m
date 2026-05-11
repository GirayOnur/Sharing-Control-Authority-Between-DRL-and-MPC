function [nextObs,reward,isDone,nextInfo] = rlStepFunc(action,info,agent) %(action,state,agent)

state = info.states;
demands = info.demands;

MPC_param = param_MPC_get(1);
MPC_param_high = param_MPC_get(0);
param = param_get;
param_mm = param_get_mm;

x = state(1:75);
mpc_action = state(84);
rl_actions = state(82:83);
k = state(85);
scenario = state(86);

u_mpc_0 = [state(87),state(88)];

x_norm = calc_x_norm();

xx = zeros(75,MPC_param.M);

u_combined = [mpc_action;action];
for i=1:MPC_param.M
    x = fun_benchmark_RM_nd(x,u_combined,k,param,scenario,demands);
    xx(:,i) = x;
    k = k + 1;
end


%run the MPC controller before starting the next episode
if mod(k,MPC_param_high.M) == 0
    u_mpc = MPC_solve_SR_w_RL_mm(x,u_mpc_0,mpc_action,rl_actions,k,param_mm,MPC_param,MPC_param_high,scenario,agent);
    u_mpc_0 = [u_mpc(:,2:end),u_mpc(:,end)];
    mpc_action = u_mpc(:,1);
end


%update the next states and the observation:
demando1c1 = demands.o1c1(k+1);
demando1c2 = demands.o1c2(k+1);
demando2c1 = demands.o2c1(k+1);
demando2c2 = demands.o2c2(k+1);
demando3c1 = demands.o3c1(k+1);
demando3c2 = demands.o3c2(k+1);


nextState = [x
            [demando1c1,demando1c2]';...
            [demando2c1,demando2c2]';...
            [demando3c1,demando3c2]';...
             action; mpc_action; k; scenario;  u_mpc_0(:,1); u_mpc_0(:,2)]; 

nextInfo.states = nextState;
nextInfo.demands = demands;

nextObs = [x./x_norm;...
          [demando1c1,demando1c2]'./1000;...
          [demando2c1,demando2c2]'./1000;...
          [demando3c1,demando3c2]'./1000];





rho_1_1_c1 = xx(3,:);
rho_1_1_c2 = xx(4,:);

rho_1_2_c1 = xx(10,:);
rho_1_2_c2 = xx(11,:);

rho_1_3_c1 = xx(17,:);
rho_1_3_c2 = xx(18,:);

rho_2_1_c1 = xx(24,:);
rho_2_1_c2 = xx(25,:);

rho_3_1_c1 = xx(31,:);
rho_3_1_c2 = xx(32,:);

rho_3_2_c1 = xx(38,:);
rho_3_2_c2 = xx(39,:);

rho_4_1_c1 = xx(45,:);
rho_4_1_c2 = xx(46,:);

rho_5_1_c1 = xx(52,:);
rho_5_1_c2 = xx(53,:);

rho_5_2_c1 = xx(59,:);
rho_5_2_c2 = xx(60,:);

w_o_1_c1 = xx(64,:);
w_o_1_c2 = xx(65,:);

w_o_2_c1 = xx(68,:);
w_o_2_c2 = xx(69,:);

w_o_3_c1 = xx(72,:);
w_o_3_c2 = xx(73,:);

    u_pen  = MPC_param.r_cost.*sum((rl_actions - action).^2); 
    tts = sum(param.T.*((rho_1_1_c1.*param.lambda.l1 + rho_1_2_c1.*param.lambda.l2 + rho_1_3_c1.*param.lambda.l3...
    + rho_2_1_c1.*param.lambda.l4 + rho_3_1_c1.*param.lambda.l5 + rho_3_2_c1.*param.lambda.l6...
    + rho_4_1_c1.*param.lambda.l7 + rho_5_1_c1.*param.lambda.l8 + rho_5_2_c1.*param.lambda.l9).*param.L_m+w_o_1_c1+w_o_2_c1+w_o_3_c1)...
+   param.T.*((rho_1_1_c2.*param.lambda.l1 + rho_1_2_c2.*param.lambda.l2 + rho_1_3_c2.*param.lambda.l3...
    + rho_2_1_c2.*param.lambda.l4 + rho_3_1_c2.*param.lambda.l5 + rho_3_2_c2.*param.lambda.l6...
    + rho_4_1_c2.*param.lambda.l7 + rho_5_1_c2.*param.lambda.l8 + rho_5_2_c2.*param.lambda.l9).*param.L_m+w_o_1_c2+w_o_2_c2+w_o_3_c2));

    %q_pen = sum(max(0,(w_o_1_c1+w_o_1_c2) - param.w_con(1)).^2 + max(0,(w_o_2_c1+w_o_2_c2) - param.w_con(2)).^2 ...
                %+ max(0,(w_o_3_c1+w_o_3_c2) - param.w_con(3)).^2);

    q_pen= (10 + max(w_o_1_c1+w_o_1_c2)./100).*(max((w_o_1_c1+w_o_1_c2)-param.w_con(1))>0) + (10 + max(w_o_2_c1+w_o_2_c2)./100).*(max((w_o_2_c1+w_o_2_c2)-param.w_con(2))>0) + (10 + max(w_o_3_c1+w_o_3_c2)./100).*(max((w_o_3_c1+w_o_3_c2)-param.w_con(3))>0);
    
    reward = -(tts + u_pen + q_pen)./30;

    %reward = -(tts + u_pen + q_pen/MPC_param.Np); %scale q_pen since it only considers the max values instead of considering values at each time step

if k>959
    isDone=true;
else
    isDone=false;
end

end
