function [nextObs,reward,isDone,nextState] = rlStepFunc(action,state,agent) %(action,state,agent)
% One environment step for the agent, which lasts M simulation steps (1 min).
% The agent's action is the pair of ramp metering rates and is held over all
% of them. The high level MPC is re-solved whenever its own slower control
% step comes up.
%
% state carries more than the observation, because the environment has to
% remember things the agent does not see:
%   1:75   network states
%   76:81  demands at the three origins, both classes
%   82:83  ramp rates applied in the previous step
%   84     split rate currently applied by the MPC
%   85     simulation step, 86 scenario
%   87:88  previous MPC solution, reused as the warm start

MPC_param = param_MPC_get(1);
MPC_param_high = param_MPC_get(0);
param = param_get;

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
    x = fun_benchmark_RM(x,u_combined,k,param,scenario);
    xx(:,i) = x;
    k = k + 1;
end


%The MPC control step is slower than the agent's, so the split rate is
%only re-solved once every M_high simulation steps.
if mod(k,MPC_param_high.M) == 0
    u_mpc = MPC_solve_SR_w_RL(x,u_mpc_0,mpc_action,rl_actions,k,param,MPC_param,MPC_param_high,scenario,agent);
    u_mpc_0 = [u_mpc(:,2:end),u_mpc(:,end)];
    mpc_action = u_mpc(:,1);
end


%update the next states and the observation:
[demando1c1,demando1c2]  = demando1(k-1,scenario);
[demando2c1,demando2c2]  = demando2(k-1,scenario);
[demando3c1,demando3c2]  = demando3(k-1,scenario);

nextState = [x
            [demando1c1,demando1c2]';...
            [demando2c1,demando2c2]';...
            [demando3c1,demando3c2]';...
             action; mpc_action; k; scenario;  u_mpc_0(:,1); u_mpc_0(:,2)]; 



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

    %Reward is minus the cost the MPC minimises, so both levels chase the same
    %thing: time spent, plus a penalty for changing the ramp rates, plus a
    %queue penalty. The /30 only keeps the numbers in a range the critic
    %handles well.
    %Reward is minus the cost the MPC minimises, so both levels chase the same
    %thing: time spent, plus a penalty for changing the ramp rates, plus a
    %queue penalty. The /30 only keeps the numbers in a range the critic
    %handles well.
    u_pen  = MPC_param.r_cost.*sum((rl_actions - action).^2); 
    tts = sum(param.T.*((rho_1_1_c1.*param.lambda.l1 + rho_1_2_c1.*param.lambda.l2 + rho_1_3_c1.*param.lambda.l3...
    + rho_2_1_c1.*param.lambda.l4 + rho_3_1_c1.*param.lambda.l5 + rho_3_2_c1.*param.lambda.l6...
    + rho_4_1_c1.*param.lambda.l7 + rho_5_1_c1.*param.lambda.l8 + rho_5_2_c1.*param.lambda.l9).*param.L_m+w_o_1_c1+w_o_2_c1+w_o_3_c1)...
+   param.T.*((rho_1_1_c2.*param.lambda.l1 + rho_1_2_c2.*param.lambda.l2 + rho_1_3_c2.*param.lambda.l3...
    + rho_2_1_c2.*param.lambda.l4 + rho_3_1_c2.*param.lambda.l5 + rho_3_2_c2.*param.lambda.l6...
    + rho_4_1_c2.*param.lambda.l7 + rho_5_1_c2.*param.lambda.l8 + rho_5_2_c2.*param.lambda.l9).*param.L_m+w_o_1_c2+w_o_2_c2+w_o_3_c2));

    %q_pen = sum(max(0,(w_o_1_c1+w_o_1_c2) - param.w_con(1)).^2 + max(0,(w_o_2_c1+w_o_2_c2) - param.w_con(2)).^2 ...
                %+ max(0,(w_o_3_c1+w_o_3_c2) - param.w_con(3)).^2);

    %Queue penalty: a flat charge plus a term that grows with the queue, and
    %only when a limit is actually crossed.
    %Queue penalty: a flat charge plus a term that grows with the queue, and
    %only when a limit is actually crossed.
    q_pen= (10 + max(w_o_1_c1+w_o_1_c2)./100).*(max((w_o_1_c1+w_o_1_c2)-param.w_con(1))>0) + (10 + max(w_o_2_c1+w_o_2_c2)./100).*(max((w_o_2_c1+w_o_2_c2)-param.w_con(2))>0) + (10 + max(w_o_3_c1+w_o_3_c2)./100).*(max((w_o_3_c1+w_o_3_c2)-param.w_con(3))>0);
    
    reward = -(tts + u_pen + q_pen)./30;

    %reward = -(tts + u_pen + q_pen/MPC_param.Np); %scale q_pen since it only considers the max values instead of considering values at each time step

%60 warm-up steps plus 900 simulated steps ends the episode.
if k>959
    isDone=true;
else
    isDone=false;
end

end

