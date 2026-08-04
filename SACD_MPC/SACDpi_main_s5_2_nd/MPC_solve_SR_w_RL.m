function u_mpc = MPC_solve_SR_w_RL(x0,u0,u_mpc_prev,u_rm,k,param,MPC_param_low,MPC_param_high,scenario,agent,demands)
% High-level MPC controller of the DRL-MPC framework: computes the vehicle
% split rate. Unlike MPC_solve_SR, it does not reuse a stored low-level
% trajectory. It evaluates the trained DRL policy every m_l steps inside
% the prediction, so the predicted ramp metering rates are the ones the
% agent would actually apply.
%
% Soft-penalty version, used while training the DRL agent. The queue length
% limits enter the cost as the penalty q_pen instead of hard constraints,
% which avoids infeasibility early in training when the policy does not yet
% regulate the ramps well. MPC_solve_SR_w_RL_h is the deployment version.

%rng('default')
x_norm = calc_x_norm;

Np = MPC_param_low.Np;
Nc = MPC_param_low.Nc;
M = MPC_param_low.M;

Np_high = MPC_param_high.Np;
Nc_high = MPC_param_high.Nc;
M_high = MPC_param_high.M;

r_cost = MPC_param_high.r_cost;
s_cost = MPC_param_high.s_cost;
k_opt = MPC_param_high.N_multi_start;

w_con = param.w_con;

%options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1, 'TolX',1e-2, 'TolCon', 1e-2);
%Optimality, step and constraint tolerances are set to 1e-2 to trade
%computational accuracy for efficiency, since this problem is solved at
%every control step of the simulation.
options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','OptimalityTolerance',1e-2, 'StepTolerance',1e-2, 'MaxIterations', 6, 'MaxFunctionEvaluations', 30);


lb = repmat([0],1,Nc_high);
ub = repmat([1],1,Nc_high);

fun = @(u_mpc) J_calc(x0, u_mpc, k, Nc, Np, M, u_mpc_prev, u_rm,Nc_high, Np_high, M_high, r_cost, s_cost, param, scenario,agent,x_norm,demands);
nonlcon = [];
%nonlcon = @(u_mpc) x_con(x0, u_mpc, k, w_con, Nc, Np, M,u_rm, Nc_high, Np_high, M_high, param, scenario,agent,x_norm,demands);


%Multi-start strategy. The problem is nonconvex, so SQP is run from several
%initializations: the shifted previous solution, both bounds, then random
%guesses. The best candidate is kept.
fval = nan(1,k_opt);
u_opt = cell(1,k_opt);


parfor i=1:k_opt
    if i == 1
        u_init = u0;
    elseif i == 2
        u_init = lb;
    elseif i == 3
        u_init = ub;
    else
        %u_init = u0;
        u_init = repmat([rand(1)],1, Nc_high);
    end

    [u_opt{i}, fval(i), exitflag] = fmincon(fun, u_init, [], [], [], [], lb, ub, nonlcon, options);
    
    %uncomment below for the exact constraint satisfaction.
    %comment it assuming that constraints already exists in TTS due to
    %considering queues in TTS, so that even the constraints are violated,
    %the solution with min. TTS will be likely the solution that mostly
    %have queue values close to the constraints
    % if exitflag <= 0
    %     fval(i) = inf;
    % end
end


[fval_min, i_min] = min(fval);
%disp(fval_min)

u_mpc = u_opt{i_min};

% if fval_min == inf
%     error('SQP failed to find a feasible solution.')
% else
%     u_mpc = u_opt{i_min};
% end

end


%%%
function J = J_calc(x,u,k,Nc,Np,M,u0_1,u_rm,Nc_high,Np_high,M_high,r_cost,s_cost,param,scenario,agent,x_norm,demands)
    k_obj = k;
    u1 = [u0_1, u];
    %Quadratic penalty on fluctuations between consecutive control inputs.
    %u0_1 is the input currently applied, which is why it is put in front.
    u_diff = u1(:,2:end) - u1(:,1:end-1);  
    u_pen = s_cost.*sum(sum(u_diff.^2));
    
    
    u_high = u;
    u_Np_high = [u_high repelem(u_high(:, end),1,Np_high-Nc_high)];
    u_seq_high = repelem(u_Np_high,1,M_high);
    
    %Predict the network states at every sampling step of the horizon, not just
    %at the control steps, so the objective and the constraints are evaluated
    %on the full trajectory.
    x_seq = zeros(size(x,1),Np*M);
    %u_pen_RL = 0;
    %u_RM_prev = u_rm;
    for i=1:Np*M
        %A low-level control step falls here, so the DRL policy is evaluated to get
        %the predicted ramp metering rates. The observation is built exactly as
        %during deployment: normalized network states plus the origin demands.
        if mod(k_obj,M) == 0
            demando1c1 = demands.o1c1(k_obj+1);
            demando1c2 = demands.o1c2(k_obj+1);
            demando2c1 = demands.o2c1(k_obj+1);
            demando2c2 = demands.o2c2(k_obj+1);
            demando3c1 = demands.o3c1(k_obj+1);
            demando3c2 = demands.o3c2(k_obj+1);
            agentObs = [x./x_norm;...
                         [demando1c1,demando1c2]'./1000;...
                         [demando2c1,demando2c2]'./1000;...
                         [demando3c1,demando3c2]'./1000];       
            u_low = getAction(agent,agentObs);
            u_rl = u_low{1};

            %u_pen_RL = u_pen_RL + r_cost.*sum((u_rl-u_RM_prev).^2);
            %u_RM_prev = u_rl;
        end

        x = fun_benchmark_RM_nd(x,[u_seq_high(:,i);u_rl],k_obj,param,scenario,demands);
        x_seq(:,i) = x;
        k_obj = k_obj + 1;
    end
    

rho_1_1_c1 = x_seq(3,:);
rho_1_1_c2 = x_seq(4,:);

rho_1_2_c1 = x_seq(10,:);
rho_1_2_c2 = x_seq(11,:);

rho_1_3_c1 = x_seq(17,:);
rho_1_3_c2 = x_seq(18,:);

rho_2_1_c1 = x_seq(24,:);
rho_2_1_c2 = x_seq(25,:);

rho_3_1_c1 = x_seq(31,:);
rho_3_1_c2 = x_seq(32,:);

rho_3_2_c1 = x_seq(38,:);
rho_3_2_c2 = x_seq(39,:);

rho_4_1_c1 = x_seq(45,:);
rho_4_1_c2 = x_seq(46,:);

rho_5_1_c1 = x_seq(52,:);
rho_5_1_c2 = x_seq(53,:);

rho_5_2_c1 = x_seq(59,:);
rho_5_2_c2 = x_seq(60,:);

w_o_1_c1 = x_seq(64,:);
w_o_1_c2 = x_seq(65,:);

w_o_2_c1 = x_seq(68,:);
w_o_2_c2 = x_seq(69,:);

w_o_3_c1 = x_seq(72,:);
w_o_3_c2 = x_seq(73,:);


    %TTS (total time spent): vehicles on the network, density * lanes * segment
    %length, plus the vehicles waiting in the three origin queues, summed over
    %the prediction horizon and both vehicle classes.
    tts = sum(param.T.*((rho_1_1_c1.*param.lambda.l1 + rho_1_2_c1.*param.lambda.l2 + rho_1_3_c1.*param.lambda.l3...
    + rho_2_1_c1.*param.lambda.l4 + rho_3_1_c1.*param.lambda.l5 + rho_3_2_c1.*param.lambda.l6...
    + rho_4_1_c1.*param.lambda.l7 + rho_5_1_c1.*param.lambda.l8 + rho_5_2_c1.*param.lambda.l9).*param.L_m+w_o_1_c1+w_o_2_c1+w_o_3_c1)...
+   param.T.*((rho_1_1_c2.*param.lambda.l1 + rho_1_2_c2.*param.lambda.l2 + rho_1_3_c2.*param.lambda.l3...
    + rho_2_1_c2.*param.lambda.l4 + rho_3_1_c2.*param.lambda.l5 + rho_3_2_c2.*param.lambda.l6...
    + rho_4_1_c2.*param.lambda.l7 + rho_5_1_c2.*param.lambda.l8 + rho_5_2_c2.*param.lambda.l9).*param.L_m+w_o_1_c2+w_o_2_c2+w_o_3_c2));

    q_pen = sum(max(0,(w_o_1_c1+w_o_1_c2) - param.w_con(1)).^2 + max(0,(w_o_2_c1+w_o_2_c2) - param.w_con(2)).^2 ...
                + max(0,(w_o_3_c1+w_o_3_c2) - param.w_con(3)).^2);

    J = tts + u_pen*(Nc/Nc_high) + q_pen; %  + u_pen_RL; %u_pen is scaled to be consistent with the high freq. action penalty in high freq. MPC (MPC_RM)
end
%%%





%%%
%Queue length limits as hard constraints: at every predicted sampling step
%the queue at each origin has to stay below w_con. fmincon gets c <= 0.
function [c,ceq] = x_con(x,u,k,w_con,Nc,Np,M,u_rm,Nc_high,Np_high,M_high,param,scenario,agent,x_norm,demands)
    ceq = [];
    c = [];
    
    % k_con = k;
    % 
    % u_high = u;
    % u_Np_high = [u_high repelem(u_high(:, end),1,Np_high-Nc_high)];
    % u_seq_high = repelem(u_Np_high,1,M_high);
    % 
    % x_seq = zeros(size(x,1),Np*M);
    % for i=1:Np*M
    % 
    %     if mod(k_con,M) == 0
    %         demando1c1 = demands.o1c1(k_con+1);
    %         demando1c2 = demands.o1c2(k_con+1);
    %         demando2c1 = demands.o2c1(k_con+1);
    %         demando2c2 = demands.o2c2(k_con+1);
    %         demando3c1 = demands.o3c1(k_con+1);
    %         demando3c2 = demands.o3c2(k_con+1);            
    %         agentObs = [x./x_norm;...
    %                      [demando1c1,demando1c2]'./1000;...
    %                      [demando2c1,demando2c2]'./1000;...
    %                      [demando3c1,demando3c2]'./1000];       
    %         u_low = getAction(agent,agentObs);
    %         u_rl = u_low{1};
    % 
    %     end
    % 
    %     x = fun_benchmark_RM_nd(x,[u_seq_high(:,i);u_rl],k_con,param,scenario,demands);
    %     x_seq(:,i) = x;
    %     k_con = k_con + 1;
    % end
    % 
    % w_o_1_c1 = x_seq(64,:);
    % w_o_1_c2 = x_seq(65,:);
    % 
    % w_o_2_c1 = x_seq(68,:);
    % w_o_2_c2 = x_seq(69,:);
    % 
    % w_o_3_c1 = x_seq(72,:);
    % w_o_3_c2 = x_seq(73,:);
    % 
    % 
    % 
    % c = [(w_o_1_c1'+w_o_1_c2') - w_con(1); (w_o_2_c1'+w_o_2_c2') - w_con(2); (w_o_3_c1'+w_o_3_c2') - w_con(3)];
end
%%%