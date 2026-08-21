function q_o_main = calc_q_o_main(d_o,w_o,rho_m_1,q_des_o_1_ci_n,q_des_o_1_tot_n,param)
% Outflow of a mainstream origin, the smallest of:
%   1. the desired outflow q_des (demand plus the waiting queue)
%   2. the free-flow capacity of the origin, aggregated over the lanes of
%      the link it feeds (Q_o_main)
%   3. what the first segment of that link can still take at its current
%      density
% The q_des ratio distributes the total over the two vehicle classes.

% q_o_main = min([d_o + w_o/param.T, param.lambda.l1*(q_des_o_1_ci_n/q_des_o_1_tot_n)*param.Q_o*...
%                                         (param.rho_max - rho_m_1)/(param.rho_max - param.rho_crit)]);

q_o_main = min([d_o + w_o/param.T, (q_des_o_1_ci_n/q_des_o_1_tot_n)*param.Q_o_main, (q_des_o_1_ci_n/q_des_o_1_tot_n)*param.Q_o_main*...
                                        (param.rho_max - rho_m_1)/(param.rho_max - param.rho_crit)]);


end

