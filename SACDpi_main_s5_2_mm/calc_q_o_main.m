function q_o_main = calc_q_o_main(d_o,w_o,rho_m_1,q_des_o_1_ci_n,q_des_o_1_tot_n,param)

% q_o_main = min([d_o + w_o/param.T, param.lambda.l1*(q_des_o_1_ci_n/q_des_o_1_tot_n)*param.Q_o*...
%                                         (param.rho_max - rho_m_1)/(param.rho_max - param.rho_crit)]);

q_o_main = min([d_o + w_o/param.T, (q_des_o_1_ci_n/q_des_o_1_tot_n)*param.Q_o_main, (q_des_o_1_ci_n/q_des_o_1_tot_n)*param.Q_o_main*...
                                        (param.rho_max - rho_m_1)/(param.rho_max - param.rho_crit)]);


end

