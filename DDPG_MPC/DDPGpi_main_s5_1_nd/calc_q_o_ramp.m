function q_o_ramp = calc_q_o_ramp(d_o,w_o,r_o,rho_m_1,q_des_o_1_ci_n,q_des_o_1_tot_n,param)
% Same as calc_q_o_main but for a metered on-ramp. r_o is the metering
% rate in [0,1] and scales the ramp capacity directly: r_o = 1 is a fully
% open ramp, r_o = 0 closes it.

q_o_ramp = min([d_o + w_o/param.T, (q_des_o_1_ci_n/q_des_o_1_tot_n)*param.Q_o*r_o,...
    (q_des_o_1_ci_n/q_des_o_1_tot_n)*param.Q_o*(param.rho_max - rho_m_1)/(param.rho_max - param.rho_crit)]);

end

