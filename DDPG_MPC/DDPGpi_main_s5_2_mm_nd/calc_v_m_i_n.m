function v_m_i_n = calc_v_m_i_n(v_m_i,rho_m_i,v_m_im1,rho_m_ip1,v_control,q_o,v_min,theta_m_i_c1,theta_m_i_c2,param,c_i,l_i)
% Segment mean speed update for one vehicle class. The terms are:
%   1. relaxation towards the interpolated speed-density relationship
%   2. convection from the upstream segment
%   3. anticipation of the downstream density
%   4. the speed drop from merging on-ramp traffic, applied only on links
%      that carry an on-ramp (q_o is zero elsewhere)
% param.tau, param.nu, param.kappa and param.delta are tau_c, eta_c,
% kappa_c and sigma_c of the model.
% Note the merging term sits outside the max(), so the result can end up
% below v_min when the ramp flow is large.

v_m_i_n = max(v_min, v_m_i + (param.T/param.tau)*(calc_V_tilde_rho_m_i(rho_m_i,theta_m_i_c1,theta_m_i_c2,v_control,param,c_i)-v_m_i)...
                + (param.T/param.L_m)*v_m_i*(v_m_im1 - v_m_i)...
                - (param.nu*param.T)/(param.tau*param.L_m)*(rho_m_ip1 - rho_m_i)/(rho_m_i + param.kappa))...
                - (param.delta*param.T*q_o*v_m_i)/(param.L_m*param.lambda.(l_i)*(rho_m_i+param.kappa));

end

