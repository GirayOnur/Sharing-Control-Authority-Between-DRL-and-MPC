function v_m_i_n = calc_v_m_i_n(v_m_i,rho_m_i,v_m_im1,rho_m_ip1,v_control,q_o,v_min,theta_m_i_c1,theta_m_i_c2,param,c_i,l_i)
% METANET speed update for one class in one segment. The terms are:
%   1. relaxation towards the desired speed for the current density
%   2. convection: speed carried in from the segment upstream
%   3. anticipation: drivers reacting to the density downstream
%   4. the slowdown caused by vehicles merging in from an on-ramp (q_o)
% Note the merging term sits outside the max(), so the result can end up
% below v_min when the ramp flow is large.

v_m_i_n = max(v_min, v_m_i + (param.T/param.tau)*(calc_V_tilde_rho_m_i(rho_m_i,theta_m_i_c1,theta_m_i_c2,v_control,param,c_i)-v_m_i)...
                + (param.T/param.L_m)*v_m_i*(v_m_im1 - v_m_i)...
                - (param.nu*param.T)/(param.tau*param.L_m)*(rho_m_ip1 - rho_m_i)/(rho_m_i + param.kappa))...
                - (param.delta*param.T*q_o*v_m_i)/(param.L_m*param.lambda.(l_i)*(rho_m_i+param.kappa));

end

