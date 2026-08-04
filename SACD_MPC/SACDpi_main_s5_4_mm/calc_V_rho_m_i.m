function V_rho_m_i = calc_V_rho_m_i(rho_m_i,v_control,param,c_i)
% Desired speed of one class at a given density (the fundamental diagram).
% v_control is the speed limit; here it is always v_control_max, so it only
% acts as a cap. Speed limits are not one of the controlled inputs.

V_rho_m_i = min(param.v_free.(c_i)*exp((-1/param.a_m.(c_i))*(rho_m_i/param.rho_crit)^param.a_m.(c_i)), v_control);


end

