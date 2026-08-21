function V_rho_m_i = calc_V_rho_m_i(rho_m_i,v_control,param,c_i)
% Desired speed of the drivers of one vehicle class at a given total
% segment density, from the free-flow speed and the exponent a_m of that
% class. v_control is a speed limit cap; here it is always v_control_max,
% since variable speed limits are not one of the control measures.

V_rho_m_i = min(param.v_free.(c_i)*exp((-1/param.a_m.(c_i))*(rho_m_i/param.rho_crit)^param.a_m.(c_i)), v_control);


end

