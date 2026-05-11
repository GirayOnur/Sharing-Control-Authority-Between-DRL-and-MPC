function V_rho_m_i = calc_V_rho_m_i(rho_m_i,v_control,param,c_i)

V_rho_m_i = min(param.v_free.(c_i)*exp((-1/param.a_m.(c_i))*(rho_m_i/param.rho_crit)^param.a_m.(c_i)), v_control);


end

