function q = calc_q(rho_m_i,v_m_i,param)
% Segment outflow of one vehicle class: density * mean speed * lanes.
% Not called anywhere, fun_benchmark_RM writes this product out inline.

q = rho_m_i*v_m_i*param.lambda;

end

