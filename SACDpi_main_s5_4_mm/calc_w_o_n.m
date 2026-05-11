function w_o_n = calc_w_o_n(w_o,d_o,q_o,param)

w_o_n = w_o + param.T*(d_o - q_o);

end

