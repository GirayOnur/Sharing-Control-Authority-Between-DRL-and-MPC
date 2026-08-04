function x_n = fun_benchmark_RM_nd(x,u,k,param,scenario,demands)
% Same equations as fun_benchmark_RM, except the demand is read from a
% precomputed noisy profile instead of the nominal demand functions.
% demands holds one series per origin and class, indexed by sampling step.

v_control_max = param.v_control_max;
v_min = param.v_min;

%Unpack the state vector. Same order as it is packed again at the end.
v_1_1_c1 = x(1);
v_1_1_c2 = x(2);
rho_1_1_c1 = x(3);
rho_1_1_c2 = x(4);
rho_1_1_tot = x(5);
q_1_1_c1 = x(6);
q_1_1_c2 = x(7);

v_1_2_c1 = x(8);
v_1_2_c2 = x(9);
rho_1_2_c1 = x(10);
rho_1_2_c2 = x(11);
rho_1_2_tot = x(12);
q_1_2_c1 = x(13);
q_1_2_c2 = x(14);

v_1_3_c1 = x(15);
v_1_3_c2 = x(16);
rho_1_3_c1 = x(17);
rho_1_3_c2 = x(18);
rho_1_3_tot = x(19);
q_1_3_c1 = x(20);
q_1_3_c2 = x(21);

v_2_1_c1 = x(22);
v_2_1_c2 = x(23);
rho_2_1_c1 = x(24);
rho_2_1_c2 = x(25);
rho_2_1_tot = x(26);
q_2_1_c1 = x(27);
q_2_1_c2 = x(28);

v_3_1_c1 = x(29);
v_3_1_c2 = x(30);
rho_3_1_c1 = x(31);
rho_3_1_c2 = x(32);
rho_3_1_tot = x(33);
q_3_1_c1 = x(34);
q_3_1_c2 = x(35);

v_3_2_c1 = x(36);
v_3_2_c2 = x(37);
rho_3_2_c1 = x(38);
rho_3_2_c2 = x(39);
rho_3_2_tot = x(40);
q_3_2_c1 = x(41);
q_3_2_c2 = x(42);

v_4_1_c1 = x(43);
v_4_1_c2 = x(44);
rho_4_1_c1 = x(45);
rho_4_1_c2 = x(46);
rho_4_1_tot = x(47);
q_4_1_c1 = x(48);
q_4_1_c2 = x(49);

v_5_1_c1 = x(50);
v_5_1_c2 = x(51);
rho_5_1_c1 = x(52);
rho_5_1_c2 = x(53);
rho_5_1_tot = x(54);
q_5_1_c1 = x(55);
q_5_1_c2 = x(56);

v_5_2_c1 = x(57);
v_5_2_c2 = x(58);
rho_5_2_c1 = x(59);
rho_5_2_c2 = x(60);
rho_5_2_tot = x(61);
q_5_2_c1 = x(62);
q_5_2_c2 = x(63);

w_o_1_c1 = x(64);
w_o_1_c2 = x(65);
q_o_1_c1 = x(66);
q_o_1_c2 = x(67);

w_o_2_c1 = x(68);
w_o_2_c2 = x(69);
q_o_2_c1 = x(70);
q_o_2_c2 = x(71);

w_o_3_c1 = x(72);
w_o_3_c2 = x(73);
q_o_3_c1 = x(74);
q_o_3_c2 = x(75);


sr_c1 = u(1); %vehicle split rate for class 1, towards the 1st route
sr_c2 = u(1); %u(2); %vehicle split rate for class 2, towards the 1st route
rm_1 = u(2); %ramp metering rate for the 1st onramp
rm_2 = u(3); %ramp metering rate for the 2nd onramp

%%%
%theta is the fraction of the traffic volume held by each class in this
%segment. It is recomputed per segment further down.
theta_1_1_c1 = rho_1_1_c1/rho_1_1_tot;
theta_1_1_c2 = rho_1_1_c2/rho_1_1_tot;

rho_1_1_c1_n = calc_rho_m_i_n(rho_1_1_c1,q_o_1_c1,q_1_1_c1,param,'l1');
rho_1_1_c2_n = calc_rho_m_i_n(rho_1_1_c2,q_o_1_c2,q_1_1_c2,param,'l1');

rho_1_1_tot_n = rho_1_1_c1_n + rho_1_1_c2_n;

v_1_1_c1_n = calc_v_m_i_n(v_1_1_c1,rho_1_1_tot,v_1_1_c1,rho_1_2_tot,v_control_max,0,v_min,theta_1_1_c1,theta_1_1_c2,param,'c_1','l1');
v_1_1_c2_n = calc_v_m_i_n(v_1_1_c2,rho_1_1_tot,v_1_1_c2,rho_1_2_tot,v_control_max,0,v_min,theta_1_1_c1,theta_1_1_c2,param,'c_2','l1');

q_1_1_c1_n = rho_1_1_c1_n*v_1_1_c1_n*param.lambda.l1;
q_1_1_c2_n = rho_1_1_c2_n*v_1_1_c2_n*param.lambda.l1;

%%%

theta_1_2_c1 = rho_1_2_c1/rho_1_2_tot;
theta_1_2_c2 = rho_1_2_c2/rho_1_2_tot;

rho_1_2_c1_n = calc_rho_m_i_n(rho_1_2_c1,q_1_1_c1,q_1_2_c1,param,'l2');
rho_1_2_c2_n = calc_rho_m_i_n(rho_1_2_c2,q_1_1_c2,q_1_2_c2,param,'l2');

rho_1_2_tot_n = rho_1_2_c1_n + rho_1_2_c2_n;

v_1_2_c1_n = calc_v_m_i_n(v_1_2_c1,rho_1_2_tot,v_1_1_c1,rho_1_3_tot,v_control_max,0,v_min,theta_1_2_c1,theta_1_2_c2,param,'c_1','l2');
v_1_2_c2_n = calc_v_m_i_n(v_1_2_c2,rho_1_2_tot,v_1_1_c2,rho_1_3_tot,v_control_max,0,v_min,theta_1_2_c1,theta_1_2_c2,param,'c_2','l2');

q_1_2_c1_n = rho_1_2_c1_n*v_1_2_c1_n*param.lambda.l2;
q_1_2_c2_n = rho_1_2_c2_n*v_1_2_c2_n*param.lambda.l2;


%%%
theta_1_3_c1 = rho_1_3_c1/rho_1_3_tot;
theta_1_3_c2 = rho_1_3_c2/rho_1_3_tot;

rho_1_3_c1_n = calc_rho_m_i_n(rho_1_3_c1,q_1_2_c1,q_1_3_c1,param,'l3');
rho_1_3_c2_n = calc_rho_m_i_n(rho_1_3_c2,q_1_2_c2,q_1_3_c2,param,'l3');

rho_1_3_tot_n = rho_1_3_c1_n + rho_1_3_c2_n;

rho_1_4_tot = (rho_2_1_tot.^2 + rho_4_1_tot.^2) / (rho_2_1_tot + rho_4_1_tot);  %virtual downstream density for the last segment of the mainstream link, from the first segments of both leaving links

v_1_3_c1_n = calc_v_m_i_n(v_1_3_c1,rho_1_3_tot,v_1_2_c1,rho_1_4_tot,v_control_max,0,v_min,theta_1_3_c1,theta_1_3_c2,param,'c_1','l3');
v_1_3_c2_n = calc_v_m_i_n(v_1_3_c2,rho_1_3_tot,v_1_2_c2,rho_1_4_tot,v_control_max,0,v_min,theta_1_3_c1,theta_1_3_c2,param,'c_2','l3');

q_1_3_c1_n = rho_1_3_c1_n*v_1_3_c1_n*param.lambda.l3;
q_1_3_c2_n = rho_1_3_c2_n*v_1_3_c2_n*param.lambda.l3;


%%%
theta_2_1_c1 = rho_2_1_c1/rho_2_1_tot;
theta_2_1_c2 = rho_2_1_c2/rho_2_1_tot;

%The primary route gets the fraction sr of the flow leaving the mainstream
%link at the node, the secondary route gets 1-sr.
rho_2_1_c1_n = calc_rho_m_i_n(rho_2_1_c1,sr_c1*q_1_3_c1,q_2_1_c1,param,'l4');
rho_2_1_c2_n = calc_rho_m_i_n(rho_2_1_c2,sr_c2*q_1_3_c2,q_2_1_c2,param,'l4');

rho_2_1_tot_n = rho_2_1_c1_n + rho_2_1_c2_n;

v_2_1_c1_n = calc_v_m_i_n(v_2_1_c1,rho_2_1_tot,v_1_3_c1,rho_3_1_tot,v_control_max,0,v_min,theta_2_1_c1,theta_2_1_c2,param,'c_1','l4');
v_2_1_c2_n = calc_v_m_i_n(v_2_1_c2,rho_2_1_tot,v_1_3_c2,rho_3_1_tot,v_control_max,0,v_min,theta_2_1_c1,theta_2_1_c2,param,'c_2','l4');

q_2_1_c1_n = rho_2_1_c1_n*v_2_1_c1_n*param.lambda.l4;
q_2_1_c2_n = rho_2_1_c2_n*v_2_1_c2_n*param.lambda.l4;

%%%
%

%On-ramp O2 merges into this segment, so its outflow is added to the
%inflow and also passed to calc_v_m_i_n as the merging speed-drop term.
q_3_1_c1_in = q_o_2_c1 + q_2_1_c1;
q_3_1_c2_in = q_o_2_c2 + q_2_1_c2;

theta_3_1_c1 = rho_3_1_c1/rho_3_1_tot;
theta_3_1_c2 = rho_3_1_c2/rho_3_1_tot;

rho_3_1_c1_n = calc_rho_m_i_n(rho_3_1_c1,q_3_1_c1_in,q_3_1_c1,param,'l5');
rho_3_1_c2_n = calc_rho_m_i_n(rho_3_1_c2,q_3_1_c2_in,q_3_1_c2,param,'l5');

rho_3_1_tot_n = rho_3_1_c1_n + rho_3_1_c2_n;

v_3_1_c1_n = calc_v_m_i_n(v_3_1_c1,rho_3_1_tot,v_2_1_c1,rho_3_2_tot,v_control_max,q_o_2_c1+q_o_2_c2,v_min,theta_3_1_c1,theta_3_1_c2,param,'c_1','l5');
v_3_1_c2_n = calc_v_m_i_n(v_3_1_c2,rho_3_1_tot,v_2_1_c2,rho_3_2_tot,v_control_max,q_o_2_c1+q_o_2_c2,v_min,theta_3_1_c1,theta_3_1_c2,param,'c_2','l5');

q_3_1_c1_n = rho_3_1_c1_n*v_3_1_c1_n*param.lambda.l5;
q_3_1_c2_n = rho_3_1_c2_n*v_3_1_c2_n*param.lambda.l5;

%%%

rho_out_tot = param.rho_crit;  %downstream boundary at destination D1: traffic leaves at critical density

theta_3_2_c1 = rho_3_2_c1/rho_3_2_tot;
theta_3_2_c2 = rho_3_2_c2/rho_3_2_tot;

rho_3_2_c1_n = calc_rho_m_i_n(rho_3_2_c1,q_3_1_c1,q_3_2_c1,param,'l6');
rho_3_2_c2_n = calc_rho_m_i_n(rho_3_2_c2,q_3_1_c2,q_3_2_c2,param,'l6');

rho_3_2_tot_n = rho_3_2_c1_n + rho_3_2_c2_n;

v_3_2_c1_n = calc_v_m_i_n(v_3_2_c1,rho_3_2_tot,v_3_1_c1,rho_out_tot,v_control_max,0,v_min,theta_3_2_c1,theta_3_2_c2,param,'c_1','l6');
v_3_2_c2_n = calc_v_m_i_n(v_3_2_c2,rho_3_2_tot,v_3_1_c2,rho_out_tot,v_control_max,0,v_min,theta_3_2_c1,theta_3_2_c2,param,'c_2','l6');

q_3_2_c1_n = rho_3_2_c1_n*v_3_2_c1_n*param.lambda.l6;
q_3_2_c2_n = rho_3_2_c2_n*v_3_2_c2_n*param.lambda.l6;



%%%
theta_4_1_c1 = rho_4_1_c1/rho_4_1_tot;
theta_4_1_c2 = rho_4_1_c2/rho_4_1_tot;

rho_4_1_c1_n = calc_rho_m_i_n(rho_4_1_c1,(1-sr_c1)*q_1_3_c1,q_4_1_c1,param,'l7');
rho_4_1_c2_n = calc_rho_m_i_n(rho_4_1_c2,(1-sr_c2)*q_1_3_c2,q_4_1_c2,param,'l7');

rho_4_1_tot_n = rho_4_1_c1_n + rho_4_1_c2_n;

v_4_1_c1_n = calc_v_m_i_n(v_4_1_c1,rho_4_1_tot,v_1_3_c1,rho_5_1_tot,v_control_max,0,v_min,theta_4_1_c1,theta_4_1_c2,param,'c_1','l7');
v_4_1_c2_n = calc_v_m_i_n(v_4_1_c2,rho_4_1_tot,v_1_3_c2,rho_5_1_tot,v_control_max,0,v_min,theta_4_1_c1,theta_4_1_c2,param,'c_2','l7');

q_4_1_c1_n = rho_4_1_c1_n*v_4_1_c1_n*param.lambda.l7;
q_4_1_c2_n = rho_4_1_c2_n*v_4_1_c2_n*param.lambda.l7;

%%%
q_5_1_c1_in = q_o_3_c1 + q_4_1_c1;
q_5_1_c2_in = q_o_3_c2 + q_4_1_c2;

theta_5_1_c1 = rho_5_1_c1/rho_5_1_tot;
theta_5_1_c2 = rho_5_1_c2/rho_5_1_tot;

rho_5_1_c1_n = calc_rho_m_i_n(rho_5_1_c1,q_5_1_c1_in,q_5_1_c1,param,'l8');
rho_5_1_c2_n = calc_rho_m_i_n(rho_5_1_c2,q_5_1_c2_in,q_5_1_c2,param,'l8');

rho_5_1_tot_n = rho_5_1_c1_n + rho_5_1_c2_n;

v_5_1_c1_n = calc_v_m_i_n(v_5_1_c1,rho_5_1_tot,v_4_1_c1,rho_5_2_tot,v_control_max,q_o_3_c1+q_o_3_c2,v_min,theta_5_1_c1,theta_5_1_c2,param,'c_1','l8');
v_5_1_c2_n = calc_v_m_i_n(v_5_1_c2,rho_5_1_tot,v_4_1_c2,rho_5_2_tot,v_control_max,q_o_3_c1+q_o_3_c2,v_min,theta_5_1_c1,theta_5_1_c2,param,'c_2','l8');

q_5_1_c1_n = rho_5_1_c1_n*v_5_1_c1_n*param.lambda.l8;
q_5_1_c2_n = rho_5_1_c2_n*v_5_1_c2_n*param.lambda.l8;

%%%
rho_out_tot = param.rho_crit;

theta_5_2_c1 = rho_5_2_c1/rho_5_2_tot;
theta_5_2_c2 = rho_5_2_c2/rho_5_2_tot;

rho_5_2_c1_n = calc_rho_m_i_n(rho_5_2_c1,q_5_1_c1,q_5_2_c1,param,'l9');
rho_5_2_c2_n = calc_rho_m_i_n(rho_5_2_c2,q_5_1_c2,q_5_2_c2,param,'l9');

rho_5_2_tot_n = rho_5_2_c1_n + rho_5_2_c2_n;

v_5_2_c1_n = calc_v_m_i_n(v_5_2_c1,rho_5_2_tot,v_5_1_c1,rho_out_tot,v_control_max,0,v_min,theta_5_2_c1,theta_5_2_c2,param,'c_1','l9');
v_5_2_c2_n = calc_v_m_i_n(v_5_2_c2,rho_5_2_tot,v_5_1_c2,rho_out_tot,v_control_max,0,v_min,theta_5_2_c1,theta_5_2_c2,param,'c_2','l9');

q_5_2_c1_n = rho_5_2_c1_n*v_5_2_c1_n*param.lambda.l9;
q_5_2_c2_n = rho_5_2_c2_n*v_5_2_c2_n*param.lambda.l9;



%%%
d_o_1_c1 = demands.o1c1(k+1);
d_o_1_c2 = demands.o1c2(k+1);
d_o_1_c1_n = demands.o1c1(k+2);
d_o_1_c2_n = demands.o1c2(k+2);

w_o_1_c1_n = calc_w_o_n(w_o_1_c1,d_o_1_c1,q_o_1_c1,param);
w_o_1_c2_n = calc_w_o_n(w_o_1_c2,d_o_1_c2,q_o_1_c2,param);

q_des_o_1_c1_n = d_o_1_c1_n + (w_o_1_c1_n/param.T);
q_des_o_1_c2_n = d_o_1_c2_n + (w_o_1_c2_n/param.T);
q_des_o_1_tot_n = q_des_o_1_c1_n + q_des_o_1_c2_n;

q_o_1_c1_n = calc_q_o_main(d_o_1_c1_n,w_o_1_c1_n,rho_1_1_tot_n,q_des_o_1_c1_n,q_des_o_1_tot_n,param);
q_o_1_c2_n = calc_q_o_main(d_o_1_c2_n,w_o_1_c2_n,rho_1_1_tot_n,q_des_o_1_c2_n,q_des_o_1_tot_n,param);


%
d_o_2_c1 = demands.o2c1(k+1);
d_o_2_c2 = demands.o2c2(k+1);
d_o_2_c1_n = demands.o2c1(k+2);
d_o_2_c2_n = demands.o2c2(k+2);

w_o_2_c1_n = calc_w_o_n(w_o_2_c1,d_o_2_c1,q_o_2_c1,param);
w_o_2_c2_n = calc_w_o_n(w_o_2_c2,d_o_2_c2,q_o_2_c2,param);


q_des_o_2_c1_n = d_o_2_c1_n + (w_o_2_c1_n/param.T);
q_des_o_2_c2_n = d_o_2_c2_n + (w_o_2_c2_n/param.T);
q_des_o_2_tot_n = q_des_o_2_c1_n + q_des_o_2_c2_n;


q_o_2_c1_n = calc_q_o_ramp(d_o_2_c1_n,w_o_2_c1_n,rm_1,rho_3_1_tot_n,q_des_o_2_c1_n,q_des_o_2_tot_n,param);
q_o_2_c2_n = calc_q_o_ramp(d_o_2_c2_n,w_o_2_c2_n,rm_1,rho_3_1_tot_n,q_des_o_2_c2_n,q_des_o_2_tot_n,param);


%
d_o_3_c1 = demands.o3c1(k+1);
d_o_3_c2 = demands.o3c2(k+1);
d_o_3_c1_n = demands.o3c1(k+2);
d_o_3_c2_n = demands.o3c2(k+2);

w_o_3_c1_n = calc_w_o_n(w_o_3_c1,d_o_3_c1,q_o_3_c1,param);
w_o_3_c2_n = calc_w_o_n(w_o_3_c2,d_o_3_c2,q_o_3_c2,param);


q_des_o_3_c1_n = d_o_3_c1_n + (w_o_3_c1_n/param.T);
q_des_o_3_c2_n = d_o_3_c2_n + (w_o_3_c2_n/param.T);
q_des_o_3_tot_n = q_des_o_3_c1_n + q_des_o_3_c2_n;


q_o_3_c1_n = calc_q_o_ramp(d_o_3_c1_n,w_o_3_c1_n,rm_2,rho_5_1_tot_n,q_des_o_3_c1_n,q_des_o_3_tot_n,param);
q_o_3_c2_n = calc_q_o_ramp(d_o_3_c2_n,w_o_3_c2_n,rm_2,rho_5_1_tot_n,q_des_o_3_c2_n,q_des_o_3_tot_n,param);


%%%
x_n  =  [v_1_1_c1_n;
        v_1_1_c2_n;
        rho_1_1_c1_n;
        rho_1_1_c2_n;
        rho_1_1_tot_n;
        q_1_1_c1_n;
        q_1_1_c2_n;
        v_1_2_c1_n;
        v_1_2_c2_n;
        rho_1_2_c1_n;
        rho_1_2_c2_n;
        rho_1_2_tot_n;
        q_1_2_c1_n;
        q_1_2_c2_n;
        v_1_3_c1_n;
        v_1_3_c2_n;
        rho_1_3_c1_n;
        rho_1_3_c2_n;
        rho_1_3_tot_n;
        q_1_3_c1_n;
        q_1_3_c2_n;
        v_2_1_c1_n;
        v_2_1_c2_n;
        rho_2_1_c1_n;
        rho_2_1_c2_n;
        rho_2_1_tot_n;
        q_2_1_c1_n;
        q_2_1_c2_n;
        v_3_1_c1_n;
        v_3_1_c2_n;
        rho_3_1_c1_n;
        rho_3_1_c2_n;
        rho_3_1_tot_n;
        q_3_1_c1_n;
        q_3_1_c2_n;
        v_3_2_c1_n;
        v_3_2_c2_n;
        rho_3_2_c1_n;
        rho_3_2_c2_n;
        rho_3_2_tot_n;
        q_3_2_c1_n;
        q_3_2_c2_n;
        v_4_1_c1_n;
        v_4_1_c2_n;
        rho_4_1_c1_n;
        rho_4_1_c2_n;
        rho_4_1_tot_n;
        q_4_1_c1_n;
        q_4_1_c2_n;
        v_5_1_c1_n;
        v_5_1_c2_n;
        rho_5_1_c1_n;
        rho_5_1_c2_n;
        rho_5_1_tot_n;
        q_5_1_c1_n;
        q_5_1_c2_n;
        v_5_2_c1_n;
        v_5_2_c2_n;
        rho_5_2_c1_n;
        rho_5_2_c2_n;
        rho_5_2_tot_n;
        q_5_2_c1_n;
        q_5_2_c2_n;
        w_o_1_c1_n;
        w_o_1_c2_n;
        q_o_1_c1_n;
        q_o_1_c2_n;
        w_o_2_c1_n;
        w_o_2_c2_n;
        q_o_2_c1_n;
        q_o_2_c2_n;
        w_o_3_c1_n;
        w_o_3_c2_n;
        q_o_3_c1_n;
        q_o_3_c2_n...
                   ];

end

