%Old single-class version of the simulation, kept for reference only. It
%calls fun_benchmark, which is no longer part of this repository, so it does
%not run as is.
clear
clc


param_sim = param_get;

x=zeros(22,1);
u=[200,200,1];
scenario=3;
N=900;
k = 0;
for i=1:60
    x=fun_benchmark(x,u,k,param_sim,scenario);
    k = k + 1;
end
xx=[];

for i=1:N
    x=fun_benchmark(x,u,k,param_sim,scenario);
    xx=[xx x];
    k = k+1;
end

v_11=xx(1,:);
rho_11=xx(2,:);
q_11=xx(3,:);

v_12=xx(4,:);
rho_12=xx(5,:);
q_12=xx(6,:);

v_13=xx(7,:);
rho_13=xx(8,:);
q_13=xx(9,:);

v_14=xx(10,:);
rho_14=xx(11,:);
q_14=xx(12,:);

v_21=xx(13,:);
rho_21=xx(14,:);
q_21=xx(15,:);

v_22=xx(16,:);
rho_22=xx(17,:);
q_22=xx(18,:);

w_o1=xx(19,:);
q_o1 = xx(20,:);

w_o2=xx(21,:);
q_o2=xx(22,:);

TTS=10/3600.*((rho_11+rho_12+rho_13+rho_14+rho_21+rho_22).*1000./1000.*2+w_o1+w_o2);
%%
Rho=[rho_22;rho_21;rho_14;rho_13;rho_12;rho_11];
Velocity=[v_22;v_21;v_14;v_13;v_12;v_11];
Flow=[q_22;q_21;q_14;q_13;q_12;q_11];

fprintf('TTS is %.3f veh*h \n', sum(TTS))
fprintf('Average flow is %.3f veh*h \n', mean(Flow, 'all'))
fprintf('Max flow is %.3f veh*h \n', max(Flow, [], 'all'))
fprintf('Minimal speed is %.3f km/h \n', min(Velocity, [], 'all'));
