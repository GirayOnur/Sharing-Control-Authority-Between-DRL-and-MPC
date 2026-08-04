%Plots the base demand of all origins and classes in one figure, the version
%used in the paper. Index 62:962 skips the 60 warm-up steps.

kf = 900;
scenario = 3;

t = linspace(0,900,901).*10./3600;



base_demands = load('base_demands.mat');

Demands.o1c1 = base_demands.base_demand_o1c1;
Demands.o1c2 = base_demands.base_demand_o1c2;
Demands.o2c1 = base_demands.base_demand_o2c1;
Demands.o2c2 = base_demands.base_demand_o2c2;
Demands.o3c1 = base_demands.base_demand_o3c1;
Demands.o3c2 = base_demands.base_demand_o3c2;

[b,a] = butter(1,0.1);


%%%mismatched MPC demands %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scenario = 3;
N_demand = 1030;


base_demand_o1c1mm = nan(1,N_demand);
base_demand_o1c2mm = nan(1,N_demand);
base_demand_o2c1mm = nan(1,N_demand);
base_demand_o2c2mm = nan(1,N_demand);
base_demand_o3c1mm = nan(1,N_demand);
base_demand_o3c2mm = nan(1,N_demand);

for k = 1:N_demand
    [base_demand_o1c1mm(1,k),base_demand_o1c2mm(1,k)] = demando1mm(k-2, scenario);
    [base_demand_o2c1mm(1,k),base_demand_o2c2mm(1,k)] = demando2mm(k-2, scenario);
    [base_demand_o3c1mm(1,k),base_demand_o3c2mm(1,k)] = demando3mm(k-2, scenario);
end


%%figure mainstream demads %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)

demands = filtfilt(b,a,Demands.o1c1(62:962)); % + Demands.o1c2(62:962);
% upperCurve11 = demands + 200;
% lowerCurve11 = demands - 200;

plot(t,demands, 'Color', "#1171BE","LineWidth",2)
%title("Vehicle demand vs time")
xlabel('Time [h]') 
ylabel('Demand [veh/h]')
legend
hold on

% areaX = [t, fliplr(t)];
% areaY = [lowerCurve11, fliplr(upperCurve11)];
% shadedArea = patch("XData",areaX, "YData",areaY, ...
%     "FaceColor",'flat',"EdgeColor",'flat',"SeriesIndex",2, ...
%     "FaceAlpha",0.2, "EdgeAlpha",1);


demands = filtfilt(b,a,Demands.o1c2(62:962)); % + Demands.o1c2(62:962);

plot(t,demands,'Color', "#AA3232", "LineWidth",2)
% areaX = [t, fliplr(t)];
% areaY = [lowerCurve12, fliplr(upperCurve12)];
% shadedArea = patch("XData",areaX, "YData",areaY, ...
%     "FaceColor",'flat',"EdgeColor",'flat',"SeriesIndex",2, ...
%     "FaceAlpha",0.2, "EdgeAlpha",1);

demands = filtfilt(b,a,base_demand_o1c1mm(62:962)); % + Demands.o1c2(62:962);
% upperCurve11 = demands + 200;
% lowerCurve11 = demands - 200;

plot(t,demands, '--', 'Color', "#1171BE","LineWidth",2)

demands = filtfilt(b,a,base_demand_o1c2mm(62:962)); % + Demands.o1c2(62:962);

plot(t,demands,'--','Color', "#AA3232","LineWidth",2)
xlim([0 2.5])
ylim([0 6200])

leg = legend('$d_{1,1}$','$d_{1,2}$','$\tilde{d}_{1,1}$','$\tilde{d}_{1,2}$');
set(leg,'Interpreter','latex');
set(leg,'FontSize',12);



%%figure on-ramp demands %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(2)

demands = filtfilt(b,a,Demands.o2c1(62:962)); % + Demands.o2c2(62:962);
% upperCurve12 = demands + 50;
% lowerCurve12 = demands - 50;
% 
plot(t,demands,'Color', "#3BAA32","LineWidth",2)
% areaX = [t, fliplr(t)];
% areaY = [lowerCurve12, fliplr(upperCurve12)];
% shadedArea = patch("XData",areaX, "YData",areaY, ...
%     "FaceColor",'flat',"EdgeColor",'flat',"SeriesIndex",2, ...
%     "FaceAlpha",0.2, "EdgeAlpha",1);
%title("Vehicle demand vs time")
xlabel('Time [h]') 
ylabel('Demand [veh/h]')
legend
hold on

demands = filtfilt(b,a,Demands.o2c2(62:962)); % + Demands.o2c2(62:962);

plot(t,demands,'Color', "#FF880F", "LineWidth",2)

demands = filtfilt(b,a,base_demand_o2c1mm(62:962)); % + Demands.o2c2(62:962);
% upperCurve12 = demands + 50;
% lowerCurve12 = demands - 50;
% 
plot(t,demands,'--','Color', "#3BAA32","LineWidth",2)
% areaX = [t, fliplr(t)];
% areaY = [lowerCurve12, fliplr(upperCurve12)];
% shadedArea = patch("XData",areaX, "YData",areaY, ...
%     "FaceColor",'flat',"EdgeColor",'flat',"SeriesIndex",2, ...
%     "FaceAlpha",0.2, "EdgeAlpha",1);


demands = filtfilt(b,a,base_demand_o2c2mm(62:962)); % + Demands.o2c2(62:962);

plot(t,demands,'--','Color', "#FF880F","LineWidth",2)
xlim([0 2.5])
ylim([0 1120])

leg = legend('$d_{2,1},d_{3,1}$','$d_{2,2},d_{3,2}$','$\tilde{d}_{2,1},\tilde{d}_{3,1}$','$\tilde{d}_{2,2},\tilde{d}_{3,2}$');
set(leg,'Interpreter','latex');
set(leg,'FontSize',12);