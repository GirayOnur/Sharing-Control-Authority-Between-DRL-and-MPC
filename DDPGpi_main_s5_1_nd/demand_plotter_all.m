
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


demands = filtfilt(b,a,Demands.o1c2(62:962)); % + Demands.o1c2(62:962);

plot(t,demands,"LineWidth",2)

demands = filtfilt(b,a,Demands.o2c2(62:962)); % + Demands.o2c2(62:962);

plot(t,demands,"LineWidth",2)
xlim([0 2.5])


%title("Vehicle demand vs time")
xlabel('Time [h]') 
ylabel('Demand [veh/h]')
legend
hold on

leg = legend('$d_{1,1}$','$d_{2,1},d_{3,1}$','$d_{1,2}$','$d_{2,2},d_{3,2}$');
set(leg,'Interpreter','latex');
set(leg,'FontSize',12);


