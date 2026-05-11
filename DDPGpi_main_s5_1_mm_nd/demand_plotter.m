

kf = 900;
scenario = 3;

for i=1:kf
    [demandc1,demandc2] = demando1(i,scenario);
    demand = demandc1 + demandc2;
    plot(i,demand,'o')
    hold on
end


