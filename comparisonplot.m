L1 = x(1);
L2 = x(2);
L3 = x(3);
L4 = x(4);
L5 = x(5);
alpha = x(6);
x_WS = x(7);
y_WS = x(8);
z_WS = x(9);

% 5. Stiffness Normalization
stiffMax = 15*0.3^3;
stiffMin = 0;
stiffMECA = (0.135^3+2*0.135^3+3*0.038^3+4*0.120^3+5*0.07^3);
stiffMECANormalized = (stiffMax - stiffMECA) / (stiffMax - stiffMin);
for ii = 1:length(gapopulationhistory(1,1,:))
    L1 = gapopulationhistory(:,1,ii);
    L2 = gapopulationhistory(:,2,ii);
    L3 = gapopulationhistory(:,3,ii);
    L4 = gapopulationhistory(:,4,ii);
    L5 = gapopulationhistory(:,5,ii);
    stiff = L1.^3+2*L2.^3+3*L3.^3+4*L4.^3+5*L5.^3;
    stiffnessNormalized(:,ii) = (stiffMax - stiff) / (stiffMax - stiffMin);
end

figure(5)
hold on
grid on
% p1 = plot(stiffnessNormalized(1,1),-gascorehistory(1,1),'m.');
plot(stiffnessNormalized,-gascorehistory,'r.');
p2 = plot(stiffMECANormalized,0.0220,'b*');
p3 = plot(stiffMECANormalized,0,'g*');
xlabel('Stiffness')
ylabel('Total Score')
title('Population Distribution and MECA')
legend([p1 p2 p3],{'Populations','MECA(axial cut only)','MECA'})

