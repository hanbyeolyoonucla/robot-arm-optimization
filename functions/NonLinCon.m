function [c, ceq] = NonLinCon (X)

L1 = X(1);
L2 = X(2);
L3 = X(3);
L4 = X(4);
L5 = X(5);
x_WS = X(7);
y_WS = X(8);
z_WS = X(9);
Ltool = 0.140;
Lprime = sqrt(L3^2+L4^2);

c(1) = x_WS - L2 - Lprime;
c(2) = - L2 - Lprime - y_WS;
c(3) = y_WS - L2 - Lprime;
c(4) = L1-L2-Lprime-L5-Ltool - z_WS;
c(5) = z_WS - (L1+L2+Lprime+L5+Ltool);

ceq = [];
end