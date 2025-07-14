clc
clear 
close all
x0 = 0;
y0 = 0;
theta0 = 0;%45*pi/180;
xg = 10;
yg = 12;
thetag = 0;
L = 2;
dt = 0.5;


F = @(P) [y0-P(1)-P(2)*x0-P(3)*x0^2-P(4)*x0^3;
         yg-P(1)-P(2)*xg-P(3)*xg^2-P(4)*xg^3;
         tan(theta0)-P(2)-2*P(3)*x0-3*P(4)*x0^2;
         tan(thetag)-P(2)-2*P(3)*xg-3*P(4)*xg^2];

f0 = rand(1,4);
P = fsolve(F,f0);

x = x0:0.5:xg;
y = P(1) + P(2)*x + P(3)*x.^2 + P(4)*x.^3;
theta = atan(P(2)+2*P(3)*x+3*P(4)*x.^2);
dtheta = theta(2:end)-theta(1:end-1);
dx = x(2:end) - x(1:end-1);
dxdt = dx/dt;
dy = y(2:end) - y(1:end-1);
dydt = dy/dt;
v = sqrt(dxdt.^2+dydt.^2);
del = atan((L./v).*dtheta);
nonhol = dxdt.*sin(theta(1:end-1))-dydt.*cos(theta(1:end-1));
figure
plot(x,y)
title('Path')
figure
plot(x,theta*180/pi)
title('heading')
figure
plot(x(1:end-1),v)
title('speed')
figure
plot(180*del/pi)
title('steering')
figure
plot(nonhol)
title('nonholonomic value')
% for i = length(theta)-1
%     dtheta(i) = theta(i+1)-theta(i)
% end
dd = dxdt.*tan(theta(1:end-1))
nonhol2 = dxdt.*sin(theta(1:end-1))-dd.*cos(theta(1:end-1))


