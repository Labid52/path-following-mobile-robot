clc
close all
clear

%% Path Planning Astar
pathplanningAstar;
%% Initial and final goal location and other parameters

x_st = 0;
y_st = 0;
theta0 = 0; %45*pi/180;
x_gl = 20;
y_gl = 20;
thetag = 0;
L = 2;
dt = 0.1;
tf = x_gl;
%% path planning polynomial
xss = [x_st]; yss = [y_st];
xgg = [x_gl]; ygg = [y_gl];
x0 = xss(1);y0 = yss(1);xg = xgg(1); yg = ygg(1);
path_f = [0 0 0];
for i = 1:length(xss)
    x0 = xss(i);y0 = yss(i);xg = xgg(i); yg = ygg(i);
    F = @(P) [y0-P(1)-P(2).*x0-P(3)*x0.^2-P(4).*x0^3;
             yg-P(1)-P(2).*xg-P(3)*xg.^2-P(4).*xg^3;
             tan(theta0)-P(2)-2*P(3)*x0-3*P(4).*x0.^2;
             tan(thetag)-P(2)-2*P(3)*xg-3*P(4).*xg^2];
    f0 = rand(1,4);
    P = fsolve(F,f0);
    x = x0:0.1:xg;
    for j =1:length(x)
        y(j) = P(1) + P(2)*x(j) + P(3)*x(j).^2 + P(4)*x(j).^3;
        thetad(j) = atan(P(2)+2*P(3)*x(j)+3*P(4)*x(j).^2);
     end
     xx = x';
     yy =y';
     tt = thetad';
     ss = [xx yy tt];
     path_f = [path_f;ss(2:end,:)]
    clear x y thetad xx yy tt gg;
end
xd = path_f(:,1);
yd =path_f(:,2);
thetad = path_f(:,3);

dtheta = thetad(2:end)-thetad(1:end-1);
dx = xd(2:end) - xd(1:end-1);
dxdt = dx/dt;
dy = yd(2:end) - yd(1:end-1);
dydt = dy/dt;
vd = sqrt(dxdt.^2+dydt.^2);
deltad = atan((L./vd).*dtheta);

%% controller 
t = 0:dt:tf;
n = length(t);
h = t(2) - t(1);
x(1) = x0;
y(1)=y0;
theta(1)=theta0;
K_x = 0.6829;
K_y =0.2226;
K_theta = 2.6280;
% K_x = 1.3369;
% K_y = 0.2021;
% K_theta = 0.7290;

for i = 1:n-1
    x_e(i) = (xd(i) - x(i))*cos(theta(i)) + (yd(i)- y(i))*sin(theta(i));
    y_e(i) = -(xd(i) - x(i))*sin(theta(i)) + (yd(i)- y(i))*cos(theta(i));
    theta_e(i) = thetad(i) - theta(i);


    v(i) = vd(i)*cos(theta_e(i)) + K_x*x_e(i);
    delta(i) = atan((L/(v(i)))*(thetad(i) + (vd(i)/K_y)*(K_y*y_e(i) + K_theta*sin(theta_e(i))-h*theta(i))));

  
    x(i+1) = x(i) + h*v(i).*cos(theta(i));
    y(i+1) = y(i) + h*v(i).*sin(theta(i));
    theta(i+1) = theta(i) + h*v(i)*tan(delta(i))/L; 

    E_tot(i) = sqrt((xd(i)-x(i))^2+(yd(i)-y(i))^2+(thetad(i)-theta(i))^2);
end
RMSE = sum(E_tot);
disp(RMSE);
%% visualization
figure
plot(x,y,'Color','r','LineWidth',1.2);grid on
hold on
plot(xd,yd,'Color','b','LineWidth',1.2)
xlabel('X'); ylabel('Y');
legend('Path Controlled by Robot','Reference Path')
hold off
figure
plot(t(1:end-1),x_e,'LineWidth',1.2); grid on;
hold on
plot(t(1:end-1),y_e,'LineWidth',1.2);hold on
plot(t(1:end-1),theta_e,'LineWidth',1.2);
xlabel('Time'); ylabel('Error');
legend('X error', 'Y error','Theta error')
hold off
figure
plot(x(1:end-1),v,'LineWidth',1.2);grid on
hold on
plot(x(1:end-1),vd,'LineWidth',1.2);
xlabel('X'); ylabel('Speed');
legend('v','vd');hold off
figure
plot(x(1:end-1),delta,'LineWidth',1.2);grid on
hold on
plot(x(1:end-1),deltad,'LineWidth',1.2);
xlabel('X'); ylabel('Steering Angle');
legend('delta','deltad');hold off



