function G = objective(K)
x(1,:) = -5;
y(1,:) = 50;
theta(1,:) = pi/2+0.1;

xg = 0;
yg = 0;
thetag = pi/2;

krho = 1;
kalpha = 5;
kphi = -2;

h = 0.01;
dr = 0;
l = 11.29;
t = 0;



%% simulation

for i = 1:1000
    delx(i,:) = real(x(i,:)-xg);
    dely(i,:) = real(y(i,:)-yg);

    rho(i,:) = sqrt(delx(i,:)^2+dely(i,:)^2);

    if dr == 0
        phi(i,:) = -atan2(-dely(i,:),-delx(i,:));
        alpha(i,:) = -theta(i,:)-phi(i,:);

        if alpha(i,:) > pi/2 || alpha(i,:) < -pi/2
            dr = -1;
            fprintf('backward motion\n');
        else
            dr = 1;
             fprintf('going forwards\n');
        end
    elseif dr ==-1
        phi(i,:) = -atan2(dely(i,:),delx(i,:));
        alpha(i,:) = -theta(i,:)-phi(i,:);
    else
        phi(i,:) = -atan2(-dely(i,:),-delx(i,:));
        alpha(i,:) = -theta(i,:)-phi(i,:);
    end

    if alpha(i,:) > pi/2
        alpha(i,:) = pi/2;
    end
    if alpha(i,:) < -pi/2
        alpha(i,:) = -pi/2;
    end

    phi(i,:) = phi(i,:) + thetag;
    
    d1(i,:)=rand;d2(i,:) = rand;d3(i,:) = rand;


    v(i,:) = krho*dr*rho(i,:);
    omega(i,:) = dr*(kalpha*alpha(i,:)+kphi*phi(i,:))/abs(v(i,:));
    delta(i,:) = atan(omega(i,:));

    x(i+1,:) = x(i,:) + h*(v(i,:)*cos(theta(i,:)));
    y(i+1,:) = y(i,:) + h*(v(i,:)*sin(theta(i,:)));
    theta(i+1,:) = theta(i,:) + h*((v(i,:)/l).*tan(delta(i,:)));

    X(i+1,:) = [x(i+1,:) y(i+1,:) theta(i+1,:)];
    t(i+1,:) = t(i,:) + h;
    
%     E_tot(i) = sqrt(x_e(i)^2+y_e(i)^2+theta_e(i)^2);
    E_tot(i) = sqrt((xd(i)-x(i))^2+(yd(i)-y(i))^2+(thetad(i)-theta(i))^2);
end

G = sum(E_tot);
end