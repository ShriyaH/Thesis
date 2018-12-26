mu=1.326807104000000e+20;
kep_orbit(1,1) = 6787746.891;
kep_orbit(1,2) = 0.000731104;
kep_orbit(1,3) = 51.68714486*pi /180;
kep_orbit(1,4) = 127.5486706*pi /180;
kep_orbit(1,5) = 74.21987137*pi /180;
kep_orbit(1,6) = 24.06608426*pi /180;

[state,theta,state_p,theta_p] = kep2cart(kep_orbit,mu);


Y0 = [state(1:3,1);state(5:7,1)];

T = [0 2*86400];

Opt = odeset('Events', @stopevent);
[t,y,te,ye,ie] = ode45(@orb_int, T, Y0,Opt); 
for i = 1:length(y)
d(i,:)=norm(y(i,1:3));
e(i,:)=norm(y(i,4:6));
end


uw = d(end,end)-6371e3;
comet3(y(:,1),y(:,2),y(:,3))


function [value, isterminal, direction] = stopevent(T,Y)
imp = sqrt(Y(1)^2 + Y(2)^2 + Y(3)^2)                                     %Maximum allowable acceleration at that position
value = imp-6371e3    %Impact condition, Escape condition
if abs(value)<=1e-2
    value = 0;
end
isterminal = 1;          %Stop the integration
direction  = -1;          %Bi-directional approach
end


function dY = orb_int(T,Y)
 mu=1.326807104000000e+20;
r = sqrt(Y(1)^2 + Y(2)^2 + Y(3)^2);
dY = [Y(4); Y(5); Y(6); -mu*Y(1)/r^3; -mu*Y(2)/r^3; -mu*Y(3)/r^3];
end

