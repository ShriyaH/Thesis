function  [X,theta_rad,t] = Propagate_Orbit( kepler_orbit, state, mu,tspan)
%This function is to propagate the Kepler orbit subject to only gravity 
%tspan = [0 T+7*24*3600]

% T = 2*pi/kepler_orbit(1,7);
T = 15*3600;

function dx = kepode(t,x)
r = sqrt(x(1)^2 + x(2)^2 + x(3)^2);
dx = [x(4); x(5); x(6); -mu*x(1)/r^3; -mu*x(2)/r^3; -mu*x(3)/r^3];
end

opts = odeset('Reltol',1e-13,'AbsTol',1e-14,'Stats','on');
x0 = state;
[t,x] = ode113(@kepode, tspan, x0, opts);
% plot(t,x)
% legend('x','x''','y','y''','z','z''')
% title('Position and Velocity Components')

%% Cartesian components to Kepler elements
for i = 1:length(x)
    X(i,:) = x(i,1:3);  %state vector
    R(i) = norm(X(i,:));  %magnitude of the X vector

    V(i,:) = x(i,4:6); %velocity vector
    v(i) = norm(V(i,:)); %magnitude of the V vector

    H(i,:) = cross(X(i,:),V(i,:)); %Angular momentum
    h(i) = norm(H(i,:));    %magnitude of the H vector

    s = [0 0 1]';   
    N(i,:) = cross(s,H(i,:));   %Nodal Vector
    n(i) = norm (N(i,:));     %magnitude of the N vector

    ee(i,:) = ((cross(V(i,:),H(i,:))./mu) - X(i,:)./R(i));  %calculating eccentricity vector
    e(i) = norm(ee(i,:));

    N_xy = sqrt (N(1)^2 + N(2)^2);

    theta_rad(i,:) = sign (dot(cross(ee(i,:),X(i,:)),H(i,:))) * acos(dot (X(i,:),ee(i,:))/(R(i)*e(i))); %calculating theta in radians
    theta(i,:) = theta_rad(i,:) * 180/pi;

    p(i,:) = sqrt((1-e(i))/(1+e(i))) * tan(theta_rad(i,:)/2);
    E_rad(i,:) = 2 * atan(p(i,:));
    E(i,:) =  E_rad(i,:) * 180 /pi;  %calculating eccentric anomaly

    M(i,:) = (E_rad(i,:) - e(i) * sin(E_rad(i,:))) * 180 /pi; %calculating mean anomaly
end

% figure
% hold on
% plot3(x(:,1),x(:,2),x(:,3))
% plot3(0,0,0,'ro')
% axis equal
% title('Orbit of ')
end
