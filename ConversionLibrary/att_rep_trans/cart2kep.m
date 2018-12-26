function [Kepler_rad,Kepler_deg]=cart2kep(x,v,mu)
% Cartesian components to Kepler elements
% x = 8751268.4691; 
% y = -7041314.6869;
% z = 4846546.9938; 
% xdot = 332.2601039; 
% ydot = -2977.0815768; 
% zdot = -4869.8462227;
% x = x(1);
% y = x(2);
% z = x(3);
% xdot = v(1);
% ydot = v(2);
% zdot = v(3);

%NASA,2014 data
% x = -2700816.14; 
% y = -3314092.8;
% z = 5266346.42; 
% xdot = 5168.606550; 
% ydot = -5597.546618; 
% zdot = -868.878445;

%mu = 398600.441e9; %Gravitational parameter

X = x;  %state vector
r = norm(X);  %magnitude of the X vector

V = v; %velocity vector
v = norm(V); %magnitude of the V vector

H = cross(X,V); %Angular momentum
h = norm(H);    %magnitude of the H vector

s = [0 0 1]';   
N = cross(s,H);   %Nodal Vector
n = norm (N);     %magnitude of the N vecto

a = 1/((2/r) - (v^2/mu));   %calculating semi-major axis

ee = ((cross(V,H)/mu) - X/r);  %calculating eccentricity vector
e = norm(ee);

i = acos(H(3)/h) * 180/pi;  %calculating angle of inclination

N_xy = sqrt (N(1)^2 + N(2)^2);
RAAN = atan2 ((N(2)/N_xy),(N(1)/N_xy)) * 180/pi;  %calculating right ascension of ascending node
if isnan(RAAN)
    RAAN = 0;
end

omega = sign (dot(cross(N,ee),H)) * acos(dot (ee,N)/(n*e)) * 180/pi; %calculating argument of perigee
if isnan(omega)
    omega = 0;
end

theta_rad = sign (dot(cross(ee,X),H)) * acos(dot (X,ee)/(r*e)); %calculating theta in radians
theta = theta_rad * 180/pi;

p = sqrt((1-e)/(1+e)) * tan(theta_rad/2);
E_rad = 2 * atan(p);
E =  E_rad * 180 /pi;  %calculating eccentric anomaly

M = (E_rad - e * sin(E_rad)) * 180 /pi; %calculating mean anomaly

Kepler_rad = [a e deg2rad(i) deg2rad(RAAN) deg2rad(omega) deg2rad(theta) E M];
Kepler_deg = [a e i RAAN omega theta E M];
end


