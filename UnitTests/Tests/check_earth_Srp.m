A = 0.2;
c = 2.999*10^8;
W = 1360;
r = 1.8;
a = 500e3+6378e3;
mu = 3.986004418e14;
    
f = (r*W*A)/c;
T = 2*pi*sqrt(a^3/mu);
n = 2*pi/T;
alpha = 70;

a_max = 4*(f/n^2)*cosd(alpha);
i_max = (3600*180*(f/(n^2*a))*sind(alpha))/pi;
o_max = (2*3600*180*(f/(n^2*a))*sind(alpha))/pi;