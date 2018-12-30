%% Find perturbing accelerations

%SRP
A = 2*32 + 2.8*2.1;
M = 2.108383601375709e+03;
c = 2.999*10^8;
W = 1361;
r = 0.5;
r1 = 0.21;
d = 2.7941;
r0 = 100e3;
G = 6.67e-11;
m = 2.618e18;
mu = G*m;
% mu = 309677312;
    
f = (r*W*A)/(c*M*d^2);
f = 1.516883308382794e-08;
n = sqrt(mu/r0^3);
alpha = 90;
t = 2*pi/n;
a_max = 4*(f/n^2)*cosd(alpha);
i_max = (180*(f/(n^2*r0))*sind(alpha))/pi;
o_max = (180*2*(f/(n^2*r0))*sind(alpha))/pi;

%  4.578657295439825e-054.703464296085258e-05