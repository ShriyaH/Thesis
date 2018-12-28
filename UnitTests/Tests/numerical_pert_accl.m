%% Find perturbing accelerations

%SRP
A = 2*(2.1*2+2.8*2+2.1*2.8);
A1 = 2*32;
M = 1950;
M1 = 150;
c = 2.999*10^8;
W = 1361;
r = 0.5;
r1 = 0.21;
d = 2.7941;
a = 60e3;
G = 6.67e-11;
m = 2.618e18;
mu = G*m;
    
f1 = (r*W*A)/(c*M*d^2);
f2 = (r1*W*A1)/(c*M1*d^2);
f = f1+f2;

n = sqrt(mu/a^3);
alpha = 90;
r0 = 60e3;
a_max = 4*(f/n^2)*cosd(alpha);
i_max = (180*(f/(n^2*r0))*sind(alpha))/pi;
o_max = (180*2*(f/(n^2*r0))*sind(alpha))/pi;