%%Find perturbing accelerations and variations in Kepler elements
%% Initial values
c = 2.999*10^8;             %vel of light
W = 1361;                   %solar flux constant
G = 6.67e-11;               %grav. constant
AU = 1.497e11;
mu_s = 1.32712440018e20;
mu_l = 4.9048695e12 ;
mu_e = 3.986004418e14;
mu_k = 309677312;           %Kleopatra stand. grav. para.
m_t = 2.618e18;             %mass of target body
mu_t = G*m_t;               %standard grav parameter
d_se = AU;
d_me = 384399e3; 

%% SRP
alpha = 90;                 %angle between solar direction and orbital plane
d = 2.7941;                 %distance of SC from the sun in AU
r0 = 100e3;                 %semi-major axis of SC orbit
W_t = W/(c*d^2);
A1 = 69.44;                 %area of SC under SRP
M = 2.1084e3;               %mass of SC
r = 1.4;                    %reflectivity of SC body
                 
n = sqrt(mu_t/r0^3);                           %mean motion
t = 2*pi/n;                                  %period of orbit

f = r*W_t*A1/M;                                %numerical SRP acceleration 1.516883308382794e-08
da_max = 4*(f/n^2)*cosd(alpha);               %max variation in semi-major axis
di_max = (180*(f/(n^2*r0))*sind(alpha))/pi;   %max variation in inclination
do_max = (180*2*(f/(n^2*r0))*sind(alpha))/pi; %max variation in  RAAN 4.578657295439825e-05 4.703464296085258e-05

kep_orbit(1,1) = r0; %a
kep_orbit(1,2) = 0; %e
kep_orbit(1,3) = pi/2; %i
kep_orbit(1,4) = pi/2; %RAAN
kep_orbit(1,5) = pi/2; %omega
kep_orbit(1,6) = 0; %M
[x_srp,theta_srp] = kep2cart(kep_orbit,mu_t); %get orbit to check

%% 3BP 

i0 = pi/18; %orbit inclination
d = d*AU;
d =3.811366027806348e+11;
%a = -(3/2)*pi*(mu_l/mu_e)*((r0+400e3+6371e3)/d_me)^3; %constant product (earth moon check)
a = -(3/2)*pi*(mu_s/mu_t)*(r0/d)^3; %constant product for test case

count = 0;
for alpha = 0:1:2*pi
    count = count +1 ;
    di(count) = a*sin(i0)*sin(2*alpha);
    do(count) = 2*a*cos(i0)*(sin(alpha))^2;
end

di_max2 = max(abs(di))*180/pi;
do_max2 = max(abs(do))*180/pi;

kep_orbit(1,1) = r0; %a
kep_orbit(1,2) = 0; %e
kep_orbit(1,3) = i0; %i
kep_orbit(1,4) = pi/2; %RAAN
kep_orbit(1,5) = 0; %omega
kep_orbit(1,6) = 0; %M
[x_3bp,theta_3bp] = kep2cart(kep_orbit,mu_t); 

%% GG