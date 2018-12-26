%%---Unit_Test: Third body perturbation force and torques---%%
G = 6.67408e-11; 
q_BI =[0; 0; 0; 0];
mu_s = 1.326807104e20;
mu_m = 4.9048695e12;
mu_e = 3.986e14;

Switch = 1;
kep_orbit(1,1) = 3844e3; %a
kep_orbit(1,2) = 0.01; %e
kep_orbit(1,4) = 0; %RAAN
kep_orbit(1,5) = 0; %omega
kep_orbit(1,6) = 0; %M
count = 1;
for i = [55 60 70 80]
    
    kep_orbit(1,3) = i; %i
    [state(:,count),theta(count),state_p(:,count),theta_p(count)] = kep2cart(kep_orbit,mu_m);
    count = count +1;
end
% [x y z] = sph2cart(31.2958,12.69,1.0056074*1.496e11);
% r_d = [x y z];
% 
% r_i = state(1:3,1)';
% 
% % r_d = [1.496e11 0 0];
% [a_3BP_I]   = F_3BP(r_i,r_d,mu_s,Switch);
% a = norm(a_3BP_I);
% 
% aa = 2*(mu_s/mu)*(norm(r_i)/norm(r_d))^3;


