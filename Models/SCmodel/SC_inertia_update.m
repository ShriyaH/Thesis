function [I_new,m_n,percent_change] = SC_inertia_update(mass_new,mag_v,I_old)
%for mapping: input mag_v (magnitude of deltav) and mass_new = 0
%for descent: input mass_new (new mass obtained from optimiser) and mag_v = 0
%I_old is the previous time step inertia

global SC

I_c = SC.I.I_constant;
m_d = SC.mass.dry;
m_i = SC.mass.m_i;
t = SC.dim.tank; 
fr = SC.fuel_ratio;
rho = SC.fuel_rho;
v_e = SC.v_exh;

if mag_v ~=0
    %Tsiolkovsky's equation
    dv = mag_v; 
    m_n = m_i .* 2.718.^(-dv/v_e);
else
    m_n = mass_new;
end

%% Mass properties
mf = m_n - m_d;                   %Mass of fuel at insertion, kg
mb = [mf./(1+fr) mf - mf./(1+fr)];  %Mass of propellant, oxidiser at insertion, kg
vol = mb./rho;
ht = vol./(pi*t(2)^2);

%% SC propellant tank inertia 
Ixx_p = (1/12)*mb(1)*(ht(1)^2 + 3*t(2)^2) + mb(1)*(ht(1)/2)^2;
Iyy_p = (1/12)*mb(1)*(ht(1)^2 + 3*t(2)^2) + mb(1)*(ht(1)/2)^2;
Izz_p = (1/2)*mb(1)*t(2)^2;

Ixx_o = (1/12)*mb(2)*(ht(2)^2 + 3*t(2)^2) + mb(2)*(t(1)-ht(2)/2)^2;
Iyy_o = (1/12)*mb(2)*(ht(2)^2 + 3*t(2)^2) + mb(2)*(t(1)-ht(2)/2)^2;
Izz_o = (1/2)*mb(2)*t(2)^2;

I_tank = [(Ixx_p + Ixx_o) 0 0
          0 (Iyy_p + Iyy_o) 0
          0 0 (Izz_p + Izz_o)];          

I_new = I_c + I_tank;

%% Percentage change in Inertia
I_change = I_old-I_new;
percent_change = 100.*(I_change/I_old);

end