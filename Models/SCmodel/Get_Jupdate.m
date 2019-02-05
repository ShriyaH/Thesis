function [J_up] = Get_Jupdate(m)
global SC
    % Mass properties
    mf_new = m - SC.mass.dry;  
    mbi_new = [mf_new/(1+SC.fuel_ratio) mf_new - mf_new/(1+SC.fuel_ratio)];  %Mass of propellant, oxidiser at insertion, kg
    vol_new = mbi_new./SC.fuel_rho;
    tankh_new = vol_new./(pi*SC.dim.tank(2)^2);

    % Inertia Change
    Ixx_p = (1/12)*mbi_new(1)*(tankh_new(1)^2 + 3*SC.dim.tank(2)^2) + mbi_new(1)*(tankh_new(1)/2)^2;
    Iyy_p = (1/12)*mbi_new(1)*(tankh_new(1)^2 + 3*SC.dim.tank(2)^2) + mbi_new(1)*(tankh_new(1)/2)^2;
    Izz_p = (1/2)*mbi_new(1)*SC.dim.tank(2)^2;

    Ixx_o = (1/12)*mbi_new(2)*(tankh_new(2)^2 + 3*SC.dim.tank(2)^2) + mbi_new(2)*(SC.dim.block(3)/4 - tankh_new(1)/2)^2;
    Iyy_o = (1/12)*mbi_new(2)*(tankh_new(2)^2 + 3*SC.dim.tank(2)^2) + mbi_new(2)*(SC.dim.block(3)/4 - tankh_new(1)/2)^2;
    Izz_o = (1/2)*mbi_new(2)*SC.dim.tank(2)^2;

    J_up = [(Ixx_p + Ixx_o) 0 0
             0 (Iyy_p + Iyy_o) 0
             0 0 (Izz_p + Izz_o)];          

    J_up = SC.I.I_constant + J_up;

end