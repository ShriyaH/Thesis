function [SC] = Int_mass_update(SC,DV) 
% Rosetta : 12 bi-propellant reaction thrusters, 10N nominal force (Attitude control and velocity increments)
 m_t = [];
 m_t(1,1) = SC.mass_prop.m_i;
 
    for i=1:100       
        
        % Tsiolkovsky's equation
        m_rem(i) = m_t(i) * 2.718^(-DV/SC.v_exh); %Mass of SC at insertion, kg

        % Mass properties
        mf_new(i) = m_rem(i) - SC.mass_prop.m_d;  %Mass of fuel after Delta V, kg
        mbi_new(i,:) = [mf_new(i)/(1+SC.fuel_ratio) mf_new(i) - mf_new(i)/(1+SC.fuel_ratio)];  %Mass of propellant, oxidiser at insertion, kg
        vol_new(i,:) = mbi_new(i,:)./SC.fuel_rho;
        tankh_new(i,:) = vol_new(i,:)./(pi*SC.dim.ftank(2)^2);
        
        % Inertia Change
        Ixx_p(i) = (1/12)*mbi_new(i,1)*(tankh_new(i,1)^2 + 3*SC.dim.ftank(2)^2) + mbi_new(i,1)*(tankh_new(i,1))/2)^2;
        Iyy_p(i) = (1/12)*mbi_new(i,1)*(tankh_new(i,1)^2 + 3*SC.dim.ftank(2)^2) + mbi_new(i,1)*(tankh_new(i,1)/2)^2;
        Izz_p(i) = (1/2)*mbi_new(i,1)*SC.dim.ftank(2)^2;

        Ixx_o(i) = (1/12)*mbi_new(i,2)*(tankh_new(i,2)^2 + 3*SC.dim.ftank(2)^2) + mbi_new(i,2)*(SC.dim.SC(3)/4 - tankh_new(i,1)/2)^2;
        Iyy_o(i) = (1/12)*mbi_new(i,2)*(tankh_new(i,2)^2 + 3*SC.dim.ftank(2)^2) + mbi_new(i,2)*(SC.dim.SC(3)/4 - tankh_new(i,1)/2)^2;
        Izz_o(i) = (1/2)*mbi_new(i,2)*SC.dim.ftank(2)^2;

        SC.I.I_prop_new(i) = [(Ixx_p(i) + Ixx_o(i)) 0 0
                            0 (Iyy_p(i) + Iyy_o(i)) 0
                            0 0 (Izz_p(i) + Izz_o(i))];          
 
        SC.I.I_new(i) = SC.I.I_i_notank + SC.I.I_prop_new(i);
    end
    SC.Mass_prop.updates = m_rem;
 end

