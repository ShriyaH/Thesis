%%Inertia change check
clc 
clear all

dim_t = [2.8/2 1.197/2];    %Fuel tank dimensions(height and radius), m
% vol_t = [1.106 1.106]; %Volume of fuel tank, m^3
m_wet = 3000;       %Total mass of SC, kg
m_dry = 1130;       %SC block mass, kg
fr = 1.65;          %Ratio of propellant to oxidiser 
v_e = 2200;         %Thruster exhaust velocity, m/s  
f_rho = [880 1440];
I_const = [15631.9133333333 0 0;0 1065.39666666667 0;0 0 15286.8166666667];
I0 = [16638.7149212849 0 0;0 2072.19825461825 0;0 0 15462.0462703546];

for i = 1:5
    a(i) = i*10;
    dv(i) = 776 + a(i);                     %DeltaV at insertion at 100km altitude, m/s
    m(i) = m_wet * 2.718^(-dv(i)/v_e);      %Mass of SC at insertion, kg
    mf(i) = m(i) - m_dry;                   %Mass of fuel at insertion, kg
    mb(i,:) = [mf(i)/(1+fr) mf(i) - mf(i)/(1+fr)];  %Mass of propellant, oxidiser at insertion, kg
    vol(i,:) = mb(i,:)./f_rho;
    ht(i,:) = vol(i,:)./(pi*dim_t(2)^2);    %reduced tank height at insertion

    Ixx_p = (1/12)*mb(i,1)*(ht(i,1)^2 + 3*dim_t(2)^2) + mb(i,1)*(ht(i,1)/2)^2;
    Iyy_p = (1/12)*mb(i,1)*(ht(i,1)^2 + 3*dim_t(2)^2) + mb(i,1)*(ht(i,1)/2)^2;
    Izz_p = (1/2)*mb(i,1)*dim_t(2)^2;

    Ixx_o = (1/12)*mb(i,2)*(ht(i,2)^2 + 3*dim_t(2)^2) + mb(i,2)*(dim_t(1)-ht(i,2)/2)^2;
    Iyy_o = (1/12)*mb(i,2)*(ht(i,2)^2 + 3*dim_t(2)^2) + mb(i,2)*(dim_t(1)-ht(i,2)/2)^2;
    Izz_o = (1/2)*mb(i,2)*dim_t(2)^2;

    I_tank(3*i-2:3*i,1:3) = [(Ixx_p + Ixx_o) 0 0
                             0 (Iyy_p + Iyy_o) 0
                             0 0 (Izz_p + Izz_o)];

    I_t(3*i-2:3*i,1:3) = I_tank(3*i-2:3*i,1:3) + I_const;

    c(3*i-2:3*i,1:3) = (I0\(I0-I_t(3*i-2:3*i,1:3)))*100 ;
    C(i,1:3) = [c(3*i-2,1) c(3*i-1,2) c(3*i,3)];
end

figure()
plot(a/1000,C(:,1),'Linewidth',1.5)
hold on
plot(a/1000,C(:,2),'Linewidth',1.5)
plot(a/1000,C(:,3),'Linewidth',1.5)
xlabel('\Delta v (km/s)')
ylabel('% Inertia Change')
legend('I_x','I_y','I_z','Location','northeastoutside')
grid on


figure()
plot(a,C(:,1),'Linewidth',1.5)
hold on
plot(a,C(:,2),'Linewidth',1.5)
plot(a,C(:,3),'Linewidth',1.5)
xlabel('\Delta v (m/s)')
ylabel('% Inertia Change')
legend('I_x','I_y','I_z','Location','northeastoutside')
grid on