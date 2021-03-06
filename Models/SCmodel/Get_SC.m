function [ SC ] = Get_SC( name )
% This function outputs a structure containing all the data for an SC
% Developed only for Rosetta, can be modified for Osiris
% The inputs are: name  

Rosetta.name = 'Rosetta';
Rosetta.dim.block = [2.1 2 2.8]; %SC dimensions, m
Rosetta.dim.SA = [0 14 2.28]; %Solar array dimensions, m
Rosetta.dim.SA2 = [2.28 14/5 0];
Rosetta.dim.tank = [2.8/2 1.197/2]; %Fuel tank dimensions(height and radius), m
Rosetta.volumet = [1.106 1.106]; %Volume of fuel tank, m^3
Rosetta.mass.wet = 3000; %Total mass of SC, kg
Rosetta.mass.dry = 1380; %SC total dry mass, kg
Rosetta.mass.SA = 75; %Solar array mass (one), kg
Rosetta.mass.fuel = [660 1060]; %InitialMass of fuel, kg
Rosetta.fuel_rho = [880 1440]; %Propellant and Oxidiser densities respectively, kg/m3
Rosetta.fuel_ratio = 1.65; %Ratio of propellant to oxidiser 
Rosetta.v_exh = 2200; %Thruster exhaust velocity, m/s  
Rosetta.dv_i = 826; %DeltaV at insertion at 100km altitude, m/s
Rosetta.model = {'Rosetta.stl'}; %filename to read SC 3D model

Osiris.name = 'Osiris';
Osiris.mass = 2900; %Total mass of SC, kg
Osiris.dim.SC = [2.1 2 2.8]; %SC dimensions, m
Osiris.dim.SA = [3 14]; %Solar array dimensions, m
Osiris.dim.ftank = [2.8/2 1.197/2]; %Fuel tank dimensions(height and radius), m
Osiris.fuel = 1720; %InitialMass of fuel, kg
Osiris.volumet = 1.106; %Volume of fuel tank, m^3
Osiris.fuel_rho = [880 1440]; %Propellant and Oxidiser densities respectively, kg/m3
Osiris.fuel_ratio = 1.65; %Ratio of oxidiser to propellant  
Osiris.model = {'Osiris.stl'}; %filename to read SC 3D model
 
switch name
    case 'Rosetta'
        SC=Rosetta;
        [SC.Actual.F,SC.Actual.V,SC.Actual.N] = stlread('Rosetta.stl'); %actual SC
        display(SC.name, 'Selected SC:');

    case 'Osiris'
        SC=Osiris;
        [SC.Actual.F,SC.Actual.V,SC.Actual.N] = stlread('Osiris.stl'); %actual SC
        display(SC.name, 'Selected SC:');

    otherwise
         disp('Selected non-existing SC'); 
end

%% Simplified SC Structure (only for Rosetta)
x = SC.dim.block(1)/2;
y = SC.dim.block(2)/2;
z = SC.dim.block(3)/2;

sy = SC.dim.SA(2)/5;
sz = SC.dim.SA(3)/2;
d = 0.06;
d1 = 1+y;

block = [x -y -z
      x y -z
     -x y -z
     -x -y -z
     x -y z
      x y z
      -x y z
     -x -y z];
 
SA1 = [0 d1 -sz 
      0 d1 sz 
      0 d1+sy -sz  
      0 d1+sy sz 
      0 d1+sy+d -sz  
      0 d1+sy+d sz  
      0 d1+2*sy+d -sz 
      0 d1+2*sy+d sz 
      0 d1+2*sy+2*d -sz 
      0 d1+2*sy+2*d sz 
      0 d1+3*sy+2*d -sz 
      0 d1+3*sy+2*d sz 
      0 d1+3*sy+3*d -sz 
      0 d1+3*sy+3*d sz 
      0 d1+4*sy+3*d -sz 
      0 d1+4*sy+3*d sz 
      0 d1+4*sy+4*d -sz 
      0 d1+4*sy+4*d sz 
      0 d1+5*sy+4*d -sz 
      0 d1+5*sy+4*d sz]; 
  
SA2 = -SA1;

[X,Y,Z] = cylinder(0.025);
X = [X(1,1:length(X)),X(2,1:length(X))];
Y = [Y(1,1:length(Y)),Y(2,1:length(Y))];
Z = -Z-z;
Z = [Z(1,1:length(Z)),Z(2,1:length(Z))-2];
TAG = [X'+x,Y',Z'];

Polyhedron.Vertices = [block;SA1;SA2;TAG];
Polyhedron.Facets = [1 2 6
    1 6 5
    2 3 7
    2 7 6
    3 4 8
    3 8 7
    4 1 5
    4 5 8
    4 3 2
    4 2 1
    5 6 7
    5 7 8       % block
    9 11 12
    9 12 10
    13 15 16
    13 16 14
    17 19 20
    17 20 18
    21 23 24
    21 24 22
    25 27 28
    25 28 26
    29 32 30
    29 31 32
    33 36 34
    33 35 36
    37 40 38
    37 39 40
    41 44 42
    41 43 44
    45 48 46
    45 47 48
    49 70 71
    49 50 71
    50 71 72
    50 51 72
    51 72 73
    51 52 73
    52 73 74
    52 53 74
    53 74 75
    53 54 75
    54 75 76
    54 55 76
    55 76 77
    55 56 77
    56 77 78
    56 57 78
    57 78 79
    57 58 79
    58 79 80
    58 59 80
    59 80 81
    59 60 81
    60 81 82
    60 61 82
    61 82 83
    61 62 83
    62 83 84
    62 63 84
    63 84 85
    63 64 85
    64 85 86
    64 65 86
    65 86 87
    65 66 87
    66 87 88 
    66 67 88
    67 88 89
    67 68 89
    68 89 90
    68 69 90
    69 90 70
    69 49 90];

SA11 = [-sz d1 0 
      sz d1 0 
      -sz d1+sy 0  
      sz d1+sy 0]; 
  
SA22 = -SA11;
Polyhedron.Vertices2 = [block;SA11;SA22;TAG];
       
Polyhedron.Facets2 = [1 2 6
    1 6 5
    2 3 7
    2 7 6
    3 4 8
    3 8 7
    4 1 5
    4 5 8
    4 3 2
    4 2 1
    5 6 7
    5 7 8
    9 11 12
    9 12 10
    13 15 16
    13 16 14
    17 19 20
    17 20 18
    21 23 24
    21 24 22
    25 27 28
    25 28 26
    29 32 30
    29 31 32
    33 36 34
    33 35 36
    37 40 38
    37 39 40];

% [Polyhedron.Facets, Polyhedron.Vertices] = stlread('Rosetta.stl');
% [nf,nv] = reducepatch(SC.Polyhedron.Facets,SC.Polyhedron.Vertices,0.02);
[Polyhedron.normalsf,Polyhedron.R_1,Polyhedron.R_2,Polyhedron.R_3,Polyhedron.C,Polyhedron.A_facet] = SC_prop(Polyhedron);

SC.Polyhedron = Polyhedron;

%% Initial Inertia
m_b = SC.mass.dry - 2*SC.mass.SA; %dry mass of SC without solar arrays

Ixx_b = (1/12)*m_b*(SC.dim.block(2)^2 + SC.dim.block(3)^2); %dry SC box
Iyy_b = (1/12)*m_b*(SC.dim.block(1)^2 + SC.dim.block(3)^2);
Izz_b = (1/12)*m_b*(SC.dim.block(1)^2 + SC.dim.block(2)^2);

SC.I.I_block = [Ixx_b 0 0
                0 Iyy_b 0
                0 0 Izz_b];


Ixx_s = (1/12)*SC.mass.SA*(SC.dim.SA(2)^2 + SC.dim.SA(3)^2) + SC.mass.SA*(d1+7)^2;
Iyy_s = (1/12)*SC.mass.SA*(SC.dim.SA(1)^2 + SC.dim.SA(3)^2); 
Izz_s = (1/12)*SC.mass.SA*(SC.dim.SA(1)^2 + SC.dim.SA(2)^2) + SC.mass.SA*(d1+7)^2;
          
SC.I.I_solar = [Ixx_s 0 0
                0 Iyy_s 0
                0 0 Izz_s];
 
Ixx_s2 = (1/12)*SC.mass.SA*(SC.dim.SA2(2)^2 + SC.dim.SA2(3)^2) + SC.mass.SA*(d1+1.4)^2;
Iyy_s2 = (1/12)*SC.mass.SA*(SC.dim.SA2(1)^2 + SC.dim.SA2(3)^2); 
Izz_s2 = (1/12)*SC.mass.SA*(SC.dim.SA2(1)^2 + SC.dim.SA2(2)^2) + SC.mass.SA*(d1+1.4)^2;
          
SC.I.I_solar2 = [Ixx_s2 0 0
                0 Iyy_s2 0
                0 0 Izz_s2];            

SC.I.I_constant = SC.I.I_block + 2*SC.I.I_solar;  %SA inertia constant for now
SC.I.I_constant2 = SC.I.I_block + 2*SC.I.I_solar2;

SC.mass.m_i = SC.mass.wet * 2.718^(-SC.dv_i/SC.v_exh); %Mass of SC at insertion, kg

SC.mass.mf_i = SC.mass.m_i - SC.mass.dry;                  %Mass of fuel at insertion, kg
SC.mass.mb_i = [SC.mass.mf_i/(1+SC.fuel_ratio) SC.mass.mf_i - SC.mass.mf_i/(1+SC.fuel_ratio)];  %Mass of propellant, oxidiser at insertion, kg
SC.mass.vol_i = SC.mass.mb_i./SC.fuel_rho;
SC.mass.ht_i = SC.mass.vol_i./(pi*SC.dim.tank(2)^2); %reduced tank height at insertion

Ixx_p = (1/12)*SC.mass.mb_i(1)*(SC.mass.ht_i(1)^2 + 3*SC.dim.tank(2)^2) + SC.mass.mb_i(1)*(SC.mass.ht_i(1)/2)^2;
Iyy_p = (1/12)*SC.mass.mb_i(1)*(SC.mass.ht_i(1)^2 + 3*SC.dim.tank(2)^2) + SC.mass.mb_i(1)*(SC.mass.ht_i(1)/2)^2;
Izz_p = (1/2)*SC.mass.mb_i(1)*SC.dim.tank(2)^2;

Ixx_o = (1/12)*SC.mass.mb_i(2)*(SC.mass.ht_i(2)^2 + 3*SC.dim.tank(2)^2) + SC.mass.mb_i(2)*(SC.dim.tank(1)-SC.mass.ht_i(2)/2)^2;
Iyy_o = (1/12)*SC.mass.mb_i(2)*(SC.mass.ht_i(2)^2 + 3*SC.dim.tank(2)^2) + SC.mass.mb_i(2)*(SC.dim.tank(1)-SC.mass.ht_i(2)/2)^2;
Izz_o = (1/2)*SC.mass.mb_i(2)*SC.dim.tank(2)^2;

SC.I.I_tank = [(Ixx_p + Ixx_o) 0 0
             0 (Iyy_p + Iyy_o) 0
             0 0 (Izz_p + Izz_o)];          
 

SC.I.I_total = SC.I.I_constant + SC.I.I_tank;
SC.I.I_total2 = SC.I.I_constant2 + SC.I.I_tank;
SC.mass.m_facet = [((SC.mass.m_i-2*SC.mass.SA)/12).*ones(12,1);(2*SC.mass.SA/20).*ones(20,1)];

%% Reflectivity
SC.a_r = [ones(12,1)*0.5; ones(20,1)*0.21];
SC.a_d = [ones(12,1)*0.2; ones(20,1)*0.1];

%% Lidar reference frame
th = deg2rad(30); %axis of LIDAR wrt z axis
SC.q_LB = [0; sin(th/2); 0; cos(th/2)];

%% Solar panel forbidden region
SC.th_sa_forb = atan(sz/(d1+5*sy+4*d));
 
%% Ratio for point mass structure
A_block = sum(Polyhedron.A_facet(1:12,1));
A_SA = sum(Polyhedron.A_facet(13:32,1));
SC.pm_ratio = (m_b*A_block)/(SC.mass.SA*A_SA);
SC.pm_ratio = round(SC.pm_ratio);

%% Thrusters
SC.phi = deg2rad(27);    %cant angle
SC.I_sp = 225;           %sp. impulse
SC.n_e = 6;              %no. of thrusters

%% TAG sampler
SC.e_TAG  = [0,0,-1];
SC.l_TAG = 3;
% p = atan2(SC.e_TAG (2),SC.e_TAG (1));
% t = atan2(SC.e_TAG (3),norm(SC.e_TAG ));
% SC.q_TAGB = Eul2Q([-p,t+pi/2-0.09,-p],'ZYZ');
SC.q_TAGB = [0;0;0;-1];
end