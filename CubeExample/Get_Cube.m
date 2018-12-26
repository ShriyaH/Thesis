function [Cube] = Get_Cube(x,y,z,G)
% This function outputs the cube with all its properties
% The inputs are:
% xyz - dimesions wantedfor the cube
% n - number of facets

%% Polyhedron Cube Physical Properties

Cube.w_AI = [0 0 3e-04]; % rotation rate of cube relaitve to inertial, rad/s
Cube.mass = 4e18; % Mass of cube, kg
Cube.dim = [x y z]; %Dimensions of cube, m
Cube.rho = 2670; %Density of cubes, kgm^3
Cube.T = 5*3600; %Rotation period, sec
Cube.axis = deg2rad([170;30]); %Rotation axis angle wrt J2000 ecliptic,rad (longitude, latitude)
Cube.kep_orbit = [4.1793e+11 0.25079 0.2289 3.7587 3.1440 5.3059 4.2824e-08];
%[semi-major axis(3 AU); m, eccentricity, inclination(15deg); rad, RAAN(200deg); rad, argument of perigee(180deg); rad, mean anomaly(250deg); rad, mean motion(2.5e-06deg/s); rad/s]

%% Build Polyhedron Cube 

Polyhedron.Vertices = [-x/2 -y/2 -z/2;x/2 -y/2 -z/2;x/2 y/2 -z/2;-x/2 y/2 -z/2;-x/2 -y/2 z/2;x/2 -y/2 z/2;x/2 y/2 z/2;-x/2 y/2 z/2];
Polyhedron.Facets = [1 2 6;1 6 5;2 3 7;2 7 6;3 4 8;3 8 7;4 1 5;4 5 8;1 3 2;1 4 3;5 6 7; 5 7 8];

[Polyhedron.Edges, Polyhedron.E_tilde, Polyhedron.normalsa, Polyhedron.normalsb, Polyhedron.E.R1, Polyhedron.E.R2] = Find_edges(Polyhedron); % Find the edges, and calculate the diadic matrix E_tilde
[Polyhedron.F_tilde, Polyhedron.normalsf,  Polyhedron.x,  Polyhedron.y,  Polyhedron.z, Polyhedron.A_facet] = F_tild(Polyhedron); % Calculating diadic matrix for the faces

% Calculate volume of the Cube, often useful for checking the mean
% density/mass of the Cube
[Cube.Volume, Cube.Area]=stlVolume(Polyhedron.Vertices',Polyhedron.Facets'); 

% Cube.mass=Cube.rho*Cube.Volume; %Uncomment this if you want
% calculated mass
Cube.mu = G*Cube.mass;
Cube.Polyhedron = Polyhedron;
end






