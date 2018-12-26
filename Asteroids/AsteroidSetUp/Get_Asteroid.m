function [ Asteroid ] = Get_Asteroid( name, n )
% This function outputs a structure containing all the data for an asteroid
% Author: Razgus B.
% The inputs are:
%
% name - for now it is only Kleopatra, Itokawa and Churyomov Gerasimenko
%
% n - number of file to read (set it to 1, to have the lowest resolution,
% see the code below. Not all the asteroids have higher resolution, e.g.
% Kleopatra)
% 
% Edited by Hazra S. 19/01/18

G=6.67408e-11;

Kleopatra.name = 'Kleopatra';
Kleopatra.w_AI = [0 0 3.2411e-04]; % rotation rate of asteroid relaitve to inertial, rad/s
Kleopatra.mass = 4.64e18; % Mass of asteroid, kg
Kleopatra.rho = 6545; %Density of asteroids, kgm^3
Kleopatra.T = 5.385*3600; %Rotation period, sec
Kleopatra.axis = deg2rad([76;16]); %Rotation axis angle wrt J2000 ecliptic,rad (longitude, latitude)
Kleopatra.kep_orbit = [4.1793e+11 0.25079 0.2289 3.7587 3.1440 5.3059 4.2824e-08]; 
%[semi-major axis; m, eccentricity, inclination; rad, RAAN; rad, argument of perigee; rad, mean anomaly; rad, mean motion; rad/s]
Kleopatra.file_arr = {'kleopatra.txt'}; %filename to read polyhedron data from



CG.name = 'CG';
CG.w_AI = [0 0 1.3719e-04];
CG.mass = 1e13;
CG.rho = 532.275;
CG.T = 12.76137*3600; 
CG.axis = deg2rad([65; 59]);
CG.kep_orbit = [5.1832e+11 .64058 0.1229 0.8758 0.2216 1.6017 3.0872e-08];
CG.file_arr = {'CG1.txt','CG2.txt'} ;


Itokawa.name = 'Itokawa';
Itokawa.w_AI = [0 0 1.4386e-04];
Itokawa.mass = 3.51e10;
Itokawa.rho =  1959; 
Itokawa.T = 12.132*3600; 
Itokawa.axis = deg2rad([355; -84]);
Itokawa.kep_orbit = [1.9809e+11 .28016 0.0283 1.2057 2.8416 4.5517 1.3067e-07];
Itokawa.file_arr = {'Itokawa.txt','Itokawa_2.txt','Itokawa_3.txt'};




switch name
    case 'Kleopatra'
        Asteroid=Kleopatra;
        Asteroid.file=Asteroid.file_arr{n};
        display(Asteroid.name, 'Selected asteroid:');

    case 'Itokawa'
        Asteroid=Itokawa;
        Asteroid.file=Asteroid.file_arr{n};
        display(Asteroid.name, 'Selected asteroid:');

    case 'CG'
        Asteroid=CG;
        Asteroid.file=Asteroid.file_arr{n};
        display(Asteroid.name, 'Selected asteroid:');

    otherwise
         disp('Selected non-existing asteroid');
end

[Asteroid.v,Asteroid.f] = Find_index(Asteroid.file); % This function finds the indices of vertices and faces, usually in the files they are specified in one column, well actually three :)

Data=importdata(Asteroid.file);

Polyhedron.Vertices=1000*Data.data(1:Asteroid.v,:); % coversion from km to m
Polyhedron.Facets=Data.data(Asteroid.v+1:Asteroid.f,:);

[Polyhedron.Edges, Polyhedron.E_tilde, Polyhedron.normalsa, Polyhedron.normalsb,Polyhedron.E.R1, Polyhedron.E.R2] = Find_edges(Polyhedron); % Yes, you need to find the edges, and calculate the diadic matrix E_tilde

[Polyhedron.F_tilde, Polyhedron.normalsf,  Polyhedron.x,  Polyhedron.y,  Polyhedron.z, Polyhedron.A_facet, Polyhedron.Centres] = F_tild(Polyhedron); % Calculating diadic matrix for the faces

clear Data

% Calculate volume of the polyhedron, often useful for checking the mean
% density/mass of the asteroid

[Asteroid.Volume, Asteroid.Area]=stlVolume(Polyhedron.Vertices',Polyhedron.Facets'); 

% Asteroid.mass=Asteroid.rho*Asteroid.Volume; %Uncomment this if you want
% calculated mass

Asteroid.mu=G*Asteroid.mass;
Asteroid.Polyhedron=Polyhedron;

end

