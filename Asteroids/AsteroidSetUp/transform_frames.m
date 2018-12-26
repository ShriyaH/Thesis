unction [] = Transform_frames()
%J2000 ecliptic to Asteroid orbital
[DCM_OS] = Create_DCM([Asteroid.kep_orbit(4);Asteroid.kep_orbit(3);Asteroid.kep_orbit(5)],'ZXZ');
% include propagation of the ref frame from perigee

%Asteroid orbital to Asetroid inertial
[DCM_IO] = Create_DCM([Asteroid.kep_orbit(4);Asteroid.kep_orbit(3);Asteroid.kep_orbit(5)],'ZXZ');

%Asteroid inertial to Asetroid rotational
[DCM_IO] = Create_DCM([Asteroid.kep_orbit(4);Asteroid.kep_orbit(3);Asteroid.kep_orbit(5)],'ZXZ');

%SC body frame to SC LVLH
[DCM_IO] = Create_DCM([Asteroid.kep_orbit(4);Asteroid.kep_orbit(3);Asteroid.kep_orbit(5)],'ZXZ');

%Asteroid inertial to SC LVLH
[DCM_IO] = Create_DCM([Asteroid.kep_orbit(4);Asteroid.kep_orbit(3);Asteroid.kep_orbit(5)],'ZXZ');
end