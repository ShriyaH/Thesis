function [Sun] = Get_sun(Asteroid,mu)
%% Generate asteroid orbit from kepler elements and locate pericentre 
[Orbit.state,Orbit.theta,Orbit.state_p,Orbit.theta_p] = kep2cart(Asteroid.kep_orbit,mu);

%% Transform reference frame from Heliocentric frame to ACFI frame
%euler_angle = [Asteroid.axis(1),pi/2-Asteroid.axis(2),0];
euler_angle = [0,0,0];
Q_IH = Eul2Q(euler_angle,'ZXZ');
r_IH = [quat_trans(Q_IH,Orbit.state(1:4,1), 'q'); quat_trans(Q_IH,Orbit.state(5:8,1), 'q')];

Sun.rs_I = -r_IH(1:4,1); %sun position in the inertial asteroid fixed frame (sun fixed due to short propagation duration)
Sun.ast_orb = Orbit;
Sun.Q_IH = Q_IH;

end