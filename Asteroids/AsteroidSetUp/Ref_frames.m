function [Frames] = Ref_frames(Asteroid)
%% Asteroid Illumination Model

mu_s = 1.32712440018e20; %standard gravitational parameter of the sun
T = 2*pi/Asteroid.kep_orbit(1,7);
tspan = [0 T+7*24*3600];
% Generate asteroid orbit from kepler elements and locate pericentre 
[Orbit.state,Orbit.theta,Orbit.state_p,Orbit.theta_p] = Get_Orbit(Asteroid.kep_orbit,mu_s);
[Orbit.x,Orbit.true_anomalies,Orbit.t_pos] = Propagate_Orbit(Asteroid.kep_orbit, Orbit.state, mu_s,tspan);

% Transform reference frame from J2000 ecliptic to asteroid orbital frame
eul_angl = [Asteroid.kep_orbit(4);Asteroid.kep_orbit(3);Asteroid.kep_orbit(5)+ pi/2];
DCM.DCM_int_S = Eul2DCM(eul_angl,'ZXZ');
Q.Q_int_S = DCM2Q(DCM.DCM_int_S); %quaternion at perigee
x_p_new = [(DCM.DCM_int_S*Orbit.state_p(1:3))' (DCM.DCM_int_S*Orbit.state_p(4:6))'];

for i = 1:length(Orbit.true_anomalies)
    Q.Q_O_int(:,i) = [0; 0; sin(Orbit.true_anomalies(i)/2); cos(Orbit.true_anomalies(i)/2)];
    Q.Q_OS(:,i) = cross_quat(Q.Q_O_int(:,i),Q.Q_int_S);
    DCM.DCM_OS(3*i-2:3*i,1:3) = Q2DCM(Q.Q_OS(:,i));
    x_new(i,:) = (DCM.DCM_OS(3*i-2:3*i,1:3)*Orbit.x(i,:)')';   
end

% Transform reference frame from asteroid orbital to asteroid inertial fixed
Q.Q_I_int1 = [sin((pi/2-Asteroid.axis(2))/2); 0; 0; cos((pi/2-Asteroid.axis(2))/2)];

Q.Q_I_int2 = [0; 0; sin(Asteroid.axis(1)/2); cos(Asteroid.axis(1)/2)];
Q.Q_IS = cross_quat(Q.Q_I_int2,cross_quat(Q.Q_I_int1,Q.Q_int_S));
DCM.DCM_IS = Q2DCM(Q.Q_IS);

% Transform reference frame from asteroid inertial fixed to asteroid rotational
dt = Asteroid.T/3600;
count = 0;
for t = 0:dt:Asteroid.T 
    count = count+1;
    Q.Q_RI(:,count) = [0; 0; sin(Asteroid.w_AI(3)*t/2); cos(Asteroid.w_AI(3)*t/2)];
    Q.Q_RS(:,count) = cross_quat(Q.Q_RI(:,count),Q.Q_IS);
    DCM.DCM_RS(3*count-2:3*count,1:3) = Q2DCM(Q.Q_RS(:,count));
end

Frames.Orbit=Orbit;
Frames.DCM=DCM;
Frames.quat = Q;

end