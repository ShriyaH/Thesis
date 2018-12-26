function [Frames_SC] = Ref_frames2(Frames,Object,mu,T,nsteps)
%% SC Reference frames

% T = 2*pi/Object.kep_orbit(1,7);
% tspan = [0 T+7*24*3600];

%% Generate asteroid orbit from kepler elements and locate pericentre 
[Orbit.state,Orbit.theta,Orbit.state_p,Orbit.theta_p] = Get_Orbit(Object.kep_orbit,mu);
[Orbit.x,Orbit.true_anomalies] = Propagate_Orbit(Orbit.state, mu,0,T,nsteps);

%% Transform reference frame from heliocentric to asteroid orbital frame
eul_angl = [Object.kep_orbit(4);Object.kep_orbit(3);Object.kep_orbit(5)];

DCM.DCM_PH = Eul2DCM(eul_angl,'ZXZ'); % DCM at perigee
Q.Q_PH = DCM2Q(DCM.DCM_PH); % quaternion at perigee
% Frames.x_p_new_d = [(DCM.DCM_PH*Orbit.state_p(1:3))' (DCM.DCM_PH*Orbit.state_p(5:7))'];
Frames.x_PH = [cross_quat(Q.Q_PH,cross_quat(Orbit.state_p(1:4,1),conj_quat(Q.Q_PH)))' cross_quat(Q.Q_PH,cross_quat(Orbit.state_p(5:8,1),conj_quat(Q.Q_PH)))'];

for i = 1:length(Orbit.true_anomalies)
    Q.Q_LP(:,i) = [0; 0; sin(Orbit.true_anomalies(i)/2); cos(Orbit.true_anomalies(i)/2)];
    Q.Q_LH(:,i) = cross_quat(Q.Q_LP(:,i),Q.Q_PH);
%     DCM.DCM_LH(3*i-2:3*i,1:3) = Q2DCM(Q.Q_LH(:,i));
%     Frames.x_new_d(i,:) = [(DCM.DCM_LH(3*i-2:3*i,1:3)*Orbit.x(i,1:3)')' (DCM.DCM_LH(3*i-2:3*i,1:3)*Orbit.x(i,5:7)')']; 
    Frames.x_LH(i,:) = [cross_quat(Q.Q_LH(:,i),cross_quat(Orbit.x(i,1:4)',conj_quat(Q.Q_LH(:,i))))' cross_quat(Q.Q_LH(:,i),cross_quat(Orbit.x(i,5:8)',conj_quat(Q.Q_LH(:,i))))'];
end

%% Transform reference frame from heliocentric to asteroid inertial fixed
Q.Q_Ilat = [sin((pi/2-Object.axis(2))/2); 0; 0; cos((pi/2-Object.axis(2))/2)];
Q.Q_Ilon = [0; 0; sin(Object.axis(1)/2); cos(Object.axis(1)/2)];
Q.Q_IH = cross_quat(Q.Q_Ilat,Q.Q_Ilon);
% DCM.DCM_IH = Q2DCM(Q.Q_IH);

for i = 1:length(Orbit.true_anomalies)
Frames.x_IH(i,:) = [cross_quat(Q.Q_IH,cross_quat(Orbit.x(i,1:4)',conj_quat(Q.Q_IH)))' cross_quat(Q.Q_IH,cross_quat(Orbit.x(i,5:8)',conj_quat(Q.Q_IH)))'];
end

% Transform reference frame from asteroid inertial fixed to asteroid rotational
count = 1;
for t = 0:1:nsteps 
    count = count+1;
    Q.Q_AI(:,count) = [0; 0; sin(Object.w_AI(3)*t*60/2); cos(Object.w_AI(3)*t*60/2)];
    Q.Q_AH(:,count) = cross_quat(Q.Q_AI(:,count),Q.Q_IH);
    Frames.x_AH(count,:) = [cross_quat(Q.Q_AH(:,count),cross_quat(Orbit.x(i,1:4)',conj_quat(Q.Q_AH(:,count))))' cross_quat(Q.Q_AH(:,count),cross_quat(Orbit.x(i,5:8)',conj_quat(Q.Q_AH(:,count))))'];
%     DCM.DCM_AH(3*count-2:3*count,1:3) = Q2DCM(Q.Q_AH(:,count));
end

Frames.Orbit=Orbit;
% Frames.DCM=DCM;
Frames.quat = Q;

end