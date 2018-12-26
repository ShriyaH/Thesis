function [ R_I,V_I,Q_r,DCM,R_A,Q_SC, SC] = Get_Rosetta_data(epoch, dt)
cspice_furnsh( 'SPICE/standard_R.txt' )


% epoch1    = 'October 15, 2014 11:00 AM UTC';
% epoch2    = 'October 20, 2014 11:00 AM UTC';

et1 = cspice_str2et( epoch)
% et2 = cspice_str2et( epoch2 );


ett=et1+dt;

[state, lt] = cspice_spkezr('ROSETTA', ett,'J2000', 'NONE', '67P/C-G');
[state2, lt] = cspice_spkezr('ROSETTA', ett,'67P/C-G_FIXED', 'NONE', '67P/C-G');


DCM= cspice_pxform('67P/C-G_FIXED' , 'J2000', ett );
SC= cspice_pxform('J2000' , 'ROS_SPACECRAFT', ett );

DCM=permute(DCM,[2 1 3]);
cspice_unload('standard_R.txt')
R_A=state2(1:3,:);
R_I=state(1:3,:);
V_I=state(4:6,:);
Qw=dcm2quat(DCM);
Q_r=[Qw(:,2:4) Qw(:,1)];

Qs=dcm2quat(SC);
Q_SC=[Qs(:,2:4) Qs(:,1)];
end
