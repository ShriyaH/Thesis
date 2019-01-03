%%ScLERP test
%Define the initial DQ
qi = [0;0;0;1]; %unit quaternion
ri = [4;3;2];  %translation vector
dqi = Q2DQ(qi,ri,1); 

%Define the final DQ
theta = deg2rad(65);
qf = [sin(theta/2);sin(theta/2);sin(theta/2);cos(theta/2)]; 
qf = qf./norm(qf);  %unit quat
rf = [10;-5;3];  %translation vector
dqf = Q2DQ(qf,rf,1); 

%Define the time vector
t = linspace(0,5,30);

%ScLERP operation
[dqm,r,ang] = ScLERP(dqi, dqf, t);

%Convert to DQ by rotation or translation first
dq = Q2DQ(q1,r,1); 