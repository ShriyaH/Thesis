%%ScLERP test
%Define the initial DQ
qi = [0;0;0;1]; %unit quaternion
ri = [4;3;2];  %translation vector
dqi = Q2DQ(qi,ri,1); 
angi = 2*rad2deg(acos(qi(4)));

%Define the final DQ
qf = Eul2Q([pi/4,pi/4,pi/4],'XYZ');
angf = 2*rad2deg(acos(qf(4)));
rf = [10;-5;3];  %translation vector
dqf = Q2DQ(qf,rf,1); 

%Define the time vector
t = linspace(0,5,30);

%ScLERP operation
[dqm,r,r_norm,ang] = ScLERP(dqi, dqf, t);

figure()
axis equal
grid on         
plot(t,ang);
xlabel('Time (sec)');
ylabel('Rotation (deg)');

figure()
axis equal
grid on         
plot(t,r_norm);
xlabel('Time (sec)');
ylabel('Rotation (deg)');

figure()
axis equal
grid on
plot3(r(1,1:30),r(2,1:30),r(3,1:30));          
xlabel('Time (sec)');
ylabel('Position (m)');