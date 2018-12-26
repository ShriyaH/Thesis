%% Rotate and translate vector (Dual quaternion)

%Define the initial attitude
q = [0;0;0;1]; %unit quaternion
d = Q2DCM([0;0;0;1]); %convert to DCM

%Define the translation and rotation to be obtained
r = [4;3;2];  %translation vector
q1 = [1;0;0;0];  %new attitude quaternion
d1 = Q2DCM(q1); 

%Convert to DQ by rotation or translation first
dq = Q2DQ(q1,r,1); 

%Compute dual quaternion conjugate
dq_c = conj_dq(dq,3);

%Define vector to be transformed
v = [1;1;1];
q = [0;0;0;1]; %unit quaternion with no rotation

%Convert vector to DQ format
dv = [q;v;0];

%DQ rotation product (dq x dv x dq^*)
%DQ with new attitude and rotated and translated vector in the initial frame of reference
vr = cross_dq(cross_dq(dq,dv),dq_c); 

%Rotated vector in the rotated and translated frame
r2 = vr(5:7)-r;

scale=1; %arrow scale for quiver3

figure()
axis equal
grid on
Ref_frame_plot(d,[0;0;0]); %plot initial ref frame
Ref_frame_plot(d1,r);      %plot new ref frame
hold on
quiver3(0,0,0,v(1),v(2),v(3),scale); %plot vector in initial ref frame
quiver3(r(1),r(2),r(3),r2(1),r2(2),r2(3),scale); %plot rotated vector in new ref frame
quiver3(0,0,0,r(1),r(2),r(3),scale,'Color','black');  %plot translation vector from initial ref frame
plotCircle3D([0,1.5,1],[1,0,0],1.81);
plotCircle3D([4,1.5,1],[1,0,0],1.81);
quiver3(0,1.5,1,4,0,0,scale,'Color','black');  %plot translation vector from initial ref frame
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');


