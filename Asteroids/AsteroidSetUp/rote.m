R = [10 10 10];
theta1=215.359362;
theta2=13.113378;
theta3=180.135560;

color1 ='yellow';
axis equal
DCM1 = [1 0 0
    0 1 0
    0 0 1];
Ref_frame_plot(DCM1, R, color1)
axis equal
hold on

color2 ='red';
DCM1 = [1 0 0
  0 cosd(theta2) -sind(theta2)
  0 sind(theta2) cosd(theta2)];
%Ref_frame_plot(DCM1, R, color2)

color3 ='blue';
DCM2 = [cosd(theta2) 0 sind(theta2)
  0 1 0
  -sind(theta2) 0 cosd(theta2)];
%Ref_frame_plot(DCM2*DCM1, R, color3)
% R1 = [x(1) x(2) x(3)];

color4 ='green';
DCM3 = [cosd(theta1) -sind(theta1) 0
  sind(theta1) cosd(theta1) 0
  0 0 1];
%Ref_frame_plot(DCM1*DCM2*DCM3, R, color4)

color5 ='green';
DCM4 = [cosd(theta3) -sind(theta3) 0
  sind(theta3) cosd(theta3) 0
  0 0 1];
Ref_frame_plot(DCM4*DCM1*DCM3, R, color5)


