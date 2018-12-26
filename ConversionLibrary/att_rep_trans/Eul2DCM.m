function [DCM] = Eul2DCM(euler_angle,sequence)
%% This function is to calculate the DCM and quaternion for a particular rotation sequence with known Euler angles.
% % Author: Shriya Hazra

% Specify sequence as 'XYZ', 'ZYZ', 'ZZZ' and so on. Specify the array for Euler
% angles as per sequence, so for the case 'ZXY' first angle for Z axis
% then X and then Y. (specify angles in rad)
%%--------------------------------------------------------------------------------- 
 
order = [];
for i=1:3
    if sequence(i) == 'X'
        order(1,i) = 1;
    elseif sequence(i) == 'Y'
        order(1,i) = 2;
    elseif sequence(i) == 'Z'
        order(1,i) = 3;
end  
end

for j =1:3    
    if order(j) == 1
        R{j} = [1 0 0                                       %Rotation about the x-axis
                0 cos(euler_angle(j)) sin(euler_angle(j))
                0 -sin(euler_angle(j)) cos(euler_angle(j))];
    elseif order(j) == 2
        R{j} = [cos(euler_angle(j)) 0 -sin(euler_angle(j))   %Rotation about the y-axis
                0 1 0
                sin(euler_angle(j)) 0 cos(euler_angle(j))];
    elseif order(j) == 3
        R{j} = [cos(euler_angle(j)) sin(euler_angle(j)) 0    %Rotation about the z-axis
                -sin(euler_angle(j)) cos(euler_angle(j)) 0
                0 0 1]; 
    end
end

% 
% %Compute the DCM matrix for the rotation
% DCM = R{3}*R{2}*R{1};
% 
% %Compute the Quaternion for the rotation
% q1 = sqrt(1/4*(1 + DCM(1,1) - DCM(2,2) - DCM(3,3)));
% q2 = sqrt(1/4*(1 - DCM(1,1) + DCM(2,2) - DCM(3,3)));
% q3 = sqrt(1/4*(1 - DCM(1,1) - DCM(2,2) + DCM(3,3)));
% q4 = sqrt(1/4*(1 + DCM(1,1) + DCM(2,2) + DCM(3,3)));
% 
% q_norm = norm([q1 q2 q3 q4]); %Norm of the computed quaternion
% 
% Q.rot = [q1;q2;q3;q4]./q_norm;  %Unit quaternion for rotation

end
   
   