function [eul] = dcm2eul(dcm)

    if abs(dcm(3,1))~= 1
        theta1 = -sin(dcm(3,1));
        theta2 = pi - theta1;

        psi1 = atan2(dcm(3,2)/cos(theta1),dcm(3,3)/cos(theta1));
        psi2 = atan2(dcm(3,2)/cos(theta2),dcm(3,3)/cos(theta2));

        phi1 = atan2(dcm(2,1)/cos(theta1),dcm(1,1)/cos(theta1));
        phi2 = atan2(dcm(2,1)/cos(theta2),dcm(1,1)/cos(theta2));
        eul = [theta1 psi1 phi1; theta2 psi2 phi2];
        
    elseif dcm(3,1) == 1
        theta = pi/2;
        psi = phi + atan2(dcm(1,2),dcm(1,3));
    else
        theta = -pi/2;
        psi = -phi + atan2(-dcm(1,2),-dcm(1,3));
    end

   
end