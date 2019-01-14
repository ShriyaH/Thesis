function [r_A,r_B]= convt_r(Y)

r_A = zeros(length(Y),4);
r_B = zeros(length(Y),3);
C = zeros(3*length(Y),3);

for i = 1:length(Y)
r_A(i,:) = 2.*cross_quat(conj_quat(Y(i,1:4)'),Y(i,5:8)')';
C(3*i-2:3*i,:) = Q2DCM(Y(i,1:4)');
r_B(i,:) = (C(3*i-2:3*i,:)*r_A(i,1:3)')';
end

end