function[error]=compute_attitude_visualization(r,q,scale,h,test_flag)
% This function visualize the attitude with rgb convention for the axes
% x,y,z in the figure pointed by the handle "h";
% q is meant as q from body to external reference frame, and having the
% components [qx qy qz qr]
% r is expressed as the position of body w.r.t. the same external reference
% frame
%
%
% Marco Sagliano, DLR, 24-Jan-2013



% TEST CASE - to run test case
if nargin<5
    test_case = 0;
else
    test_case = test_flag;
end
if test_case
    clear r q;
    
    t = linspace(0,pi/2,10);
    r0 = 5;
    r = [r0*cos(t); r0*sin(t); zeros(size(t))];
    
    angle = pi/180*linspace(180,270,10);
    
    for jj = 1:length(angle)
        q(:,jj) = DCM2Q(compute321_rotation_matrix(angle(jj),0,0)');
    end
end

% input analysis
if size(r,1) ~=3
    disp('error in input definition: r must have dimensions 3 x n');
    [x1_u,y1_u,z1_u,x2_u,y2_u,z2_u,x3_u,y3_u,z3_u]=no_output;
    return
end
if size(q,1) ~=4
    disp('error in input definition: q must have dimensions 4 x n');
    [x1_u,y1_u,z1_u,x2_u,y2_u,z2_u,x3_u,y3_u,z3_u]=no_output;
    return
end
if size(r,2)~=size(q,2)
    disp('error in input consistence: q and r must represent the same number of points');
    [x1_u,y1_u,z1_u,x2_u,y2_u,z2_u,x3_u,y3_u,z3_u]=no_output;
    return
end


% convert attitude from q into M
M = Q2DCM_vec(q);

% transform the unitary vectors x_b,y_b,z_b in the new reference frame
[x,y,z]=unitary_vector_body(M,scale);

% apply the vectors in the points of the trajectory
[rx,ry,rz]=shift_vectors(r,x,y,z);

% generate the vector field x_u,y_u,z_u
[x1_u,y1_u,z1_u,x2_u,y2_u,z2_u,x3_u,y3_u,z3_u] = generate_vector_field(r,rx,ry,rz);

% visualize
error = visualize_attitude(x1_u,y1_u,z1_u,x2_u,y2_u,z2_u,x3_u,y3_u,z3_u,h);

% test
if test_case
    % figure;
    plot3(r(1,:),r(2,:),r(3,:),'r','LineWidth',2);
    hold on;
    grid on;
    for jj = 1:size(r,2)
        plot3(x1_u(jj,:),y1_u(jj,:),z1_u(jj,:),'r','LineWidth',2);
        plot3(x2_u(jj,:),y2_u(jj,:),z2_u(jj,:),'g','LineWidth',2);
        plot3(x3_u(jj,:),y3_u(jj,:),z3_u(jj,:),'b','LineWidth',2);
    end
    axis equal;
    xlabel('X');
    ylabel('Y');
end


function DCM = Q2DCM_vec(q_vec)

for jj = 1:size(q_vec,2)
    q=q_vec(:,jj);
    
    qx=[0 -q(3) q(2);q(3) 0 -q(1);-q(2) q(1) 0];
    
    DCM(:,:,jj)=(q(4)^2-dot(q(1:3),q(1:3)))*eye(3)+2*q(1:3)*q(1:3)'-2*q(4)*qx;
end

function[x,y,z]=unitary_vector_body(M,scale);

if length(scale)==1
    x1 = scale*[1; 0; 0]; y1 = scale*[0; 1; 0]; z1 = scale*[0; 0; 1];
elseif length(scale)==3
    x1 = scale(1)*[1; 0; 0]; y1 = scale(2)*[0; 1; 0]; z1 = scale(3)*[0; 0; 1];
end



for jj = 1:size(M,3)
    x(:,jj) = M(:,:,jj)*x1;
    y(:,jj) = M(:,:,jj)*y1;
    z(:,jj) = M(:,:,jj)*z1;
end

function[rx,ry,rz]=shift_vectors(r,x,y,z);

rx= r+x;
ry = r+y;
rz = r+z;

function[x1_u,y1_u,z1_u,x2_u,y2_u,z2_u,x3_u,y3_u,z3_u] = generate_vector_field(r,rx,ry,rz);

for jj = 1:size(r,2)
    x1_u(jj,:) = [r(1,jj) rx(1,jj)]; y1_u(jj,:) = [r(2,jj) rx(2,jj)]; z1_u(jj,:) = [r(3,jj) rx(3,jj)];
    x2_u(jj,:) = [r(1,jj) ry(1,jj)]; y2_u(jj,:) = [r(2,jj) ry(2,jj)]; z2_u(jj,:) = [r(3,jj) ry(3,jj)];
    x3_u(jj,:) = [r(1,jj) rz(1,jj)]; y3_u(jj,:) = [r(2,jj) rz(2,jj)]; z3_u(jj,:) = [r(3,jj) rz(3,jj)];
end

function[x1_u,y1_u,z1_u,x2_u,y2_u,z2_u,x3_u,y3_u,z3_u]=no_output;

x1_u = NaN; y1_u = NaN; z1_u = NaN;
x2_u = NaN; y2_u = NaN; z2_u = NaN;
x3_u = NaN; y3_u = NaN; z3_u = NaN;

function error = visualize_attitude(x1_u,y1_u,z1_u,x2_u,y2_u,z2_u,x3_u,y3_u,z3_u,h);
% This function visualize the attitude with rgb convention for the axes
% x,y,z in the figure pointed by the handle "h";
%
% Marco Sagliano, DLR, 24-Jan-2013
error = 0;
try
    % hold on;
    for jj = 1:size(x1_u,1)
        plot3(x1_u(jj,:),y1_u(jj,:),z1_u(jj,:),'r','LineWidth',2);
        plot3(x2_u(jj,:),y2_u(jj,:),z2_u(jj,:),'g','LineWidth',2);
        plot3(x3_u(jj,:),y3_u(jj,:),z3_u(jj,:),'b','LineWidth',2);
    end
    
catch
    error = 1;
    disp('error in attitude visualization');
    return
end

function M=compute321_rotation_matrix(phi,theta,psi)
%%% phi around Z, theta around Y psi around X
for jj=1:length(phi)
    cphi=cos(phi(jj));
    sphi=sin(phi(jj));
    ctheta=cos(theta(jj));
    stheta=sin(theta(jj));
    cpsi=cos(psi(jj));
    spsi=sin(psi(jj));
    
    M(:,:,jj)=[ctheta*cphi ctheta*sphi -stheta;
        -cpsi*sphi+spsi*stheta*cphi cpsi*cphi+spsi*stheta*sphi spsi*ctheta;
        spsi*sphi+cpsi*stheta*cphi -spsi*cphi+cpsi*stheta*sphi cpsi*ctheta];
end
% DCM2Q.m
% Function to convert DCM to quaternion.  Uses the most numerically accurate
% method depending on DCM values.
%
% Inputs:
%     DCM - The DCM representation of q
%
% Outputs:
%     q - The quaternion to use in (vector,scalar) form
%
% Notes:
% See J. R. Wertz, "Spacecraft Attitude Determination and Control", 1978
% eq 12-14
function q = DCM2Q(DCM)
%#eml
q=zeros(4,1);
b=zeros(4,1);

tr=DCM(1,1)+DCM(2,2)+DCM(3,3);
b(1)=(1+tr)*0.25;
b(2)=(1+2*DCM(1,1)-tr)*0.25;
b(3)=(1+2*DCM(2,2)-tr)*0.25;
b(4)=(1+2*DCM(3,3)-tr)*0.25;

[v,i]=max(b);
switch i,
    case 1
        q(4)=sqrt(b(1));
        temp=0.25/q(4);
        q(1)=(DCM(2,3)-DCM(3,2))*temp;
        q(2)=(DCM(3,1)-DCM(1,3))*temp;
        q(3)=(DCM(1,2)-DCM(2,1))*temp;
    case 2
        q(1)=sqrt(b(2));
        temp=0.25/q(1);
        q(4)=(DCM(2,3)-DCM(3,2))*temp;
        q(2)=(DCM(2,1)+DCM(1,2))*temp;
        q(3)=(DCM(3,1)+DCM(1,3))*temp;
    case 3
        q(2)=sqrt(b(3));
        temp=0.25/q(2);
        q(4)=(DCM(3,1)-DCM(3,3))*temp;
        q(1)=(DCM(1,2)+DCM(2,1))*temp;
        q(3)=(DCM(2,3)+DCM(3,2))*temp;
    case 4
        q(3)=sqrt(b(4));
        temp=0.25/q(3);
        q(4)=(DCM(1,2)-DCM(2,1))*temp;
        q(1)=(DCM(3,1)+DCM(1,3))*temp;
        q(2)=(DCM(2,3)+DCM(3,2))*temp;
end

if(q(4)<0)
    q=-q;
end

