function [Traj] = Prop_samples_polygrav(n,D_sph,Asteroid)

w = Asteroid.w_AI(3);
mu = Asteroid.mu;
a = a_G + a_SRP + a_3BP;
W_f = ww;

for i = 1:n
    
T = [0 2000];
Y0(:,i)=[D_sph(1:3,i);D_sph(5:7,i)];     %Initial domain space

Opt = odeset('Event',@stopevent,'AbsTol',1e-10,'RelTol',2.3e-10);
[t{i},y{i},te{i},ye{i},ie{i}] = ode45(@orb_int, T, Y0(:,i),Opt);  
    
rho{i} = sqrt(y{i}(end,1)^2 + y{i}(end,2)^2 + y{i}(end,3)^2);
ellip{i} = (y{i}(end,1)/a)^2 + (y{i}(end,2)/b)^2 + (y{i}(end,3)/c)^2;
acc{i} = sqrt((mu*y{i}(end,1)/rho{i}^3)^2 + (mu*y{i}(end,2)/rho{i}^3)^2 + (mu*y{i}(end,2)/rho{i}^3)^2);

    for j = 1:length(y{i})
        q_AI{i}(1:4,j) = [0; 0; sin(w*t{i}(j)/2); cos(w*t{i}(j)/2)];
        y_A{i}(j,1:3) = quat_trans(q_AI{i}(1:4,j),y{i}(j,1:3)','vect');
    end
end

Traj.t = t;
Traj.y = y;
Traj.te = te;
Traj.ye = ye;
Traj.ie = ie;
Traj.y_A = y_A;
Traj.q_AI = q_AI;
Traj.end_prop.rho = rho;
Traj.end_prop.ell = ellip;
Traj.end_prop.accl = acc;

function [value, isterminal, direction] = stopevent(T,Y,W_f)
esc = sqrt((mu*Y(1)/r^3)^2 + (mu*Y(2)/r^3)^2 + (mu*Y(3)/r^3)^2);  %Instantaneous acceleration
esc2 = sqrt(3*mu/esc)/4;                                          %Maximum allowable acceleration at that position

value = [W_f-4*pi; esc2-esc];    %Impact condition, Escape condition
% if abs(value)<=0
%     value = 0;
% end
isterminal = [1; 1];          %Stop the integration
direction  = [0; 0];          %Bi-directional approach
end


function dV = orb_vel(T,a)
r = sqrt(Y(1)^2 + Y(2)^2 + Y(3)^2);
dV = [Y(4); Y(5); Y(6); a(1); a(2); a(3)];
end

function dY = orb_pos(T,dV)
r = sqrt(Y(1)^2 + Y(2)^2 + Y(3)^2);
dY = [Y(4); Y(5); Y(6); -mu*Y(1)/r^3; -mu*Y(2)/r^3; -mu*Y(3)/r^3];
end

end