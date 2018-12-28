function [ a,ecc,i,Omega,omega,theta,M ] = C2Kp( mu, R_I, V_I )

V_bar=V_I;
r_bar=R_I;

V=norm(V_I);
r=norm(r_bar);
r_hat=r_bar/r;

h=cross(r_bar,V_bar);
norm_h=norm(h);

N=cross([0 0 1],h);
N_xy=sqrt(N(1)^2+N(2)^2);
N_hat=N/norm(N);

a=1/(2/r-V^2/mu);
e_bar=1/mu*cross(V_bar, h)-r_bar/r;
ecc=norm(e_bar);

if ecc==0
    e_hat=[0 0 0]'
else
    e_hat=e_bar/ecc;
end
i=acos(h(3)/norm_h)*180/pi;

Omega=atan2(N(2)/N_xy, N(1)/N_xy)*180/pi;
Omega=mode(Omega, 360);

if dot(cross(N_hat, e_hat), h)>0
    sign_omega=1;
else
    sign_omega=-1;
end

omega=sign_omega*acos(dot(e_hat, N_hat))*180/pi;
omega=mod(omega,360);

if dot(cross(e_hat, r_hat), h)>0
    sign_theta=1;
else
    sign_theta=-1;
end
d=dot(r_hat, e_hat);
if d>1
    d=1;
end
theta=sign_theta*acos(d);

E=2*atan(sqrt((1-ecc)/(1+ecc))*tan(theta/2));
M=(E-ecc*sin(E))*180/pi;
M=mod(M,360);
theta=theta*180/pi;
theta=mod(theta,360);

end
