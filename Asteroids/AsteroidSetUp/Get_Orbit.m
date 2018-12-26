function [state,theta,state_p,theta_p] = Get_Orbit(kep_orbit,mu)
%% Kepler  elements  to  Cartesian  components
%NASA,2014  data
% kep_orbit(1,1) = 6787746.891;
% kep_orbit(1,2) = 0.000731104;
% kep_orbit(1,3) = 51.68714486*pi /180;
% kep_orbit(1,4) = 127.5486706*pi /180;
% kep_orbit(1,5) = 74.21987137*pi /180;
% kep_orbit(1,6) = 24.06608426*pi /180;


format  long

Etemp = 0;
deltaE = 1;

while  abs ( deltaE ) > 1e-8 %lower limit for the difference
    E = Etemp + (kep_orbit(1,6)-Etemp + kep_orbit(1,2)*sin (Etemp) ) /(1-kep_orbit(1,2)*cos (Etemp) ) ; %updating E
    deltaE = E-Etemp ; %difference
    Etemp = E; %for next iteration
end

r = kep_orbit(1,1)*(1-kep_orbit(1,2)*cos  (E) );  %position with respect to kepler elements
theta = 2*atan ( sqrt ((1 + kep_orbit(1,2) ) /(1-kep_orbit(1,2) ) )*tan  (E/2) );  %calculating the true anomaly
% true_anomaly = rad2deg(theta);

S = [ ( r*cos  ( theta ) ) 
    ( r*sin  ( theta ) ) ] ; %calculating perifocal co-ordinates

l1 = cos (kep_orbit(1,4))*cos (kep_orbit(1,5))-sin  (kep_orbit(1,4))*sin  (kep_orbit(1,5))*cos ( kep_orbit(1,3) ) ;
l2 =-cos (kep_orbit(1,4))*sin (kep_orbit(1,5))-sin  (kep_orbit(1,4))*cos  (kep_orbit(1,5))*cos ( kep_orbit(1,3) ) ;
m1 = sin (kep_orbit(1,4))*cos (kep_orbit(1,5)) + cos  (kep_orbit(1,4))*sin  (kep_orbit(1,5))*cos ( kep_orbit(1,3) ) ;
m2 =-sin (kep_orbit(1,4))*sin (kep_orbit(1,5)) + cos  (kep_orbit(1,4))*cos  (kep_orbit(1,5))*cos ( kep_orbit(1,3) ) ;
n1 = sin  (kep_orbit(1,5))*sin  ( kep_orbit(1,3) ) ;
n2 = cos (kep_orbit(1,5))*sin  ( kep_orbit(1,3) ) ;

H = sqrt (mu*kep_orbit(1,1)*(1-kep_orbit(1,2) ^2) ) ; %calculating angular momentum

L = [ l1  l2
      m1 m2
      n1 n2 ] ;

X = L*S ;  %calculating the state vector
V = [ (mu/H*(-l1*sin  ( theta ) + l2*( kep_orbit(1,2) + cos ( theta ) ) ) )    %calculating the velocity vector
      (mu/H*(-m1*sin  ( theta ) + m2*( kep_orbit(1,2) + cos ( theta ) ) ) )
      (mu/H*(-n1*sin  ( theta ) + n2*( kep_orbit(1,2) + cos ( theta ) ) ) ) ] ;
  
state = [X
       V];
Cartesian = { 'x ' ;  'y ' ;  ' z ' ;  ' xdot ' ;  ' ydot ' ;  ' zdot ' };
Values = [X(1) ; X(2) ; X(3) ; V(1) ; V(2) ; V(3) ] ;
T = table ( Cartesian ,  Values );


%% State at pericentre

format  long
E_p=0;

r_p = kep_orbit(1,1)*(1-kep_orbit(1,2)*cos  (E_p) );  %position with respect to kepler elements
theta_p = 2*atan ( sqrt ((1 + kep_orbit(1,2) ) /(1-kep_orbit(1,2) ) )*tan  (E_p/2) );  %calculating the true anomaly
% true_anomaly_p = rad2deg(theta_p);

S_p = [ ( r_p*cos  ( theta_p ) ) 
    ( r_p*sin  ( theta_p ) ) ] ; %calculating perifocal co-ordinates

l1_p = cos (kep_orbit(1,4))*cos (kep_orbit(1,5))-sin  (kep_orbit(1,4))*sin  (kep_orbit(1,5))*cos ( kep_orbit(1,3) ) ;
l2_p =-cos (kep_orbit(1,4))*sin (kep_orbit(1,5))-sin  (kep_orbit(1,4))*cos  (kep_orbit(1,5))*cos ( kep_orbit(1,3) ) ;
m1_p = sin (kep_orbit(1,4))*cos (kep_orbit(1,5)) + cos  (kep_orbit(1,4))*sin  (kep_orbit(1,5))*cos ( kep_orbit(1,3) ) ;
m2_p =-sin (kep_orbit(1,4))*sin (kep_orbit(1,5)) + cos  (kep_orbit(1,4))*cos  (kep_orbit(1,5))*cos ( kep_orbit(1,3) ) ;
n1_p = sin  (kep_orbit(1,5))*sin  ( kep_orbit(1,3) ) ;
n2_p = cos (kep_orbit(1,5))*sin  ( kep_orbit(1,3) ) ;

H_p = sqrt (mu*kep_orbit(1,1)*(1-kep_orbit(1,2) ^2) ) ; %calculating angular momentum

L_p = [ l1_p  l2_p
      m1_p m2_p
      n1_p n2_p ] ;

X_p = L_p*S_p ;  %calculating the state vector
V_p = [ (mu/H_p*(-l1_p*sin  ( theta_p ) + l2_p*( kep_orbit(1,2) + cos ( theta_p ) ) ) )    %calculating the velocity vector
      (mu/H_p*(-m1_p*sin  ( theta_p ) + m2_p*( kep_orbit(1,2) + cos ( theta_p ) ) ) )
      (mu/H_p*(-n1_p*sin  ( theta_p ) + n2_p*( kep_orbit(1,2) + cos ( theta_p ) ) ) ) ] ;
  
state_p = [X_p
       V_p];

end
