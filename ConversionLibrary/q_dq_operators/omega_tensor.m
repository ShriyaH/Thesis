function [W] = omega_tensor(w,n)
%Case 1: cross product tensor
%Case 2: quaternion cross product tensor
%Case 3: dual quaternion cross product
%Case 4: dual number cross product

switch n
    case 1
        W = [0 -w(3) w(2)
             w(3) 0 -w(1)
             -w(2) w(1) 0];
    case 2
        W = [0 w(3) -w(2) w(1)
             -w(3) 0 w(1) w(2)
             w(2) -w(1) 0 w(3)
             -w(1) -w(2) -w(3) 0];
    case 3
        W = [0 w(3) -w(2) w(1) 0 0 0 0 
             -w(3) 0 w(1) w(2) 0 0 0 0 
             w(2) -w(1) 0 w(3) 0 0 0 0
             -w(1) -w(2) -w(3) 0 0 0 0 0
             0 w(7) -w(6) w(5) 0 w(3) -w(2) w(1)
             -w(7) 0 w(5) w(6) -w(3) 0 w(1) w(2)
             w(6) -w(5) 0 w(7) w(2) -w(1) 0 w(3)
             -w(5) -w(6) -w(7) 0 -w(1) -w(2) -w(3) 0];
      
    case 4
        W = [0 -w(3) w(2) w(1) 0 0 0 0 
             w(3) 0 -w(1) w(2) 0 0 0 0 
             -w(2) w(1) 0 w(3) 0 0 0 0
             0 0 0 0 0 0 0 0
             0 -w(7) w(6) w(5) 0 -w(3) w(2) w(1)
             w(7) 0 -w(5) w(6) w(3) 0 -w(1) w(2)
             -w(6) w(5) 0 w(7) -w(2) w(1) 0 w(3)
             0 0 0 0 0 0 0 0];
         
    otherwise
        disp('Wrong selection of type of tensor');
end
 
end