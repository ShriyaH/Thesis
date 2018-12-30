function [] = acik_comp(i)
global CONSTANTS ITR

t = ITR.t_k;
T1 = ones(1,length(t)).*CONSTANTS.T1;
T2 = ones(1,length(t)).*CONSTANTS.T2;
gm = CONSTANTS.theta_gm;
gm = ones(1,length(t)).*rad2deg(gm);
gs = CONSTANTS.theta_gs;
gs = ones(1,length(t)).*rad2deg(gs);
ta = CONSTANTS.theta_tilt;
ta = ones(1,length(t)).*rad2deg(ta);
wm = CONSTANTS.w_max;
wm = ones(1,length(t)).*rad2deg(wm);



for ii = 1:i
    for k = 1:30
        x = ITR.x_k{ii}; 
        q = x(8:11,k)./norm(x(8:11,k));
        C = Q2DCM(conj_quat(q));
        %get torque norm
        T(ii,k) = norm(x(15:17,k));
        d(k) = x(15,k)/T(ii,k);
        
        %get gimbal angle
        if x(16,k) < 0
            Gm(ii,k) = -rad2deg(acos(d(k)));
        else
            Gm(ii,k) = rad2deg(acos(d(k)));
        end
        
        %get tilt angle
        Ta_norm(k) = 1-2*(norm(x(9:10,k)))^2; 
        vec1 = conj_quat(q);
        if vec1(3) < 0  
            Ta(ii,k) = -rad2deg(acos(Ta_norm(k))); 
        else
            Ta(ii,k) = rad2deg(acos(Ta_norm(k)));
        end
        
        %get all positions
        r(3*ii-2:3*ii,k) = x(2:4,k); 
        
        %get glide-slope angle
        Gs(ii,k) = rad2deg(atan(x(2,k)/norm(x(3:4,k))));
        
        %get angular rate
        vec2(3*ii-2:3*ii,k) = C*x(12:14,k);
        if vec2(3*ii,k) < 0 
            w(ii,k) = -rad2deg(norm(x(12:14,k)));
        else
            w(ii,k) = rad2deg(norm(x(12:14,k)));
        end
        
        if ii < i
        %quat normalisation
        q_norm(ii,k) = norm(ITR.x_k{ii}(8:11,1));
        end
    end
    diff(ii) = abs(ITR.x_k{end}(1,end) - ITR.x_k{ii}(1,end));
end




end