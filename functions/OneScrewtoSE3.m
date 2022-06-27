function [ T ] = OneScrewtoSE3(screw_i,q_i)
% se3 ( input : ith screw, ith joint angle )

    w = screw_i(1:3,1);
    v = screw_i(4:6,1);
    
    skeww = [0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
    A = [skeww v; 0 0 0 0] * q_i;
    
    T = expm(A) ;
end