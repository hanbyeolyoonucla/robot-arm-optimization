function T_inv = SE3Inverse(T)
    R = T(1:3,1:3);
    p = T(1:3,4);
    T_inv = [R.' -R.'*p; zeros(1,3) 1];
end