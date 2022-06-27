function adV=adjointV(V)
w=V(1:3,1);
v=V(4:6,1);
skeww = [0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
skewv = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
adV=[skeww zeros(3,3);skewv skeww];
end