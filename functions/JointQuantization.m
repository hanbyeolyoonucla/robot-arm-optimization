function q_Quantized = JointQuantization(q)
    
    q_degree = q*180/pi;
    remain = mod(q_degree ,0.1);
    q_Quantized = zeros(size(q,1),1);
    
    for i=1:size(q,1)
        if remain(i) <= 0.05
            q_Quantized(i) = (q_degree(i)-remain(i))*pi/180;
        else
            q_Quantized(i)=(q_degree(i)-remain(i)+0.1)*pi/180;
        end
    end
end