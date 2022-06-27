function R = rot(axisNum,angle)
    if axisNum == 'x'
        R = [1 0 0;0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)]; 
    elseif axisNum == 'y'
        R = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)]; 
    elseif axisNum == 'z'
         R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1]; 
    else 
            R=0;
    end 
end