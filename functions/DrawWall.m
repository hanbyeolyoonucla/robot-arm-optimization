function DrawWall(TopCircleCenter,Diameter,NormalVector,Height)
    
%     TopCircleCenter = [0;0;0];
%     NormalVector = [0;0;1];
%     Diameter = 1;
%     Height = 1;
    theta=0:0.15:2*pi;
    points2 = zeros(3,2*length(theta));
    v = null(NormalVector'); % Normal vector is 3x1, then two v are 3x1
    h = repmat(NormalVector/norm(NormalVector)*Height,1,size(theta,2));
    points = repmat(TopCircleCenter,1,size(theta,2))+Diameter/2*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
    top = patch(points(1,:),points(2,:),points(3,:),'');
    bottom = patch(points(1,:)-h(1,:),points(2,:)-h(2,:),points(3,:)-h(3,:),'');
%     set(top, 'FaceColor', [0 1 0],'FaceAlpha', 0.99, 'linestyle', 'none', 'SpecularStrength', 0.7);
%     set(bottom, 'FaceColor', [0 1 0],'FaceAlpha', 0.99, 'linestyle', 'none', 'SpecularStrength', 0.7);
   
    for i=1:length(theta)
        points2(:,2*i-1) = points(:,i);
        points2(:,2*i) = points(:,i)-h(:,i);
    end
    
    for i=1:2*length(theta)-2
        outer = patch(points2(1,i:i+2),points2(2,i:i+2),points2(3,i:i+2),'');
        set(outer, 'FaceColor', [0 0.7 0.3],'FaceAlpha', 0.1, 'linestyle', 'none','SpecularStrength', 0.7);
    end

end