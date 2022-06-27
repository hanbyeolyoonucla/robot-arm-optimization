function [ T ] = ScrewstoSE3( Screw , q )

    T = eye(4);
    for i = 1:size(Screw,2)
    T = T * OneScrewtoSE3( Screw(:,i), q(i,1) ) ;
    end
    
end