function J = SJacobian(S,q,n_joint)

    POE = eye(4);
    J = zeros(6,n_joint);
    for i=1:n_joint
        J(:,i) = AdjointT(POE)*S(:,i);
        POE = POE*OneScrewtoSE3(S(:,i),q(i));
    end

end