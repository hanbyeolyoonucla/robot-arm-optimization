function score = keepOutZone(score, alpha_space, beta_space, x_space, y_space, z_space)

for ii = 1:length(beta_space)
    for jj = 1:length(alpha_space)
        Rvis = rot('x',-beta_space(ii))*rot('y',alpha_space(jj))*rot('z',pi/2);
        for kk = 1:length(x_space)
            for ll = 1:length(y_space)
                for mm = 1:length(z_space)
                    
                    % Rotation from Robot to World
                    p_WP = Rvis * [x_space(kk); y_space(ll); z_space(mm)];
                    
                    % Check Head Constraints
                    if p_WP(1) <= 0.279 && p_WP(1) >= -0.1
                        if p_WP(2) <= 0.425/2 && p_WP(2) >= -0.425/2
                            if p_WP(3) >= -0.11
                                if score(ii,jj,kk,ll,mm) ~= 0
%                                     score(ii,jj,kk,ll,mm) = -1;
                                    score(ii,jj,kk,ll,mm) = 0;
                                end
                            end
                        end
                        % Check Shoulder Constraints
                    elseif p_WP(1) < -0.1
                        if p_WP(2) <= 0.763/2 && p_WP(2) >= -0.763/2
                            if p_WP(3) >= -0.11
                                if score(ii,jj,kk,ll,mm) ~= 0
%                                     score(ii,jj,kk,ll,mm) = -1;
                                    score(ii,jj,kk,ll,mm) = 0;
                                end
                            end
                        end
                    end
                    
                end
            end
        end
    end
end
end