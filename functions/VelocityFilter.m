function q_dot_est = VelocityFilter(q_est, q_est_,dt)
    q_dot_est=(q_est-q_est_)/dt;
end