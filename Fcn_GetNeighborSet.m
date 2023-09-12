%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: get neighboring set 
function output=Fcn_GetNeighborSet(sim_param,robot_state)
    % variable substitution
    r_sense=sim_param.r_sense; 
    swarm_size=sim_param.swarm_size;
    pos_set=robot_state.pos_set;
    % calculate distance matrix
    x_mtr=(pos_set(1,:)'*ones(1,swarm_size));
    y_mtr=(pos_set(2,:)'*ones(1,swarm_size));
    x_rel=x_mtr'-x_mtr;
    y_rel=y_mtr'-y_mtr;
    d_mtr=sqrt((x_rel).^2+(y_rel).^2);
    % calculate adjacent matrix
    a_temp=d_mtr<=(ones(swarm_size,swarm_size).*r_sense);
    a_mtr=a_temp-diag(ones(1,swarm_size));
    % output  substitution    
    output.a_mtr=a_mtr;
    output.d_mtr=d_mtr;
    output.rx_mtr=x_rel.*a_mtr;
    output.ry_mtr=y_rel.*a_mtr;
end