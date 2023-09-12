%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: calculate interaction velocity command
function cmd_set=Fcn_CalInteractionCmd(sim_param,neigh_mtr,robot_state)
    % set proportional gain
    kappa_1=25;
    kappa_2=1;
    % calculate avoidance command
    cmd_avoid=CalAvoidanceCmd(sim_param,neigh_mtr);
    % calculate consensus command
    cmd_conse=CalConsensusCmd(sim_param,neigh_mtr,robot_state);
    % calculate interaction command
    cmd_set=kappa_1.*cmd_avoid-kappa_2.*cmd_conse;
end

%% Auxiliary functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: calculate avoidance command
function cmd_set=CalAvoidanceCmd(sim_param,neigh_mtr)
    % variable substitution
    r_avoid=sim_param.r_avoid; 
    a_mtr=neigh_mtr.a_mtr; 
    d_mtr=neigh_mtr.d_mtr; 
    rx_mtr=neigh_mtr.rx_mtr; 
    ry_mtr=neigh_mtr.ry_mtr; 
    % calculate command
    dis_mtr=d_mtr+(~a_mtr).*r_avoid;
    temp=r_avoid-d_mtr; 
    temp(temp<=0)=0;
    spr_mtr=temp./dis_mtr;
    unit_x=-rx_mtr./dis_mtr;
    unit_y=-ry_mtr./dis_mtr;
    cmd_x=sum(spr_mtr.*unit_x,2);
    cmd_y=sum(spr_mtr.*unit_y,2);    
    cmd_set=[cmd_x';cmd_y']; 
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: calculate consensus command
function cmd_set=CalConsensusCmd(sim_param,neigh_mtr,robot_state)
    % variable substitution
    a_mtr=neigh_mtr.a_mtr;     
    vel_set=robot_state.vel_set;
    swarm_size=sim_param.swarm_size;
    % calculate the relative velocity matrix
    vx_mtr=(vel_set(1,:)'*ones(1,swarm_size));
    vy_mtr=(vel_set(2,:)'*ones(1,swarm_size));
    vx_rel=(vx_mtr-vx_mtr').*a_mtr;
    vy_rel=(vy_mtr-vy_mtr').*a_mtr;
    % calculate control force
    cmd_x=sum(vx_rel,2);
    cmd_y=sum(vy_rel,2);
    neigh_num=sum(a_mtr,2); 
    neigh_num(neigh_num==0)=1;    
    cmd_set=[cmd_x';cmd_y']./neigh_num';
end