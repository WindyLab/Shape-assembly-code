%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: update functions
function lib=Lib_UpdateFunctions
    lib.UpdateRobotMotion=@UpdateRobotMotion;
    lib.UpdateRecordState=@UpdateRecordState;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: update robot motion
function state=UpdateRobotMotion(sim_param,robot_state,cmd_set)
    % variable substitution
    t=sim_param.t; 
    vel_max=sim_param.vel_max;
    pos_set=robot_state.pos_set; 
    vel_set=robot_state.vel_set;
    % limit control input (max speed)
    vscl_set=sqrt(cmd_set(1,:).^2+cmd_set(2,:).^2);
    head_set=atan2(cmd_set(2,:),cmd_set(1,:));
    index_set=find(vscl_set>vel_max);
    cmd_set(:,index_set)=vel_max.*[cos(head_set(index_set));sin(head_set(index_set))];
    % upate motion states    
    ratio=0.1;
    state.pos_set=pos_set+vel_set.*t;  
    state.vel_set=ratio.*cmd_set+(1-ratio).*vel_set;      
end

