%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: record library
function lib=Lib_RecordFunctions
    lib.InitRecordState=@InitRecordState;
    lib.UpdateRecordState=@UpdateRecordState;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: initialize record states
function state=InitRecordState(sim_param,robot_state,refer_state,shape_state,rcent_state,metric)
    % variable substitution
    swarm_size=sim_param.swarm_size; 
    max_step=sim_param.max_step;
    % initialize robot-motion variables
    state.pset_rec=zeros(2,swarm_size,max_step+1); 
    state.vset_rec=zeros(2,swarm_size,max_step+1); 
    state.ptra_rec=zeros(2,max_step+1,swarm_size); % use for drawing trajectories
    state.vtra_rec=zeros(2,max_step+1,swarm_size);
    % initialize referecne variables
    state.rpos_rec=zeros(2,max_step+1);           
    state.rvel_rec=zeros(2,max_step+1);
    state.rhead_rec=zeros(1,max_step+1);
    state.rhvel_rec=zeros(1,max_step+1);
    % initialize shape-motion variables
    state.spos_rec=zeros(2,max_step+1,swarm_size);
    state.svel_rec=zeros(2,max_step+1,swarm_size);
    state.shead_rec=zeros(max_step+1,swarm_size); % head angle
    state.shvel_rec=zeros(max_step+1,swarm_size); % head angular velocity
    state.spcen_rec=zeros(2,max_step+1);
    state.shcen_rec=zeros(2,max_step+1);
    % initialize metric variables
    state.cover_rate=zeros(1,max_step+1); 
    state.ent_rate=zeros(1,max_step+1);
    state.dist_var=zeros(1,max_step+1); 
    state.mean_vel=zeros(1,max_step+1); 
    % record robot-motion variables
    state.pset_rec(:,:,1)=robot_state.pos_set; 
    state.vset_rec(:,:,1)=robot_state.vel_set;
    state.ptra_rec(:,1,:)=robot_state.pos_set; 
    state.vtra_rec(:,1,:)=robot_state.vel_set;
    % record referecne variables
    state.rpos_rec(:,1)=refer_state.pos_set;
    state.rvel_rec(:,1)=refer_state.vel_set;  
    state.rhead_rec(:,1)=refer_state.head_set;
    state.rhvel_rec(:,1)=refer_state.hvel_set;
    % record shape-motion variables
    state.spos_rec(:,1,:)=shape_state.pos_set;
    state.svel_rec(:,1,:)=shape_state.vel_set;
    state.shead_rec(1,:)=shape_state.head_set;
    state.shvel_rec(1,:)=shape_state.hvel_set;
    state.spcen_rec(:,1)=rcent_state.pos_set;
    state.shcen_rec(:,1)=rcent_state.head_set;
    % record metric variables
    state.cover_rate(1,1)=metric.cover_rate; 
    state.ent_rate(1,1)=metric.ent_rate;
    state.dist_var(1,1)=metric.dist_var; 
    state.mean_vel(1,1)=metric.mean_vel; 
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: update record states
function state=UpdateRecordState(step,record_state,robot_state,refer_state,shape_state,rcent_state,metric)
    % record robot-motion variables
    record_state.pset_rec(:,:,step+1)=robot_state.pos_set; 
    record_state.vset_rec(:,:,step+1)=robot_state.vel_set;
    record_state.ptra_rec(:,step+1,:)=robot_state.pos_set; 
    record_state.vtra_rec(:,step+1,:)=robot_state.vel_set;
    % record referecne variables
    record_state.rpos_rec(:,step+1)=refer_state.pos_set;
    record_state.rvel_rec(:,step+1)=refer_state.vel_set;  
    record_state.rhead_rec(:,step+1)=refer_state.head_set;
    record_state.rhvel_rec(:,step+1)=refer_state.hvel_set;
    % record shape-motion variables
    record_state.spos_rec(:,step+1,:)=shape_state.pos_set;
    record_state.svel_rec(:,step+1,:)=shape_state.vel_set;
    record_state.shead_rec(step+1,:)=shape_state.head_set;
    record_state.shvel_rec(step+1,:)=shape_state.hvel_set;
    record_state.spcen_rec(:,step+1)=rcent_state.pos_set;
    record_state.shcen_rec(:,step+1)=rcent_state.head_set;
    % record metric variables
    record_state.cover_rate(1,step+1)=metric.cover_rate;
    record_state.ent_rate(1,step+1)=metric.ent_rate;
    record_state.dist_var(1,step+1)=metric.dist_var;
    record_state.mean_vel(1,step+1)=metric.mean_vel;
    % output substitution
    state=record_state; 
end
