%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: negotiate formation position
function state=Fcn_NegotPositState(sim_param,neigh_mtr,shape_state,refer_state,inform_index)
    % set proportional gain
    kappa_conse=1.6;
    kappa_track=3.0;
    alpha=0.8;
    % variable substitution
    t=sim_param.t;
    a_mtr=neigh_mtr.a_mtr;
    pos_set=shape_state.pos_set;
    vel_set=shape_state.vel_set;
    ref_pos=refer_state.pos_set;
    ref_vel=refer_state.vel_set;
    swarm_size=sim_param.swarm_size;
    % calculate consensus command for all the robots
    err_set=GetConsensusError(swarm_size,a_mtr,pos_set);
    [sign_set,pabs_set]=SepSymbAndValue(err_set);
    cmd_cons=kappa_conse.*sign_set.*(pabs_set.^alpha);
    % calculate reference-tracking command for informed robots
    cmd_track=(kappa_track.*(ref_pos-pos_set)+ref_vel);
    % calculate alignment command for uninformed robots
    cmd_align=GetAlignConsensus(swarm_size,a_mtr,vel_set);
    % get valid set of informed robots
    inform_set=zeros(1,swarm_size);
    inform_set(inform_index)=1;
    % calculate velocity command    
    cmd_set=-cmd_cons+inform_set.*cmd_track+(~inform_set).*cmd_align;
    % update negotiation movements
    state=shape_state;
    state.pos_set=state.pos_set+state.vel_set.*t;
    state.vel_set=cmd_set;
end

%% Auxiliary functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: calculate alignment consensus
function cons_set=GetAlignConsensus(swarm_size,a_mtr,state_set)
    x_mtr=kron(state_set(1,:),ones(swarm_size,1));
    y_mtr=kron(state_set(2,:),ones(swarm_size,1));
    x_set=sum(x_mtr.*a_mtr,2);
    y_set=sum(y_mtr.*a_mtr,2);
    neigh_num=sum(a_mtr,2)';
    neigh_num(neigh_num==0)=1;
    cons_set=[x_set,y_set]'./neigh_num;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: calculate consensus errors
function [sign_set,abs_set]=SepSymbAndValue(value_set)
    sign_set=sign(value_set);
    abs_set=abs(value_set);
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: calculate consensus errors
function err_set=GetConsensusError(swarm_size,a_mtr,state_set)
    x_mtr=kron(state_set(1,:),ones(swarm_size,1));
    y_mtr=kron(state_set(2,:),ones(swarm_size,1));
    x_rel=(x_mtr'-x_mtr).*a_mtr;
    y_rel=(y_mtr'-y_mtr).*a_mtr;
    neigh_num=sum(a_mtr,2)';
    neigh_num(neigh_num==0)=1;
    err_set=[sum(x_rel,2),sum(y_rel,2)]'./neigh_num;
end
