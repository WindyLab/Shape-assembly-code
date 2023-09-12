%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: negotiate formation orientation
function state=Fcn_NegotOrientState(sim_param,neigh_mtr,shape_state,refer_state,inform_index)
    % set proportional gain
    kappa_conse=1.6;
    kappa_track=3.0;
    alpha=0.8;
    % variable substitution
    t=sim_param.t;
    a_mtr=neigh_mtr.a_mtr;
    head_set=shape_state.head_set;
    hvel_set=shape_state.hvel_set;
    ref_head=refer_state.head_set;
    ref_hvel=refer_state.hvel_set;
    swarm_size=sim_param.swarm_size;
    % calculate consensus command for all the robots
    err_set=GetConsensusError(swarm_size,a_mtr,head_set);
    [sign_set,pabs_set]=SepSymbAndValue(err_set);
    cmd_cons=kappa_conse.*sign_set.*(pabs_set.^alpha);
    % calculate reference-tracking command for informed robots
    cmd_track=(kappa_track.*(ref_head-head_set)+ref_hvel);
    % calculate alignment command for uninformed robots
    cmd_align=GetAlignConsensus(swarm_size,a_mtr,hvel_set);
    % get valid set of informed robots
    inform_set=zeros(1,swarm_size);
    inform_set(inform_index)=1;
    % calculate velocity command    
    hvel_max=pi/2;
    cmd_set=-cmd_cons+inform_set.*cmd_track+(~inform_set).*cmd_align;
    cmd_set(cmd_set>hvel_max)=hvel_max; 
    cmd_set(cmd_set<-hvel_max)=-hvel_max;
    % update negotiation movements
    state=shape_state;
    state.head_set=state.head_set+state.hvel_set.*t;
    state.hvel_set=cmd_set;    
end

%% Auxiliary functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: calculate alignment consensus
function cons_set=GetAlignConsensus(swarm_size,a_mtr,state_set)
    h_mtr=kron(state_set,ones(swarm_size,1));
    h_set=sum(h_mtr.*a_mtr,2);
    neigh_num=sum(a_mtr,2)';
    neigh_num(neigh_num==0)=1;
    cons_set=h_set'./neigh_num;
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
    h_mtr=kron(state_set,ones(swarm_size,1));
    h_rel=(h_mtr'-h_mtr).*a_mtr;
    neigh_num=sum(a_mtr,2)';
    neigh_num(neigh_num==0)=1;
    err_set=sum(h_rel,2)'./neigh_num;
end