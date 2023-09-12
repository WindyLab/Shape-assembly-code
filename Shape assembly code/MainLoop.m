close all; clear; clc;

%% Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AUTHORS: Guibin Sun, Beihang Universuty
% INSTRUCTOR: Shiyu Zhao, Westlake University
% TIME: 2020.9-2021.7
% DECLARATION: Copyright @ Authors. All Rights Reserved.

%% Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% declare function library
Init_lib=Lib_InitialFunctions; % use for simulation initialization
Shape_lib=Lib_ShapeFunctions;  % use for shape processing
Updt_lib=Lib_UpdateFunctions;  % use to update robot motion
Anim_lib=Lib_AnimatFunctions;  % use for animated simulation
Draw_lib=Lib_DrawingFunctions; % use to draw curves 
Recd_lib=Lib_RecordFunctions;  % use to record data

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% set simulation params
Run_time=20;                   % run time of simulation
Swarm_size=50;                 % size of robot swarm
Sim_range=[0,30,0,30];         % simulation range
Sim_param=Init_lib.InitSimuParam(Run_time,Swarm_size);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% initialize robots' states
Pos_init=[Sim_range(1)+(Sim_range(2)-Sim_range(1))/2,Sim_range(3)+(Sim_range(4)-Sim_range(3))/2]'; 
Robot_state=Init_lib.InitRobotState(Sim_param,Pos_init);
% initialize reference states
Refer_state=Init_lib.InitReferState(Pos_init);
% randomly select a given number of agents as informed robots
Inform_index=Init_lib.SelectInformer(Sim_param);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% initialize formation shape
Type=1;  % shape type: 1-snowflake, 2-starfish, 3-letter R, 4-letter O, 5-letter B
Gray_image=Shape_lib.LoadShapeImage(Type);
[Gray_mtr,Gray_info]=Shape_lib.InitFormShape(Sim_param,Gray_image);
% initialize formation states
Shape_state=Shape_lib.InitFormState(Sim_param,Robot_state);
% initialize shape center 
Rcent_state=Shape_lib.GetShapeCenter(Sim_param.swarm_size,Shape_state);
% initialize performance metric
Neigh_mtr=Fcn_GetNeighborSet(Sim_param,Robot_state); 
Metric=Fcn_GetPerformMetric(Sim_param,Neigh_mtr,Robot_state,Rcent_state,Gray_mtr,Gray_info);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% initialize record states
Record_state=Recd_lib.InitRecordState(Sim_param,Robot_state,Refer_state,Shape_state,Rcent_state,Metric);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% initialize animated simulation
Anim_mode=3; % 0-no drawing, 1-robot swarm, 2-negotiation, 3-static shape
form_dyn=Shape_lib.GetDynFormation(1,Gray_mtr,Gray_info,Rcent_state);
Hand_set=Anim_lib.InitAnimation(Anim_mode,Sim_param,Sim_range,Robot_state,Refer_state,Shape_state,Rcent_state,Inform_index,form_dyn,Gray_info);
% set curves flags 
Draw_mode=3; % 0-no curve, 1-negotiation error, 2-motion trajectory, 3-performance metric

%% Control Process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for step=1:1:Sim_param.max_step
    if Anim_mode==0&&rem(step,25)==0
        disp(['The simulation has been run ',num2str(step),' steps!']);
    end
    
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % get neighboring set
    Neigh_mtr=Fcn_GetNeighborSet(Sim_param,Robot_state); 
    
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % negotiate formation states
    state=Fcn_NegotPositState(Sim_param,Neigh_mtr,Shape_state,Refer_state,Inform_index);
    Shape_state=Fcn_NegotOrientState(Sim_param,Neigh_mtr,state,Refer_state,Inform_index);
    % get target formation
    Form_dyn=Shape_lib.GetDynFormation(Sim_param.swarm_size,Gray_mtr,Gray_info,Shape_state);
    
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % calculate shape-entering command
    [cmd_enter,grid_set]=Fcn_CalEnteringCmd(Sim_param,Form_dyn,Gray_info,Robot_state,Shape_state);
    % calculate shape-exploration command 
    cmd_explore=Fcn_CalExplorationCmd(Sim_param,Form_dyn,Gray_info,grid_set,Neigh_mtr,Robot_state,Shape_state);
    % calculate interaction velocity command
    cmd_interact=Fcn_CalInteractionCmd(Sim_param,Neigh_mtr,Robot_state);
    % calculate resultant command
    cmd_set=cmd_enter+cmd_explore+cmd_interact;

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % update robot motion
    state=Updt_lib.UpdateRobotMotion(Sim_param,Robot_state,cmd_set);
    Robot_state=state;
    % update shape center 
    Rcent_state=Shape_lib.GetShapeCenter(Sim_param.swarm_size,Shape_state);
    % update performance metric
    Metric=Fcn_GetPerformMetric(Sim_param,Neigh_mtr,Robot_state,Rcent_state,Gray_mtr,Gray_info);
    % update record state
    state=Recd_lib.UpdateRecordState(step,Record_state,Robot_state,Refer_state,Shape_state,Rcent_state,Metric);
    Record_state=state;
    % update animated simulation    
    Anim_lib.UpadateAnimation(Hand_set,Anim_mode,Sim_param,Robot_state,Refer_state,Shape_state,Rcent_state);    
    drawnow;
end

%% Curves Drawing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch Draw_mode
    case 1
        Draw_lib.DrawNegotError(Sim_param,Record_state);
    case 2
        form_dyn=Shape_lib.GetDynFormation(1,Gray_mtr,Gray_info,Rcent_state);
        Draw_lib.DrawTrajectory(Sim_param,Sim_range,form_dyn,Gray_info,Record_state);
    case 3
        Draw_lib.DrawPerfMetric(Sim_param,Record_state);
end
