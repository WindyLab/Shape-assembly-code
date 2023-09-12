%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: animated simulation functions
function lib=Lib_AnimatFunctions
    global scale; scale=0.2;
    lib.InitAnimation=@InitAnimation;
    lib.UpadateAnimation=@UpadateAnimation;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: initialize animated simulation
function hand_set=InitAnimation(draw_mode,sim_param,sim_range,robot_state,refer_state,shape_state,rcent_state,inform_index,shape_mtr,shape_info)
    % no drawing
    if draw_mode==0
        hand_set=false;
        return;
    end
    % initialize animation
    figure(1);
    box on; hold on; grid on; axis equal;
    set(gcf,'units','normalized','position',[0.05,0.05,0.9,0.85]);
    % draw objects
    switch draw_mode
        case 1 % draw robot swarm
            [hand_set.hr_pos,hand_set.hr_vel]=DrawReferMotion(sim_param.r_body,refer_state.pos_set,refer_state.vel_set);
            [hand_set.ha_pos,hand_set.ha_vel]=DrawRobotMotionA(sim_param.swarm_size,sim_param.r_body,robot_state.pos_set,robot_state.vel_set,inform_index);
        case 2 % draw negotiation
            hand_set.hs_pos=plot(shape_state.pos_set(1,:),shape_state.pos_set(2,:),'+','color',[0.9290 0.6940 0.1250],'markersize',3);
            hand_set.hc_pos=plot(rcent_state.pos_set(1,:),rcent_state.pos_set(2,:),'rs','markersize',8);
            [hand_set.ha_pos,hand_set.ha_vel]=DrawRobotMotionA(sim_param.swarm_size,sim_param.r_body,robot_state.pos_set,robot_state.vel_set,inform_index);
        case 3 % draw static shape
            hand_set.h_shape=DrawTargetShape(shape_mtr.size,shape_info.rn,shape_info.cn,shape_info.grid,shape_mtr);
            [hand_set.ha_pos,hand_set.ha_vel]=DrawRobotMotionB(sim_param.swarm_size,sim_param.r_body,robot_state.pos_set,robot_state.vel_set);
    end
    % set drawing properties
    xlabel('x axis (m)');
    ylabel('y axis (m)');
    set(gca,'FontSize',10,'Fontname','Times New Roman');
    axis(sim_range);
    set(gca,'xtick',sim_range(1):(sim_range(2)-sim_range(1))/4:sim_range(2));
    set(gca,'ytick',sim_range(3):(sim_range(4)-sim_range(3))/4:sim_range(4));  
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: update animated simulation
function hand_set=UpadateAnimation(hand_set,draw_mode,sim_param,robot_state,refer_state,shape_state,rcent_state)
    % no drawing
    if draw_mode==0
        return;
    end
    % update objects
    switch draw_mode
        case 1 % update robot swarm
            UpdateReferMotion(hand_set.hr_pos,hand_set.hr_vel,sim_param.r_body,refer_state.pos_set,refer_state.vel_set);
            UpdateRobotMotion(hand_set.ha_pos,hand_set.ha_vel,sim_param.swarm_size,sim_param.r_body,robot_state.pos_set,robot_state.vel_set);
        case 2 % update negotiation
            set(hand_set.hs_pos,'XData',shape_state.pos_set(1,:),'YData',shape_state.pos_set(2,:));
            set(hand_set.hc_pos,'XData',rcent_state.pos_set(1,:),'YData',rcent_state.pos_set(2,:));
            UpdateRobotMotion(hand_set.ha_pos,hand_set.ha_vel,sim_param.swarm_size,sim_param.r_body,robot_state.pos_set,robot_state.vel_set);
        case 3 % update static shape
            UpdateRobotMotion(hand_set.ha_pos,hand_set.ha_vel,sim_param.swarm_size,sim_param.r_body,robot_state.pos_set,robot_state.vel_set);
    end
end

%% Auxiliary functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: draw robots' movements
function [ha_pos,ha_vel]=DrawRobotMotionA(swarm_size,r_body,pos_set,vel_set,inform_index)
    global scale;
    for index=1:1:swarm_size   
        if ~ismember(index,inform_index)
            color=[0 0.4470 0.7410];
        else
            color=[0.8500 0.3250 0.0980];
        end
        rect=[pos_set(1,index)-r_body,pos_set(2,index)-r_body,2*r_body,2*r_body];
        ha_pos(index)=rectangle('Position',rect,'Curvature',[1,1],'EdgeColor',color); 
        temp_x=[pos_set(1,index),pos_set(1,index)+scale.*vel_set(1,index)];
        temp_y=[pos_set(2,index),pos_set(2,index)+scale.*vel_set(2,index)];
        ha_vel(index)=plot(temp_x,temp_y,'r','linewidth',1.0);
    end
end

function [ha_pos,ha_vel]=DrawRobotMotionB(swarm_size,r_body,pos_set,vel_set)
    global scale;
    for index=1:1:swarm_size   
        color=[0 0.4470 0.7410];
        rect=[pos_set(1,index)-r_body,pos_set(2,index)-r_body,2*r_body,2*r_body];
        ha_pos(index)=rectangle('Position',rect,'Curvature',[1,1],'EdgeColor',color); 
        temp_x=[pos_set(1,index),pos_set(1,index)+scale.*vel_set(1,index)];
        temp_y=[pos_set(2,index),pos_set(2,index)+scale.*vel_set(2,index)];
        ha_vel(index)=plot(temp_x,temp_y,'r','linewidth',1.0);
    end
end

% FUNCTION: update robots' movements
function UpdateRobotMotion(ha_pos,ha_vel,swarm_size,r_body,pos_set,vel_set)
    global scale;
    for index=1:1:swarm_size
        rect=[pos_set(1,index)-r_body,pos_set(2,index)-r_body,2*r_body,2*r_body];
        set(ha_pos(index),'Position',rect);
        temp_x=[pos_set(1,index),pos_set(1,index)+scale.*vel_set(1,index)];
        temp_y=[pos_set(2,index),pos_set(2,index)+scale.*vel_set(2,index)];
        set(ha_vel(index),'XData',temp_x,'YData',temp_y);
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: draw reference movement
function [hr_pos,hr_vel]=DrawReferMotion(r_body,pos_set,vel_set)
    global scale;
    rect=[pos_set(1,:)-r_body,pos_set(2,:)-r_body,2*r_body,2*r_body];
    hr_pos=rectangle('Position',rect,'Curvature',[1,1],'EdgeColor','c','FaceColor','c'); 
    temp_x=[pos_set(1,:),pos_set(1,:)+scale.*vel_set(1,:)];
    temp_y=[pos_set(2,:),pos_set(2,:)+scale.*vel_set(2,:)];
    hr_vel=plot(temp_x,temp_y,'r','linewidth',1.0);
end

function UpdateReferMotion(hr_pos,hr_vel,r_body,pos_set,vel_set)
    global scale;
    rect=[pos_set(1,:)-r_body,pos_set(2,:)-r_body,2*r_body,2*r_body];
    set(hr_pos,'Position',rect);
    temp_x=[pos_set(1,:),pos_set(1,:)+scale.*vel_set(1,:)];
    temp_y=[pos_set(2,:),pos_set(2,:)+scale.*vel_set(2,:)];
    set(hr_vel,'XData',temp_x,'YData',temp_y);
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: draw formation
function h_shape=DrawTargetShape(swarm_size,rn,cn,grid,shape_state)
    shape_x=shape_state.shape_x; 
    shape_y=shape_state.shape_y;  
    shape_value=shape_state.shape_value; 
    shape_head=shape_state.shape_head;
    width=grid/2-grid/10;
    conut=1;
    for index=1:1:swarm_size
        color=rand(1,3);
        for i=1:1:rn
            for j=1:1:cn
                if shape_value(i,j)==0
                    temp_x=[shape_x(i,j,index)-width,...
                            shape_x(i,j,index)+width,...
                            shape_x(i,j,index)+width,...
                            shape_x(i,j,index)-width,...
                            shape_x(i,j,index)-width];
                    temp_y=[shape_y(i,j,index)-width,...
                            shape_y(i,j,index)-width,...
                            shape_y(i,j,index)+width,...
                            shape_y(i,j,index)+width,...
                            shape_y(i,j,index)-width];
                    temp_px=(temp_x-shape_x(i,j,index)).*cos(pi*shape_head/180)-(temp_y-shape_y(i,j,index)).*sin(pi*shape_head/180)+shape_x(i,j,index);
                    temp_py=(temp_x-shape_x(i,j,index)).*sin(pi*shape_head/180)+(temp_y-shape_y(i,j,index)).*cos(pi*shape_head/180)+shape_y(i,j,index);  
                    %h_shape(conut)=plot(temp_px,temp_py,'color',color); 
                    h_shape(conut)=plot(temp_px,temp_py,'color',[185 255 185]./255); 
                    conut=conut+1;
                    %plot(shape_mtr(i,j,1),shape_mtr(i,j,2),'*','color',[0 0 255]./255);
                end
            end
        end        
    end
end