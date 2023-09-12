%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: calculate shape-exploration velocity command
function cmd_set=Fcn_CalExplorationCmd(sim_param,shape_mtr,shape_info,grid_set,neigh_mtr,robot_state,shape_state)
    % set proportional gain
    kappa_1=5.0; 
    kappa_2=15.0; 
    % robot variable substitution
    a_mtr=neigh_mtr.a_mtr;
    swarm_size=sim_param.swarm_size;
    r_body=sim_param.r_body; 
    r_avoid=sim_param.r_avoid; 
    r_sense=sim_param.r_sense;    
    pos_set=robot_state.pos_set;
    % shape-variable substitution
    rn=shape_info.rn; 
    cn=shape_info.cn; 
    cen_x=shape_info.cen_x; 
    cen_y=shape_info.cen_y; 
    grid=shape_info.grid; 
    shape_x=shape_mtr.shape_x; 
    shape_y=shape_mtr.shape_y; 
    shape_value=shape_mtr.shape_value;  
    fpos_set=shape_state.pos_set;
    fhead_set=shape_state.head_set; 
    % calculate mean point  
    gfpos_set=zeros(size(pos_set));
    gepos_set=zeros(size(pos_set));
    robot_range=max(1,floor(max(r_avoid/2,r_body*2)/grid));
    neigh_range=ceil(r_sense/grid);
    for index=1:1:swarm_size
        % get mean point for filling action when there are both black and non-black cells around robot
        gfpos_set(:,index)=GetMeanPointFill(grid_set(:,index),pos_set(:,index),shape_x(:,:,index),shape_y(:,:,index),...
            shape_value,rn,cn,grid,neigh_range);
        % get mean point for exploring action when all the cells around robot are black 
        gepos_set(:,index)=GetMeanPointExpl(index,grid_set(:,index),pos_set,shape_x(:,:,index),shape_y(:,:,index),...
            shape_value,rn,cn,cen_x,cen_y,grid,neigh_range,robot_range,a_mtr(index,:),fpos_set(:,index),fhead_set(:,index));
    end
    % calculate attractive force
    cmd_set=kappa_1.*(gfpos_set-pos_set)+kappa_2.*(gepos_set-pos_set);
end

%% Auxiliary functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: get mean point for exploring action
function goal_set=GetMeanPointExpl(index,grid_set,pos_set,shape_x,shape_y,shape_value,rn,cn,cen_x,cen_y,grid,neigh_range,robot_range,a_mtr,fpos_set,fhead_set)
    % get neighboring position
    pos_curr=pos_set(:,index);
    pos_neigh=pos_set(:,a_mtr==1);
    % initialize valid matrix
    valid_mtr=shape_value; 
    valid_mtr(valid_mtr>0)=1;
    % determine the occupied cells by neigbors
    neigh_num=sum(a_mtr,2);
    for i=1:1:neigh_num
        % locate agent in the local coordinate system
        location=TransGoalToLocal(pos_neigh(:,i),fpos_set,fhead_set,grid,cen_x,cen_y,rn,cn);
        min_r=max(1,location(1)-robot_range); 
        max_r=min(rn,location(1)+robot_range);
        min_c=max(1,location(2)-robot_range); 
        max_c=min(cn,location(2)+robot_range);
        valid_mtr(min_r:max_r,min_c:max_c)=ones(max_r-min_r+1,max_c-min_c+1);
    end
    % get local matrix for searching
    min_r=max(1,grid_set(1)-neigh_range); 
    max_r=min(rn,grid_set(1)+neigh_range);
    min_c=max(1,grid_set(2)-neigh_range); 
    max_c=min(cn,grid_set(2)+neigh_range);
    local_mtr=valid_mtr(min_r:max_r,min_c:max_c);  
    % get local matrix for searching
    [r_temp,c_temp]=find(local_mtr==0);
    r_index=r_temp+min_r-1;
    c_index=c_temp+min_c-1;
    gx_set=shape_x(sub2ind(size(shape_x),r_index',c_index'));
    gy_set=shape_y(sub2ind(size(shape_y),r_index',c_index'));    
    temp_x=gx_set-pos_curr(1);
    temp_y=gy_set-pos_curr(2);    
    dist_set=sqrt(temp_x.^2+temp_y.^2);
    weight=CalWeightFunction(dist_set,grid*neigh_range,0);
    temp=sum(weight,2);
    if temp==0
        goal_set=pos_curr;
        return
    end
    goal_x=sum(gx_set.*weight,2)/temp;
    goal_y=sum(gy_set.*weight,2)/temp;
    goal_set=[goal_x,goal_y]';
    if size(goal_set)==0
        goal_set=pos_set(:,index);
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: get mean point for filling action
function goal_set=GetMeanPointFill(grid_set,pos_set,shape_x,shape_y,shape_value,rn,cn,grid,neigh_range)
    % get local matrix for searching
    min_r=max(1,grid_set(1)-neigh_range); 
    max_r=min(rn,grid_set(1)+neigh_range);
    min_c=max(1,grid_set(2)-neigh_range); 
    max_c=min(cn,grid_set(2)+neigh_range);
    local_mtr=shape_value(min_r:max_r,min_c:max_c); 
    local_mtr(local_mtr>0)=1; 
    % calculate local grid in local matrix
    cen_r=grid_set(1)-min_r+1; 
    cen_c=grid_set(2)-min_c+1; 
    % get local matrix for searching
    [r_temp,c_temp]=find(local_mtr==0);
    r_index=r_temp-cen_r+grid_set(1);
    c_index=c_temp-cen_c+grid_set(2);
    gx_set=shape_x(sub2ind(size(shape_x),r_index',c_index'));
    gy_set=shape_y(sub2ind(size(shape_y),r_index',c_index'));
    temp_x=gx_set-pos_set(1);
    temp_y=gy_set-pos_set(2);    
    dist_set=sqrt(temp_x.^2+temp_y.^2);
    weight=CalWeightFunction(dist_set,grid*neigh_range,0);
    temp=sum(weight,2);
    if temp==0
        goal_set=pos_set;
        return
    end
    goal_x=sum(gx_set.*weight,2)/temp;
    goal_y=sum(gy_set.*weight,2)/temp;
    goal_set=[goal_x,goal_y]';
    if size(goal_set)==0
        goal_set=pos_set;
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: weight function 
function y=CalWeightFunction(x,r,s)
    y=(1+cos(pi.*(x-2*s)./(r-2*s)))/2;
    y(x<2*s)=1;
    y(x>r)=0;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: transform global position to local locations
function location=TransGoalToLocal(pos_set,fpos_set,fhead_set,grid,cen_x,cen_y,rn,cn)
    % calculate the azimuth angle of robot to the formation center
    head_set=atan2(pos_set(2,:)-fpos_set(2),pos_set(1,:)-fpos_set(1));
    azim_set=head_set-fhead_set; 
    azim_set=LimitAngle(azim_set);
    % calculate the distance between robot and formation center
    x_temp=(pos_set(1,:)-fpos_set(1)).^2; 
    y_temp=(pos_set(2,:)-fpos_set(2)).^2;    
    dist=sqrt(x_temp+y_temp);
    % get relative position of robot to the formation center
    x_coor=round(dist.*cos(azim_set)./grid);
    y_coor=round(dist.*sin(azim_set)./grid);
    % calculate the position of robot in the formation matrix
    x_grid=x_coor+cen_x;
    y_grid=rn-(y_coor+cen_y)+1;
    % determine whether the robot is in the formation matrix
    inside=zeros(1,length(x_grid));
    index_x=find(x_grid>=1&x_grid<=cn);
    index_y=find(y_grid>=1&y_grid<=rn);
    inside(intersect(index_x,index_y))=1;
    location=[y_grid;x_grid;inside];    
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: limit angle
function angle_s=LimitAngle(angle)
    angle(angle>pi)=angle(angle>pi)-2*pi;
    angle(angle<-pi)=angle(angle<-pi)+2*pi;
    angle_s=angle;
end
