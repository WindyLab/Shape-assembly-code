%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: calculate shape-entering velocity command
function [cmd_set,grid_set]=Fcn_CalEnteringCmd(sim_param,shape_mtr,shape_info,robot_state,shape_state)
    % set proportional gain
    kappa=10.0;
    % robot-variable substitution
    t=sim_param.t;
    vel_max=sim_param.vel_max;
    swarm_size=sim_param.swarm_size;
    pos_set=robot_state.pos_set;
    % shape-variable substitution
    rn=shape_info.rn; 
    cn=shape_info.cn; 
    grid=shape_info.grid; 
    cen_x=shape_info.cen_x; 
    cen_y=shape_info.cen_y; 
    shape_x=shape_mtr.shape_x; 
    shape_y=shape_mtr.shape_y; 
    shape_value=shape_mtr.shape_value; 
    fpos_set=shape_state.pos_set;
    fvel_set=shape_state.vel_set;
    fhead_set=shape_state.head_set; 
    % calculate local target position
    grid_set=zeros(size(pos_set));
    gpos_set=zeros(size(pos_set));
    gray_set=zeros(1,swarm_size);
    for index=1:1:swarm_size
        % locate robot in the local coordinate system
        location=TransGoalToLocal(pos_set(:,index),fpos_set(:,index),fhead_set(:,index),grid,cen_x,cen_y,rn,cn);
        % get gray color of the above location
        gray_color=GetGrayValue(location,shape_value); 
        % calculate the local target point
        if gray_color==1 % robot is not in the gray grid
            goal_local=GetLocalTargetOut(location,pos_set(:,index),shape_x(:,:,index),shape_y(:,:,index),shape_value,rn);   
        else             % robot is in the gray grid
            delta=vel_max*t;
            range=ceil(delta/grid)+3;
            goal_local=GetLocalTargetIn(location,pos_set(:,index),shape_x(:,:,index),shape_y(:,:,index),shape_value,rn,cn,range);  
        end
        grid_set(:,index)=[location(1),location(2)]';
        gpos_set(:,index)=goal_local;
        gray_set(:,index)=gray_color;
    end
    % calculate shape-entering command
    dist=sqrt((gpos_set(1,:)-pos_set(1,:)).^2+(gpos_set(2,:)-pos_set(2,:)).^2); 
    dist(dist<=0)=1;    
    cmd_set=kappa.*(gpos_set-pos_set).*(gray_set./dist)+fvel_set;
end

%% Auxiliary functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: get local target location when robot is in the gray grid
function goal_set=GetLocalTargetIn(location,pos_set,shape_x,shape_y,shape_value,rn,cn,range)
    % get local matrix for searching
    min_r=max(1,location(1)-range); 
    max_r=min(rn,location(1)+range);
    min_c=max(1,location(2)-range); 
    max_c=min(cn,location(2)+range);
    local_mtr=shape_value(min_r:max_r,min_c:max_c);    
    % find the cell whose gray value is less than current gray value
    cen_r=location(1)-min_r+1; 
    cen_c=location(2)-min_c+1; 
    valid_mtr=zeros(size(local_mtr));
    gray_value=local_mtr(cen_r,cen_c);
    valid_mtr(local_mtr<gray_value)=1;  
    temp_x=shape_x(min_r:max_r,min_c:max_c)-pos_set(1);
    temp_y=shape_y(min_r:max_r,min_c:max_c)-pos_set(2);
    dist_mtr=sqrt(temp_x.^2+temp_y.^2).*valid_mtr;    
    dist_mtr=dist_mtr.*valid_mtr;
    dist_mtr(dist_mtr==0)=Inf;
    [~,index]=min(dist_mtr,[],'all','linear');
    [r,~]=size(dist_mtr);
    c_temp=fix(index/r)+1; 
    r_temp=rem(index,r); 
    if r_temp==0 
        r_temp=r;
    end
    r_index=(r_temp-cen_r)+location(1);
    c_index=(c_temp-cen_c)+location(2);
    goal_x=shape_x(r_index,c_index);
    goal_y=shape_y(r_index,c_index);
    goal_set=[goal_x,goal_y]';
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: get local target location when robot is not in the gray grid
function goal_set=GetLocalTargetOut(location,pos_set,shape_x,shape_y,shape_value,rn)
    valid_mtr=shape_value; 
    valid_mtr(valid_mtr<1)=0;
    temp_x=shape_x-pos_set(1); 
    temp_y=shape_y-pos_set(2); 
    dist_mtr=sqrt(temp_x.^2+temp_y.^2).*(~valid_mtr);
    dist_mtr(dist_mtr==0)=Inf; 
    if location(3)
        dist_mtr(location(1),location(2))=Inf;
    end
    [~,index]=min(dist_mtr,[],'all','linear');
    c_index=fix(index/rn)+1; 
    r_index=rem(index,rn);
    if r_index==0
        r_index=rn;
    end
    goal_x=shape_x(r_index,c_index);
    goal_y=shape_y(r_index,c_index);
    goal_set=[goal_x,goal_y]';
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: get gray color of the given location
function value=GetGrayValue(location,shape_value)
    if location(3)==0
        value=1;
    else
        value=shape_value(location(1),location(2));
    end
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
