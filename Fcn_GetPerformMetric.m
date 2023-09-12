%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: get performance metric
function metric=Fcn_GetPerformMetric(sim_param,neigh_mtr,robot_state,rcent_state,gray_mtr,gray_info)
    % variable substitution
    rn=gray_info.rn; 
    cn=gray_info.cn; 
    cen_x=gray_info.cen_x;
    cen_y=gray_info.cen_y;
    grid_size=gray_info.grid; 
    gray_scale=gray_info.gray_scale;
    r_sense=sim_param.r_sense;
    swarm_size=sim_param.swarm_size;
    a_mtr=neigh_mtr.a_mtr;
    d_mtr=neigh_mtr.d_mtr;
    shape_value=gray_mtr(:,:,3);
    pos_set=robot_state.pos_set;
    vel_set=robot_state.vel_set;
    fpos_set=rcent_state.pos_set;
    fhead_set=rcent_state.head_set;
    % calculate the number of cells that have been covered
    num_ent=0;
    shape_value(shape_value>0)=1;
    cell_num=length(find(shape_value==0));
    robot_range=floor(sim_param.r_avoid/grid_size);
    for i=1:1:swarm_size
        % locate robot in the local coordinate system
        location=TransGoalToLocal(pos_set(:,i),fpos_set,fhead_set,grid_size,cen_x,cen_y,rn,cn);
        % get the distribution of robots in shape
        if location(3)            
            min_r=max(1,location(1)-robot_range); 
            max_r=min(rn,location(1)+robot_range);
            min_c=max(1,location(2)-robot_range); 
            max_c=min(cn,location(2)+robot_range);
            shape_value(min_r:max_r,min_c:max_c)=ones(max_r-min_r+1,max_c-min_c+1);                
        end 
        % get gray level of this location
        shape_value_temp=gray_mtr(:,:,3);
        gray_value=GetGrayValue(location,shape_value_temp); 
        % update the number of the entered robots
        if abs(gray_value)<=1.0/gray_scale
            num_ent=num_ent+1;
        else
            num_ent=num_ent+0;
        end
    end
     % calculate coverage rate M1
    void_num=length(find(shape_value==0));    
    metric.cover_rate=(cell_num-void_num)/cell_num;
    % calculate entering rate M2
    metric.ent_rate=num_ent./swarm_size;
    % calculate distance variance M3
    ds_mtr=d_mtr.*a_mtr+r_sense.*(~a_mtr);
    min_set=min(ds_mtr');
    mean=sum(min_set,2)./swarm_size;
    derr_set=(min_set-mean).^2;
    metric.dist_var=sum(derr_set,2);
    % calculate mean velocity M4
    temp=sqrt(vel_set(1,:).^2+vel_set(2,:).^2); 
    vel_norm=sum(temp,2); 
    vel_norm(vel_norm==0)=1;
    temp=sum(vel_set,2); 
    vel_sum=sqrt(temp'*temp);    
    metric.mean_vel=vel_sum./vel_norm;
end

%% Auxiliary functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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