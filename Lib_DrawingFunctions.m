%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: drawing library
function lib=Lib_DrawingFunctions
    lib.DrawNegotError=@DrawNegotError;
    lib.DrawTrajectory=@DrawTrajectory;
    lib.DrawPerfMetric=@DrawPerfMetric;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: draw performance metric
function DrawPerfMetric(sim_param,record_state)
    % variable substitution
    t=sim_param.t;
    max_step=sim_param.max_step;
    cover_rate=record_state.cover_rate; 
    ent_rate=record_state.ent_rate;
    dist_var=record_state.dist_var; 
    % initialize drawing
    figure(4); box on; hold on; axis equal;
    % set drawing parameters
    line_width=2;
    aspect_ratio=7;
    time_set=(0:1:max_step)*t;
    % draw coverage rate
    subplot(3,1,1); box on; hold on; axis equal; 
    plot(time_set,cover_rate','LineWidth',line_width); 
    min_value=min(cover_rate);
    max_value=max(cover_rate);
    SetDrawLabel('Time (s)',['Coverage rate']);
    SetDrawRange([0,max_step*t,floor(min_value),ceil(max_value)],4,2,aspect_ratio); 
    % draw entering rate
    subplot(3,1,2); box on; hold on; axis equal; 
    plot(time_set,ent_rate','LineWidth',line_width); 
    min_value=min(ent_rate);
    max_value=max(ent_rate);
    SetDrawLabel('Time (s)',['Entering rate']);
    SetDrawRange([0,max_step*t,floor(min_value),ceil(max_value)],4,2,aspect_ratio); 
    % draw uniformity
    subplot(3,1,3); box on; hold on; axis equal; 
    plot(time_set,dist_var','LineWidth',line_width); 
    min_value=min(dist_var);
    max_value=max(dist_var);
    SetDrawLabel('Time (s)',['Uniformity']);
    SetDrawRange([0,max_step*t,floor(min_value),ceil(max_value)],4,2,aspect_ratio); 
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: draw motion trajectory
function DrawTrajectory(sim_param,sim_range,form_dyn,shape_info,record_state)
    % variable substitution
    swarm_size=sim_param.swarm_size;
    max_step=sim_param.max_step;
    ptra_rec=record_state.ptra_rec; 
    pset_rec=record_state.pset_rec; 
    vset_rec=record_state.vset_rec;
    % initialize drawing
    figure(3); box on; hold on; axis equal;
    %set(gcf,'units','normalized','position',[0.05,0.05,0.9,0.85]);
    % set drawing parameters
    scale=0.2;
    clarity=0.2;
    % draw target shape
    DrawTargetShape(form_dyn.size,shape_info.rn,shape_info.cn,shape_info.grid,form_dyn);
    % draw trajectory
    for index=1:1:swarm_size        
        handle=plot(ptra_rec(1,1:max_step,index),ptra_rec(2,1:max_step,index),'LineWidth',1);
        handle.Color(4)=clarity;
    end
    % draw initial states
    pos_set=pset_rec(:,:,1); 
    vel_set=vset_rec(:,:,1); 
    DrawInitialMotion(scale,sim_param.swarm_size,sim_param.r_body,pos_set,vel_set);
    % draw final states
    pos_set=pset_rec(:,:,max_step); 
    vel_set=vset_rec(:,:,max_step); 
    DrawFinalMotion(scale,sim_param.swarm_size,sim_param.r_body,pos_set,vel_set);
    % set prperties
    SetDrawLabel('x axis (m)','y axis (m)');
    SetDrawRange(sim_range,2,2,1); 
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: draw negotiation errors
function DrawNegotError(sim_param,record_state)    
    % variable substitution
    t=sim_param.t;
    max_step=sim_param.max_step;
    swarm_size=sim_param.swarm_size;
    pos_set=record_state.spos_rec;
    head_set=record_state.shead_rec;
    ref_pos=record_state.rpos_rec;
    ref_head=record_state.rhead_rec;    
    % initialize drawing
    figure(2); box on; hold on; axis equal;
    %set(gcf,'units','normalized','position',[0.05,0.05,0.9,0.85]);
    % set drawing parameters
    clarity=1.0;
    line_width=1.0;
    aspect_ratio=7;
    time_set=(0:1:max_step)*t;
    min_set=inf*ones(1,swarm_size);
    max_set=-inf*ones(1,swarm_size);
    % draw x-axis postion error
    subplot(3,1,1); box on; hold on; axis equal;    
    for index=1:1:swarm_size
        rel_value=pos_set(1,:,index)-ref_pos(1,:);
        H=plot(time_set,rel_value,'LineWidth',line_width);
        H.Color(4)=clarity;
        min_set(:,index)=min(rel_value);
        max_set(:,index)=max(rel_value);
    end
    SetDrawLabel('Time (s)',['x-axis position' sprintf('\n' ) 'error (m)']);
    SetDrawRange([0,max_step*t,floor(min(min_set)),ceil(max(max_set))],4,2,aspect_ratio);    
    % draw y-axis postion error
    subplot(3,1,2); box on; hold on; axis equal;
    for index=1:1:swarm_size
        rel_value=pos_set(2,:,index)-ref_pos(2,:);
        H=plot(time_set,rel_value,'LineWidth',line_width);
        H.Color(4)=clarity;
        min_set(:,index)=min(rel_value);
        max_set(:,index)=max(rel_value);
    end
    SetDrawLabel('Time (s)',['y-axis position' sprintf('\n' ) 'error (m)']);
    SetDrawRange([0,max_step*t,floor(min(min_set)),ceil(max(max_set))],4,2,aspect_ratio); 
    % draw orientation error
    subplot(3,1,3); box on; hold on; axis equal;    
    for index=1:1:swarm_size
        rel_value=head_set(:,index)-ref_head(:);
        H=plot(time_set,rel_value,'LineWidth',line_width);
        H.Color(4)=clarity;
        min_set(:,index)=min(rel_value);
        max_set(:,index)=max(rel_value);
    end
    SetDrawLabel('Time (s)',['Orientation' sprintf('\n' ) 'error (rad)']);
    SetDrawRange([0,max_step*t,floor(min(min_set)),ceil(max(max_set))],4,2,aspect_ratio); 
end

%% Auxiliary functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: set drawing range
function SetDrawLabel(x_label,y_label)
    xlabel(x_label);
    ylabel(y_label);
    set(gca,'FontSize',10,'Fontname','Times New Roman');
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: set drawing range
function SetDrawRange(range,x_num,y_num,scale)
    axis(range);    
    set(gca,'XTick',(range(1):(range(2)-range(1))/x_num:range(2)));
    set(gca,'YTick',(range(3):(range(4)-range(3))/y_num:range(4)));
    set(gca,'DataAspectRatio',[1 scale*(range(4)-range(3))/(range(2)-range(1)) 1]);
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: draw robots' movements
function DrawFinalMotion(scale,swarm_size,r_body,pos_set,vel_set)
    edge_color=[0 0.4470 0.7410];
    face_color=[0 0.4470 0.7410]; 
    for index=1:1:swarm_size   
        rect=[pos_set(1,index)-r_body,pos_set(2,index)-r_body,2*r_body,2*r_body];
        rectangle('Position',rect,'Curvature',[1,1],'EdgeColor',edge_color,'FaceColor',face_color); 
        temp_x=[pos_set(1,index),pos_set(1,index)+scale.*vel_set(1,index)];
        temp_y=[pos_set(2,index),pos_set(2,index)+scale.*vel_set(2,index)];
        plot(temp_x,temp_y,'r','linewidth',1.0);
    end
end

function DrawInitialMotion(scale,swarm_size,r_body,pos_set,vel_set)
    edge_color=[160 160 160]./255;
    face_color=[160 160 160]./255; 
    for index=1:1:swarm_size   
        rect=[pos_set(1,index)-r_body,pos_set(2,index)-r_body,2*r_body,2*r_body];
        rectangle('Position',rect,'Curvature',[1,1],'EdgeColor','none','FaceColor',face_color); 
        temp_x=[pos_set(1,index),pos_set(1,index)+scale.*vel_set(1,index)];
        temp_y=[pos_set(2,index),pos_set(2,index)+scale.*vel_set(2,index)];
        plot(temp_x,temp_y,'r','linewidth',1.0);
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: draw formation
function DrawTargetShape(swarm_size,rn,cn,grid,shape_state)
    shape_x=shape_state.shape_x; 
    shape_y=shape_state.shape_y;  
    shape_value=shape_state.shape_value; 
    shape_head=shape_state.shape_head;
    width=grid/2-grid/10;
    color=[0.85 1 0.85];
    for index=1:1:swarm_size
        for i=1:1:rn
            for j=1:1:cn
                if shape_value(i,j)<=0.0%3
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
                    patch(temp_px,temp_py,color,'EdgeColor',color); 
                end
            end
        end        
    end
end
