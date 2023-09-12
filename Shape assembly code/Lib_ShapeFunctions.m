%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION: shape processing library
function lib=Lib_ShapeFunctions
    lib.LoadShapeImage=@LoadShapeImage;
    lib.InitFormShape=@InitFormShape;
    lib.InitFormState=@InitFormState;
    lib.GetShapeCenter=@GetShapeCenter;
    lib.GetDynFormation=@GetDynFormation;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: get dynamic formation
function shape_dyn=GetDynFormation(swarm_size,shape_mtr,shape_info,shape_state)
    % variable substitution
    rn=shape_info.rn; 
    cn=shape_info.cn; 
    head_set=shape_state.head_set; 
    pos_set=shape_state.pos_set;
    % initialize output
    shape_x=zeros(rn,cn,swarm_size); 
    shape_y=zeros(rn,cn,swarm_size);    
    shape_value=shape_mtr(:,:,3);
    % transform coordinate
    for i=1:1:swarm_size
        temp_mtr(:,:,1)=shape_mtr(:,:,1).*cos(head_set(i))-shape_mtr(:,:,2).*sin(head_set(i));
        temp_mtr(:,:,2)=shape_mtr(:,:,1).*sin(head_set(i))+shape_mtr(:,:,2).*cos(head_set(i));    
        shape_x(:,:,i)=temp_mtr(:,:,1)+pos_set(1,i);
        shape_y(:,:,i)=temp_mtr(:,:,2)+pos_set(2,i);
    end
    % assign output
    shape_dyn.shape_x=shape_x;
    shape_dyn.shape_y=shape_y;
    shape_dyn.shape_value=shape_value;
    shape_dyn.shape_head=head_set;
    shape_dyn.size=swarm_size;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: get shape center 
function state=GetShapeCenter(swarm_size,shape_state)
    state.pos_set=sum(shape_state.pos_set,2)./swarm_size;
    state.head_set=sum(shape_state.head_set,2)./swarm_size;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: initialize formation states
function state=InitFormState(sim_param,robot_state)
    state.pos_set=robot_state.pos_set;
    state.vel_set=robot_state.vel_set;
    temp_head=rand(1,sim_param.swarm_size)*2*pi;
    aveg_head=sum(temp_head)/sim_param.swarm_size; 
    aveg_head(aveg_head>2*pi)=aveg_head-2*pi;
    state.head_set=temp_head-aveg_head;
    state.hvel_set=zeros(size(state.head_set));
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: initialize formation shape
function [shape_mtr,shape_info]=InitFormShape(sim_param,image_mtr)
    % variable substitution
    swarm_size=sim_param.swarm_size; 
    r_avoid=sim_param.r_avoid;
    % get parameters
    [shape_info.rn,shape_info.cn]=size(image_mtr);       
    shape_info.cen_x=ceil(shape_info.cn/2);
    shape_info.cen_y=ceil(shape_info.rn/2);
    shape_info.black_num=length(find(image_mtr==0));
    shape_info.gray_num=length(find(image_mtr<1))-shape_info.black_num;
    temp_mtr=image_mtr(shape_info.cen_y,:); 
    temp_mtr(temp_mtr==0)=2;
    shape_info.gray_scale=1/min(temp_mtr);
    % calculate grid size
    ratio=1.0;  
    shape_info.grid=sqrt((pi/4)*(swarm_size/shape_info.black_num))*r_avoid;
    shape_info.grid=shape_info.grid*ratio;
    % get grid coordinates
    shape_mtr=ones(shape_info.rn,shape_info.cn,3);
    temp_x=((1:1:shape_info.cn)-shape_info.cen_x).*shape_info.grid;
    temp_y=((1:1:shape_info.rn)-shape_info.cen_y).*shape_info.grid;
    shape_mtr(:,:,1)=shape_mtr(:,:,1).*temp_x;
    shape_mtr(:,:,2)=shape_mtr(:,:,2).*flipud(temp_y');
    shape_mtr(:,:,3)=image_mtr;  
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% FUNCTION: load shape image
function image=LoadShapeImage(type)
    root_str='.\ShapeImage\';
    file_str='Image_';
    switch type
        case 1
            type_str='geom_snow'; 
        case 2
            type_str='geom_starfish'; 
        case 3
            type_str='letter_R';
        case 4
            type_str='letter_O';
        case 5
            type_str='letter_B';
        otherwise 
            type_str='geom_snow';
    end
    % generate file path    
    path=[root_str,file_str,type_str];
    temp=load(path);
    image=temp.gray_mtr;
end
