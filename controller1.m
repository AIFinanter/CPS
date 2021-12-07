function [ out, state ] = controller1( in, state )
% Takes flight parameters of an aircraft and outputs the direction control


% in: Data Structure that stores input information for the aircraft
% controller. 
%       (in.x, in.y): Current Location of the aircraft
%       (in.xd, in.yd): Destination of aircraft
%       in.theta: Current direction of motion
%       in.m1: Message from the smaller index neighbouring aircraft 
%           - empty if aircraft not in neighbourhood
%           - (x, y, xd, yd, theta) of other aircraft if non-empty
%           - To access data (say x) from in.m, use in.m.x
%       in.m2: Message from the bigger index neighboring aircraft
%           - empty if aircraft not in neighborhood
%           - (x, y, xd, yd, theta) of other aircraft if non-empty
%           - To access data (say x) from in.m2, use in.m2.x
% out : Data Structure that stores the output information from the aircraft
%       out.val: +1, 0, -1 ( +1 - turn left, 0 - go straight, -1 - turn right)
% 
% state: 
%       any state used by the controller


% Initialize state
out.val = 0;

if(isempty(in.m1) && isempty(in.m2))
    if(in.yd>in.y)
        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
            out.val = 1;
        elseif(in.theta == 90 || in.theta == -270)
            out.val = 0;
        elseif(in.theta == 180 || in.theta == -180)
            out.val = -1;
        else
            out.val = 1;
        end    
    elseif(in.yd<in.y)
        if(in.theta==0 || in.theta == 360 || in.theta == -360)
            out.val = -1;
        elseif(in.theta == 90 || in.theta == -270)
            out.val = 1;
        elseif(in.theta == 180 || in.theta == -180)
            out.val = 1;
        else
            out.val = 0;
        end
    else
        if(in.xd>in.x)
            if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                out.val = 0;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = -1;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = 1;
            else
                out.val = 1;
            end
        elseif(in.xd<in.x)
            if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                out.val = 1;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = 1;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = 0;
            else
                out.val = -1;
            end
        end
    end  
elseif((isempty(in.m1)&&~isempty(in.m2))||(~isempty(in.m1)&&isempty(in.m2)))
    tmp = [];
    if(~isempty(in.m1))
        tmp = in.m1;
    elseif(~isempty(in.m2))
        tmp = in.m2;
    end 
    %划分优先级的办法：把离终点近的节点的优先级提高，把离终点远的节点的优先级降低
    if(abs(in.x-in.xd)+abs(in.y-in.yd)<abs(tmp.x-tmp.xd)+abs(tmp.y-tmp.yd))
        if(in.yd>in.y)
            if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                out.val = 1;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = 0;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = -1;
            else
                out.val = 1;
            end    
        elseif(in.yd<in.y)
            if(in.theta==0 || in.theta == 360 || in.theta == -360)
                out.val = -1;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = 1;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = 1;
            else
                out.val = 0;
            end
        else
            if(in.xd>in.x)
                if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = 0;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = -1;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = 1;
                else
                    out.val = 1;
                end
            else
                if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = 1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 1;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = 0;
                else
                    out.val = -1;
                end
            end
        end
    elseif(abs(in.x-in.xd)+abs(in.y-in.yd)>abs(tmp.x-tmp.xd)+abs(tmp.y-tmp.yd))

            
            
        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
            if(tmp.y>in.y)
                out.val = -1;
            else
                out.val = 1;
            end
        elseif(in.theta == 90 || in.theta == -270)
            if(tmp.x<in.x)
                out.val = -1;
            else
                out.val = 1;
            end
        elseif(in.theta == 180 || in.theta == -180)
            if(tmp.y<in.y)
                out.val = -1;
            else
                out.val = 1;
            end
        else
            if(tmp.x>in.x)
                out.val = -1;
            else
                out.val = 1;
            end
        end
        
    else
        if(in.x*10+in.y<tmp.x*10+tmp.y)
            if(in.yd>in.y)
                if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = 1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 0;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = -1;
                else
                    out.val = 1;
                end    
            elseif(in.yd<in.y)
                if(in.theta==0 || in.theta == 360 || in.theta == -360)
                    out.val = -1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 1;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = 1;
                else
                    out.val = 0;
                end
            else
                if(in.xd>in.x)
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 0;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = -1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 1;
                    else
                        out.val = 1;
                    end
                else
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 0;
                    else
                        out.val = -1;
                    end
                end
            end    
        elseif(in.x*10+in.y>tmp.x*10+tmp.y)
            if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                out.val = 0;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = -1;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = -1;
            else
                out.val = 1;
            end
        
        else
            if(in.yd>in.y)
                if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = 1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 0;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = -1;
                else
                    out.val = 1;
                end    
            elseif(in.yd<in.y)
                if(in.theta==0 || in.theta == 360 || in.theta == -360)
                    out.val = -1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 1;
                elseif(in.theta == 180 || in.theta == -180)gp
                    out.val = 1;
                else
                    out.val = 0;
                end
            else
                if(in.xd>in.x)
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 0;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = -1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 1;
                    else
                        out.val = 1;
                    end
                else
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 0;
                    else
                        out.val = -1;
                    end
                end
            end
        end
    end
    
elseif(~isempty(in.m1) && ~isempty(in.m2))
    %这个节点应该往左上飞行以避免碰撞
    if(in.m1.x>in.x && in.m2.x > in.x)
        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
            out.val = 1;
        elseif(in.theta == 90 || in.theta == -270)
            out.val = 1;
        elseif(in.theta == 180 || in.theta == -180)
            out.val = 0;
        else
            out.val = -1;
        end 
        
    elseif(in.m1.x<in.x && in.m2.x<in.x)
        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
            out.val = 0;
        elseif(in.theta == 90 || in.theta == -270)
            out.val = -1;
        elseif(in.theta == 180 || in.theta == -180)
            out.val = -1;
        else
            out.val = 1;
        end    
    elseif(in.m1.x>in.x && in.x>in.m2.x)
        %上下动
        if(abs(in.m1.y-in.y)+abs(in.m1.x-in.x)<=2 || abs(in.m2.y - in.y)+abs(in.m2.x-in.x)<=2)
            if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                in.theta = -1;
            elseif(in.theta == 90 || in.theta == -270)
                in.theta = 0;
            elseif(in.theta == 180 || in.theta == -180)
                in.theta = -1;
            else
                in.theta = 0;
            end    
        else
            if(in.yd>in.y)
                if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = 1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 0;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = -1;
                else
                    out.val = 1;
                end    
            elseif(in.yd<in.y)
                if(in.theta==0 || in.theta == 360 || in.theta == -360)
                    out.val = -1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 1;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = 1;
                else
                    out.val = 0;
                end
            else
                if(in.xd>in.x)
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 0;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = -1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 1;
                    else
                        out.val = 1;
                    end
                else
                    if(in.theta == 0 && in.theta == 360 || in.theta == -360)
                        out.val = 1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 0;
                    else
                        out.val = -1;
                    end
                end
            end
        end     
%    elseif((in.m1.x - in.x == -1 && in.m2.x - in.x == -2) || ((in.m2.x - in.x) == -2 && (in.m1.x-in.x) == -1))
%        if(in.theta == 0 || in.theta == -360 || in.theta == 360)
%            out.val = 0;
%        elseif(in.theta == 90 || in.theta == -270)
%            out.val = -1;
%        elseif(in.theta == 180 || in.theta == -180)
%            out.val = 1;
%        else
%            out.val = 1;
%        end
    %三个点的横坐标相同    
    elseif(in.m1.x == in.x && in.m2.x == in.x)
        %最上面的节点往右上方走
        if(in.m1.y>in.y && in.m2.y>in.y)
            if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                out.val = 0;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = -1;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = -1;
            else
                out.val = 1;
            end
        %最下面的节点往左下方走    
        elseif(in.y<in.m1.y && in.y < in.m2.y)
            if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                out.val = -1;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = 1;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = 0;
            else
                out.val = -1;
            end
        %其他两个节点全部让开了道路，中间的节点可以直接向终点移动
        else
            if(in.yd>in.y)
                if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = 1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 0;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = -1;
                else
                    out.val = 1;
                end    
            elseif(in.yd<in.y)
                if(in.theta==0 || in.theta == 360 || in.theta == -360)
                    out.val = -1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 1;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = 1;
                else
                    out.val = 0;
                end
            else
                if(in.xd>in.x)
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 0;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = -1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 1;
                    else
                        out.val = 1;
                    end
                else
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 0;
                    else
                        out.val = -1;
                    end
                end
            end
        end
    elseif((in.x == in.m1.x && in.x ~=in.m2.x) || (in.x == in.m2.x && in.x ~=in.m1.x))
        %in.m1表示的是同一列的节点，in.m2表示的是和此节点不在同一列的节点
        if(in.m1.x == in.x)
            tmp1 = in.m1;
            tmp2 = in.m2;
        else
            tmp1 = in.m2;
            tmp2 = in.m1;
        end    
        %不在同一列的节点在这两个节点组成的列的右边
        if(tmp2.x>in.x)
            
            %这个节点在另外一个与它同列的节点的上边
            if(in.y > tmp1.y)
                %向左上方走（向左比向上首先考虑）
                if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = 1;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = 1;
                elseif(in.theta == 180 || in.theta == -180)                     
                    out.val = 0;
                else
                    out.val = -1;
                end
            else
                %这个节点在另外一个与它同列的节点的下边
                
                if((tmp2.x-in.x==1 && tmp2.y-in.y==1) || (tmp2.x - in.x == 1 && tmp2.y - in.y == -1))
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = -1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 0;
                    else
                        out.val = -1;
                    end
                %正常走
                elseif(in.yd>in.y)
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 0;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = -1;
                    else
                        out.val = 1;
                    end    
                 elseif(in.yd<in.y)
                    if(in.theta==0 || in.theta == 360 || in.theta == -360)
                        out.val = -1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 1;
                    else
                        out.val = 0;
                    end
                else
                    if(in.xd>in.x)
                        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                            out.val = 0;
                        elseif(in.theta == 90 || in.theta == -270)
                            out.val = -1;
                        elseif(in.theta == 180 || in.theta == -180)
                            out.val = 1;
                        else
                            out.val = 1;
                        end
                    else
                        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                            out.val = 1;
                        elseif(in.theta == 90 || in.theta == -270)
                            out.val = 1;
                        elseif(in.theta == 180 || in.theta == -180)
                            out.val = 0;
                        else
                            out.val = -1;
                        end
                    end
                end
            end    
                    
           
            
        elseif(tmp2.x<in.x)
            if(in.y > tmp1.y)
                if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = 0;
                elseif(in.theta == 90 || in.theta == -270)
                    out.val = -1;
                elseif(in.theta == 180 || in.theta == -180)
                    out.val = -1;
                else
                    out.val = 1;
                end
            else
                %如果这个被赋予最高通行权的节点发现另一个节点在两个对角处,向上下方走
                if((tmp2.x - in.x == -1 && tmp2.y-in.y == -1) || (tmp2.x - in.x == -1 && tmp2.y - in.y == 1) || (tmp2.x - in.x == -2 && tmp2.y - in.y == 0))
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 0;
                    elseif(in.theta == 180 || in.theta == -180)    
                        out.val = -1;
                    else
                        out.val = 0;
                    end    
                elseif(in.yd>in.y)
                    if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                        out.val = 1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 0;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = -1;
                    else
                        out.val = 1;
                    end    
                elseif(in.yd<in.y)
                    if(in.theta==0 || in.theta == 360 || in.theta == -360)
                        out.val = -1;
                    elseif(in.theta == 90 || in.theta == -270)
                        out.val = 1;
                    elseif(in.theta == 180 || in.theta == -180)
                        out.val = 1;
                    else
                        out.val = 0;
                    end
                else
                    if(in.xd>in.x)
                        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                            out.val = 0;
                        elseif(in.theta == 90 || in.theta == -270)
                            out.val = -1;
                        elseif(in.theta == 180 || in.theta == -180)
                            out.val = 1;
                        else
                            out.val = 1;
                        end
                    else
                        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                            out.val = 1;
                        elseif(in.theta == 90 || in.theta == -270)
                            out.val = 1;
                        elseif(in.theta == 180 || in.theta == -180)
                            out.val = 0;
                        else
                            out.val = -1;
                        end
                    end
                end 
            end
        end
    elseif(in.m1.x == in.m2.x && in.m1.x~=in.x)
        %向右上方走
        if(in.m1.x<in.x)
            if(in.theta == 0 || in.theta == 360 || in.theta == -360)
                out.val = 0;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = -1;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = -1;
            else
                out.val = 1;
            end
        %向左上方走    
        elseif(in.m1.x>in.x)
            if(in.theta == 0 || in.theta == -360 || in.theta == 360)
                out.val = 1;
            elseif(in.theta == 90 || in.theta == -270)
                out.val = 1;
            elseif(in.theta == 180 || in.theta == -180)
                out.val = -1;
            elseif(in.theta == -90 || in.theta == 270)
                out.val = -1;
            end     
        end
    end
end
%end of a function
end