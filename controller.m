 
function [ out, state ] = controller( in, state )
% Takes flight parameters of an aircraft and outputs the direction control

%好好学习天天向上

% in: Data Structure that stores input information for the aircraft
% controller. 
%       (in.x, in.y): Current Location of the aircraft
%       (in.xd, in.yd): Destination of aircraft
%       in.theta: Current direction of motion
%       in.m: Message from neighbouring aircraft 
%           - empty if aircraft not in neighbourhood
%           - (x, y, xd, yd, theta) of other aircraft if non-empty
%           - To access data (say x) from in.m, use in.m.x
%
% out : Data Structure that stores the output information from the aircraft
%       out.val: +1, 0, -1 ( +1 - turn left, 0 - go straight, -1 - turn right)
% 
% state: 
%       any state used by the controller


% Initialize state

out.val = 0;
%判断是否可能会撞上
if(~isempty(in.m))
    %离终点更近的结点按照既定的算法步骤进行飞行
    if(abs(in.xd-in.x)+abs(in.yd-in.y)<abs(in.m.x-in.m.xd)+abs(in.m.y-in.m.yd))
        if(in.yd>in.y)
            if(in.theta==0||in.theta == 360 || in.theta == -360)        
                out.val = 1;
            elseif(in.theta == 90 || in.theta ==-270)
                out.val = 0;
            elseif(in.theta == -90 || in.theta == 270)
                out.val = 1;
            else
                out.val = -1;    
            end    
        elseif(in.yd<in.y)
                if(in.theta==-90 || in.theta==270)
                    out.val = 0;
                elseif(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = -1;
                elseif(in.theta == 180 || in.theta == -180)    
                    out.val = 1;
                else
                    out.val = 1;
                end
        else
            if(in.xd>in.x)
                if(in.theta==90 || in.theta == -270)
                    out.val = -1;
                elseif(in.theta == 0 || in.theta == 360)
                    out.val = 0;
                elseif(in.theta ==180 || in.theta == -180)
                    out.val = 1;
                else
                    out.val = 1;    
                end
            elseif(in.xd<in.x)
                if(in.theta == 180 || in.theta == -180)
                
                    out.val = 0;
           
                elseif(in.theta == 0 || in.theta == 360)
                    out.val = 1;
                elseif(in.theta == -90 || in.theta == 270)
                    out.val = -1;
                else
                    out.val = 1;
                end
            end
        end
    %离终点更远的节点    
    elseif(abs(in.xd-in.x)+abs(in.yd-in.y)>abs(in.m.xd-in.m.x)+abs(in.m.yd-in.m.y))    
        if(in.theta == 0 || in.theta == 360 || in.theta == -360)
            if(in.m.y>in.y)
                out.val = -1;
            else
                out.val = 1;
            end    
        elseif(in.theta == 90 || in.theta == -270)
            if(in.m.x<in.x)
                out.val = -1;
            else
                out.val = 1;
            end
        elseif(in.theta == 180 || in.theta == -180)
            if(in.m.y>in.y)
                out.val = 1;
            else
                out.val = -1;
            end
        else
            if(in.m.x>in.x)
                out.val = -1;
            else
                out.val = 1;
            end    
        end
          
            
    %abs(in.xd-in.x)+abs(in.yd-in.y)==abs(in.m.xd-in.m.x)+abs(in.m.yd-in.m.y)
    else
        if(in.x*10+in.y<in.m.x*10+in.m.y)
            if(in.yd>in.y)
                if(in.theta==0||in.theta == 360 || in.theta == -360)        
                    out.val = 1;
                elseif(in.theta == 90 || in.theta ==-270)
                    out.val = 0;
                elseif(in.theta == -90 || in.theta == 270)
                    out.val = 1;
                else
                    out.val = -1;    
                end    
            elseif(in.yd<in.y)
                if(in.theta==-90 || in.theta==270)
                    out.val = 0;
                elseif(in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = -1;
                elseif(in.theta == 180 || in.theta == -180)    
                    out.val = 1;
                else
                    out.val = 1;
                end
            else
                if(in.xd>in.x)
                    if(in.theta==90 || in.theta == -270)
                        out.val = -1;
                    elseif(in.theta == 0 || in.theta == 360)
                        out.val = 0;
                    elseif(in.theta ==180 || in.theta == -180)
                        out.val = 1;
                    else
                        out.val = 1;    
                    end
                elseif(in.xd<in.x)
                    if(in.theta == 180 || in.theta == -180)
                        out.val = 0;
                    elseif(in.theta == 0 || in.theta == 360)
                        out.val = 1;
                    elseif(in.theta == -90 || in.theta == 270)
                        out.val = -1;
                    else
                        out.val = 1;
                    end
                end
            end
        else
            if(in.m.x-in.x == -1 && in.m.y-in.y == -1)
                if(in.theta == 180 || in.theta == -180 || in.theta == 90 || in.theta == -270)
                    out.val = -1;
                else
                    out.val = 1;
                end
            elseif(in.m.x-in.x == -1  && in.m.y-in.y == 1)
                if(in.theta == -270 || in.theta == 90 || in.theta == 0 || in.theta == 360 || in.theta == -360)
                    out.val = -1;
                else
                    out.val = 1;
                end
            elseif(in.m.x-in.x == -2)
                if( in.theta == 90 || in.theta == -270)
                    out.val = -1;
                else
                    out.val = 1;
                end
            elseif(in.m.y-in.y == -2)
                if(in.theta == 180 || in.theta == -180)
                    out.val = -1;
                else
                    out. val = 1;
                end    
            end
        
        end   
    end

else    
    if(in.yd>in.y)
        if(in.theta==0||in.theta == 360 || in.theta == -360)        
            out.val = 1;
        elseif(in.theta == 90 || in.theta ==-270)
            out.val = 0;
        elseif(in.theta == -90 || in.theta == 270)
            out.val = 1;
        else
            out.val = -1;    
        end    
    elseif(in.yd<in.y)
        if(in.theta==-90 || in.theta==270)
            out.val = 0;
        elseif(in.theta == 0 || in.theta == 360 || in.theta == -360)
            out.val = -1;
        elseif(in.theta == 180 || in.theta == -180)    
            out.val = 1;
        else
            out.val = 1;
        end
    else
        if(in.xd>in.x)
            if(in.theta==90 || in.theta == -270)
                out.val = -1;
            elseif(in.theta == 0 || in.theta == 360)
                out.val = 0;
            elseif(in.theta ==180 || in.theta == -180)
                out.val = 1;
            else
                out.val = 1;    
            end
        elseif(in.xd<in.x)
            if(in.theta == 180 || in.theta == -180)
                out.val = 0;
            elseif(in.theta == 0 || in.theta == 360)
                out.val = 1;
            elseif(in.theta == -90 || in.theta == 270)
                out.val = -1;
            else
                out.val = 1;
            end
        end
    end
end    