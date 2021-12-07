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
    %�������ȼ��İ취�������յ���Ľڵ�����ȼ���ߣ������յ�Զ�Ľڵ�����ȼ�����
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
    %����ڵ�Ӧ�������Ϸ����Ա�����ײ
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
        %���¶�
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
    %������ĺ�������ͬ    
    elseif(in.m1.x == in.x && in.m2.x == in.x)
        %������Ľڵ������Ϸ���
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
        %������Ľڵ������·���    
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
        %���������ڵ�ȫ���ÿ��˵�·���м�Ľڵ����ֱ�����յ��ƶ�
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
        %in.m1��ʾ����ͬһ�еĽڵ㣬in.m2��ʾ���Ǻʹ˽ڵ㲻��ͬһ�еĽڵ�
        if(in.m1.x == in.x)
            tmp1 = in.m1;
            tmp2 = in.m2;
        else
            tmp1 = in.m2;
            tmp2 = in.m1;
        end    
        %����ͬһ�еĽڵ����������ڵ���ɵ��е��ұ�
        if(tmp2.x>in.x)
            
            %����ڵ�������һ������ͬ�еĽڵ���ϱ�
            if(in.y > tmp1.y)
                %�����Ϸ��ߣ�������������ȿ��ǣ�
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
                %����ڵ�������һ������ͬ�еĽڵ���±�
                
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
                %������
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
                %���������������ͨ��Ȩ�Ľڵ㷢����һ���ڵ��������ԽǴ�,�����·���
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
        %�����Ϸ���
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
        %�����Ϸ���    
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