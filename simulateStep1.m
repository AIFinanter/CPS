function [ res ] = simulateStep1(out, in, v, k, q )
% Sends updated inputs to controller every k seconds

% in, res: Data Structure that stores input information for the aircraft
% controller. 
%       (x, y): Current Location of the aircraft
%       (xd, yd): Destination of aircraft
%       theta: Current direction of motion
%       m1: Message from smaller index neighbouring aircraft
%       m2: Message from biggger index neighbouring aircraft
% out : Data Structure that stores the output information from the aircraft
%       val: +1, 0, -1
% 
% v: Velocity of aircraft
% k: Time after which the direction is updated
% q: Distance at which the messages from other aircraft are available. 


res = in;
for i=1:length(in)
    
    theta = wrapTo360(out(i).val*90 + in(i).theta);
    
    if(in(i).x == in(i).xd && in(i).y == in(i).yd)
        res(i).x = in(i).x;
        res(i).y = in(i).y;
    elseif(theta == 0 || theta == 360)
        res(i).x = in(i).x + k*v;
    elseif(theta == 90)
        res(i).y = in(i).y + k*v;
    elseif(theta == 180)
        res(i).x = in(i).x - k*v;
    elseif(theta == 270)
        res(i).y = in(i).y - k*v;
    end
    res(i).theta = theta;
end

% if aircrafts are close to each other then m is non-empty and both have
% not reached their destination


res(1).m1 = [];
res(1).m2 = [];
res(2).m1 = [];
res(2).m2 = [];
res(3).m1 = [];
res(3).m2 = [];


if(abs(res(1).x - res(2).x) <= q && abs(res(1).y - res(2).y) <= q && (res(1).x ~= res(1).xd || res(1).y ~= res(1).yd) && (res(2).x ~= res(2).xd || res(2).y ~= res(2).yd))
    res(1).m1 = res(2);
    res(2).m1 = res(1);
end

if(abs(res(1).x - res(3).x) <= q && abs(res(1).y - res(3).y) <= q && (res(1).x ~= res(1).xd || res(1).y ~= res(1).yd) && (res(3).x ~= res(3).xd || res(3).y ~= res(3).yd))
    res(1).m2 = res(3);
    res(3).m1 = res(1);
end

if(abs(res(2).x - res(3).x) <= q && abs(res(2).y - res(3).y) <= q && (res(2).x ~= res(2).xd || res(2).y ~= res(2).yd) && (res(3).x ~= res(3).xd || res(3).y ~= res(3).yd))
    res(2).m2 = res(3);
    res(3).m2 = res(2);
end  
end