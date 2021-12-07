function [ params ] = initGoalParams1( s1, s2, s3, t1, t2, t3, q)
% Initialize Goal Parameters

params(1).x = s1(1);
params(1).xd = t1(1);
params(1).y = s1(2);
params(1).yd = t1(2);
params(1).theta = 0;

params(2).x = s2(1);
params(2).xd = t2(1);
params(2).y = s2(2);
params(2).yd = t2(2);
params(2).theta = 0;

params(3).x = s3(1);
params(3).xd = t3(1);
params(3).y = s3(2);
params(3).yd = t3(2);
params(3).theta = 0;

params(1).m1 = [];
params(1).m2 = [];
params(2).m1 = [];
params(2).m2 = [];
params(3).m1 = [];
params(3).m2 = [];

if(abs(params(1).x-params(2).x)<=q && abs(params(1).y-params(2).y)<=q)
    params(1).m1 = params(2);
    params(2).m1 = params(1);
end    

if(abs(params(1).x-params(3).x)<=q && abs(params(1).y-params(3).y)<=q)
    params(1).m2 = params(3);
    params(3).m1 = params(1);
end

if(abs(params(2).x-params(3).x) && abs(params(2).y-params(3).y)<=q)
    params(2).m2 = params(3);
    params(3).m2 = params(2);
end


end