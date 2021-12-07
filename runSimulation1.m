function [] = runSimulation1(s1, t1, s2, t2, s3, t3,timeout)
% Runs the simulation until timeout
% s1: Source location of first aircraft
% t1: Target location of first aircraft
% s2: Source location of second aircraft
% t2: Target location of second aircraft
% s3: Source location of third aircraft
% t3: Target location of third aircraft
% timeout: Number of steps the simulation is run. 

% v: Velocity of aircraft
% k: Time after which the direction is updated
% q: Distance at which the messages from other aircraft are available. 
q = 2;  
k = 1;
v = 1;

out.val = 0;

% Initialize inputs to controller based source and target information
in = initGoalParams1(s1,s2,s3,t1,t2,t3,q);

%Compute positions of aircrafts of each step
pos1 = [];
pos2 = [];
pos3 = [];

s1 = [];
s2 = [];
s3 = [];

for i=1:timeout
    %Compute controller outputs
    [out(1),s1] = controller1(in(1),s1);
    [out(2),s2] = controller1(in(2),s2);
    [out(3),s3] = controller1(in(3),s3);
    
    pos1 = [pos1; in(1).x in(1).y in(1).theta, out(1).val];
    pos2 = [pos2; in(2).x in(2).y in(2).theta, out(2).val];
    pos3 = [pos3; in(3).x in(3).y in(3).theta, out(3).val];
    %If both aircraft reached destination,stop
    
    if(in(1).x == in(1).xd && in(1).y == in(1).yd && in(2).x == in(2).xd && in(2).y == in(2).yd && in(3).x == in(3).xd && in(3).y == in(3).yd)
        break;
    end
    %Simulate the motion of next k steps
    in = simulateStep1(out, in, v, k, q);
end    


% Plot trajectory of aircraft
for i=1:max(size(pos1, 1))
    plot(pos1(1:i, 1), pos1(1:i, 2), 'b+-');
    hold on;
    plot(pos2(1:i, 1), pos2(1:i, 2), 'r+-');
    hold on;
    plot(pos3(1:i, 1), pos3(1:i, 2), 'g+-');
    hold on;
    plot(pos1(i, 1), pos1(i, 2), 'ok', 'MarkerSize',5,'MarkerFaceColor','k');
    plot(pos2(i, 1), pos2(i, 2), 'ok', 'MarkerSize',5,'MarkerFaceColor','k');
    plot(pos3(i, 1), pos3(i, 2), 'ok', 'MarkerSize',5,'MarkerFaceColor','k');
    plot(in(1).xd, in(1).yd, 'xb', 'MarkerSize',10,'MarkerFaceColor','b');
    plot(in(2).xd, in(2).yd, 'xr', 'MarkerSize',10,'MarkerFaceColor','r');
    plot(in(3).xd, in(3).yd, 'xg', 'MarkerSize',10,'MarkerFaceColor','g');
    hold off;
    
    % axis
    lx = min([min(pos1(:,1)),min(pos2(:,1)),min(pos3(:,1)),in(1).xd,in(2).xd,in(3).xd])-1;
    rx = max([max(pos1(:,1)),max(pos2(:,1)),max(pos3(:,1)),in(1).xd,in(2).xd,in(3).xd])+1;
    ly = min([min(pos1(:,2)),min(pos2(:,2)),min(pos3(:,2)),in(1).yd,in(2).yd,in(3).yd])-1;
    ry = max([max(pos1(:,2)),max(pos2(:,2)),max(pos3(:,2)),in(1).yd,in(2).yd,in(3).yd])+1;
    axis([lx, rx, ly, ry ])
    pause(.5)
end
end
      
    