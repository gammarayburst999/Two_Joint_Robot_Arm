%%%%%%%%%%%%%
%
% Mark Whitty

close all; clear variables; %clc;
%dbstop if error
scrsz = get(groot,'ScreenSize'); % Get screen size for plotting

% Construct a 2-link simple planar manipulator, using section 7.2.1 of
% Corke. See 'Tutorial problem 1' in the lecture notes for Kinematics 3.
L(1) = Link([0 0 0.5 0]);
L(2) = Link([0 0 0.5 0]);
one_link = SerialLink(L(1), 'name', 'one link');
two_link = SerialLink(L, 'name', 'two link');
q1 = [0, 0];
q2 = [45, 170]/180*pi;
q3 = [45, -170]/180*pi;
q4 = [-45, -170]/180*pi;

% Define matrix over which configuration space will be sampled
n1 = 100;
min1 = -pi;
max1 = pi;
n2 = 100;
min2 = -150/180*pi;
max2 = 150/180*pi;
angles1 = min1:(max1 - min1)/(n1 - 1):max1;
angles2 = min2:(max2 - min2)/(n2 - 1):max2;
config_space_binary = zeros(n1, n2);
map=zeros(n1, n2);
% Limits on range of motion in Cartesian plan
y_top = 0.6;
y_bottom = 0;
theta_rad=[5 5]*(1/180)*pi;
theta_rad_02=[175 -5]*(1/180)*pi;
[xstart, ystart, zstart] = transl(two_link.fkine(theta_rad));
[xfinal, yfinal, zfinal] = transl(two_link.fkine(theta_rad_02));
Startx = angle2cell(theta_rad(1), min1, max1, n1);
Starty = angle2cell(theta_rad(2), min1, max1, n1);
Finalx = angle2cell(theta_rad_02(1), min1, max1, n1);
Finaly = angle2cell(theta_rad_02(2), min1, max1, n1);
%% Generate discretised plot of joint space
for i = angles1
    % Check position at end of first link
    [x1, y1, z1] = transl(one_link.fkine(i));
    for j = angles2
        % If position of first link invalid, flag it.
        if(y1 < y_bottom || y1 > y_top)
            index_x = angle2cell(i, min1, max1, n1);
            index_y = angle2cell(j, min2, max2, n2);
            config_space_binary(index_x, index_y) =10;
            map(index_x, index_y) =1;
            continue;
        end
        % Check position at end of second link
        [x2, y2, z2] = transl(two_link.fkine([i, j]));
        if(y2 < y_bottom || y2 > y_top) % If invalid, flag it.
            index_x = angle2cell(i, min1, max1, n1);
            index_y = angle2cell(j, min2, max2, n2);
            config_space_binary(index_x, index_y) = 10;
            map(index_x, index_y) =1;
        end
    end
end
config_space_binary(Startx,Starty)=2;
config_space_binary(Finalx,Finaly)=7;
%% Plot this
f1 = figure(1);
set(gcf, 'OuterPosition',[100 100 scrsz(3)/2-100 scrsz(4)/2]);
imagesc(config_space_binary'); % Note the transpose of the joint space for convenience
axis xy; % Sets axis with x and y positive directions as expected, not as image coordinates
xlabel('Joint 1 [index in cell array], corresponding to 0 degrees in the middle')
ylabel('Joint 2 [index in cell array], corresponding to 0 degrees in the middle')
% load map
start=[Startx,Starty];
goal = [Finalx,Finaly];

ds = Dstar(map');    % create navigation object
ds.plan(goal)% create plan for specified goal
p1=ds.query(start);
Dsangle=zeros(length(p1),2);
for k=1:length(p1)
    Dsangle(k,1) = cell2angle(p1(k,1), min1, max1, n1);
    Dsangle(k,2) = cell2angle(p1(k,2), min2, max2, n2);
end

f2 = figure(2);
ds.query(start, 'animate')
f3 = figure(3);
two_link.plot(Dsangle)


prm = PRM(map');        % create navigation object
prm.plan()    % create roadmaps
f4 = figure(4);
p2=prm.query(start, goal);  % animate path from this start loc
prm.plot(p2);

PRMangle=zeros(length(p2),2);
for k=1:length(p2)
    PRMangle(k,1) = cell2angle(p2(k,1), min1, max1, n1);
    PRMangle(k,2) = cell2angle(p2(k,2), min2, max2, n2);
end
f5 = figure(5);
two_link.plot(PRMangle)