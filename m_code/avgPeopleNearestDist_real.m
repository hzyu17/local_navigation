function [avg_vel_abs,avg_min_dis_in_range,avg_cmd_vel] = avgPeopleNearestDist_real(periods,robotPos,robotVel,controlVel,min_person_dist,range_near)
%avgNearestDist calculate the average distances between the robot and the
%person when the person is in the sight of the robot

% ---------------  input ------------------- %
% periods: indexes(start, end) of periods of seeing people
% robotPos: positions of the robot
% robotVel: velocities of the robot
% peopleSatanding: position of the standing people
% Actors: positions of the walking people

% ---------------  output ------------------- %
% avg_vel_abs: average absolute velocity during people are in the range
% threshold
% avg_min_dis_in_range: average minimum distances during people are in the range threshold
% avg_cmd_vel: average command on velocity during periods where people are
% in the range threshold

len = length(periods(:,1)); % number of periods
min_dists = zeros(len,10); % minimum distance to 10 people in each period
velocities = cell(1,len);
cmd_vels = cell(1,len);
distances_in_range = cell(1,len);

for i = 1:len
    velocity_near = [];
    distance_near = [];
    cmdvel_near = [];
    i_period_len = periods(i,2) - periods(i,1) + 1; % length of the ith period
    near_person_dists = min_person_dist(periods(i,1):periods(i,2), : ); % [i_period, 10]
    % position vectors
    i_robot_pos = robotPos( periods(i,1):periods(i,2), : );
    i_velocity = robotVel(periods(i,1):periods(i,2), : );
    i_cmdVel = controlVel(periods(i,1):periods(i,2), : );
    
    % velocities when near people
    for j = 1:length(near_person_dists(:,1))
        if near_person_dists(j,1) < range_near
            velocity_near = [velocity_near; i_velocity(j,:)];
            distance_near = [distance_near; near_person_dists(j,1)];
            cmdvel_near = [cmdvel_near; i_cmdVel(j,:)];
        end
    end
    distances_in_range{1,i} = distance_near;
    velocities{1,i} = velocity_near;
    cmd_vels{1,i} = cmdvel_near;
end

% avg vel, control vel and pos in the range of threshold (2m)
avg_vel_abs = zeros(1,len);
avg_cmd_vel = zeros(1,len);
min_dis_in_range = zeros(1,len);
count_non_empty = 0;
for i = 1:len
    i_vel = velocities{1,i};
    i_distance = distances_in_range{1,i};
    i_cmdvel = cmd_vels{1,i};
    if isempty(i_vel)
        continue;
    else
        count_non_empty = count_non_empty + 1;
        avg_vel_abs(1,i) = sum(sqrt(i_vel(:,1).*i_vel(:,1) + i_vel(:,2).*i_vel(:,2)))/length(i_vel(:,1));
        min_dis_in_range(1,i) = min(i_distance(:,1));
        avg_cmd_vel(1,i) = sum(i_cmdvel(:,1))/length(i_cmdvel(:,1));
    end
end
avg_min_dis_in_range = sum(min_dis_in_range)/count_non_empty;
avg_cmd_vel_in_range = sum(avg_cmd_vel)/count_non_empty;
avg_vel_abs_in_range = sum(avg_vel_abs)/count_non_empty;
end

function dist = distTwoPoints(Pts1, Pts2)
    dist = sqrt((Pts1(:,1)-Pts2(:,1)).*(Pts1(:,1)-Pts2(:,1)) + (Pts1(:,2)-Pts2(:,2)).*(Pts1(:,2)-Pts2(:,2)));
end

function dists = distPoint2Points(Pt1, Pts)
% ------------- input ------------- %
% Pt1(x,y)
% Pts(x1,y1,x2,y2,...xn,yn)

% ------------- output ------------- %
% dists (dist1, dist2, ..., distn)

len1 = length(Pts(:,1));
len2 = length(Pts(1,:));
dists = zeros(len1,len2/2);
for i=2:2:len2
    dists(:, i/2) = distTwoPoints(Pt1,Pts(:, i-1:i));
end
end











