function [avg_vel_abs, avg_min_dis_in_range,avg_avg_dis_in_range,avg_cmd_vel, ...
    avg_vel_decline_percentage, avg_vel_decline_percentage_in_range] = ...
    avgPeopleNearestDist_sim(periods,robotPos,robotVel,controlVel,peopleSatanding,Actors,range_near)
%avgNearestDist calculate the average distances between the robot and the
%person when the person is in the sight of the robot
% 这个函数计算速度减小的部分avg_vel_decline_percentage和avg_vel_decline_percentage_in_range需要更改，
% 依据对应的real函数文件。目前是用两倍range距离中的速度代表平时的未减速的速度，
% 要改成全局的符合三个条件的平均，跟实际场景保持一致

% ---------------  input ------------------- %
% periods: indexes(start, end) of periods of seeing people
% robotPos: positions of the robot
% robotVel: velocities of the robot
% peopleSatanding: position of the standing people
% Actors: positions of the walking people

% ---------------  output ------------------- %
% Distances: during each period, the distances between the robot and
% peolple
% avg_vel_abs: average absolute velocity during people are in the range
% threshold
% avg_vel_decline: average of the percentage of the decline in velocity of
% from 2 times of threshold to threshold
% avg_min_dis_in_range: average minimum distances during people are in the range threshold
% avg_cmd_vel: average command on velocity during periods where people are
% in the range threshold


len_period = length(periods(:,1)); % number of periods
Distances = cell(1, len_period); % period * number of people
min_dists = zeros(len_period,10); % minimum distance to 10 people in each period
candidates_index = zeros(len_period,1); % nearest people index in each period
candidates_min = zeros(len_period,1); 
velocities = cell(1,len_period);
velocities_before_brake = cell(1,len_period);
cmd_vels = cell(1,len_period);
distances_in_range = cell(1,len_period);

for i = 1:len_period
    velocity_near = [];
    distance_near = [];
    cmdvel_near = [];
    velocity_before_brake = [];
    i_period = periods(i,2) - periods(i,1) + 1; % length of the ith period
    dists = zeros(i_period,10); % [i_period, 10]
    % position vectors
    i_robot_pos = robotPos( periods(i,1):periods(i,2), : );
    i_peopleSatanding = peopleSatanding(periods(i,1):periods(i,2), : );
    i_actor = Actors(periods(i,1):periods(i,2), : );
    i_velocity = robotVel(periods(i,1):periods(i,2), : );
    i_cmdVel = controlVel(periods(i,1):periods(i,2), : );
    
    % calculate distances
    dists(:,1:2) = distPoint2Points(i_robot_pos, i_peopleSatanding); 
    dists(:,3:10) = distPoint2Points(i_robot_pos, i_actor); 
    
    % calculate the minimum distance to each people during the period
    min_dists(i,:) = min(dists,[],1);
    
    % the minimum among the min_dist is the candidate of this period
    [candidates_min(i,1), candidate_index] = min(min_dists(i,:),[],2);
    candidates_index(i,1) = candidate_index;
    Distances{1,i} = dists;
    
    % velocities when near people
    for j = 1:length(dists(:,candidate_index))
        if dists(j,candidate_index) >= range_near && dists(j,candidate_index) <= 2*range_near
            velocity_before_brake = [velocity_before_brake; i_velocity(j,:)];
        elseif dists(j,candidate_index) < range_near
            velocity_near = [velocity_near; i_velocity(j,:)];
            distance_near = [distance_near; dists(j,candidate_index)];
            cmdvel_near = [cmdvel_near; i_cmdVel(j,:)];
        end
    end
    distances_in_range{1,i} = distance_near;
    velocities{1,i} = velocity_near;
    velocities_before_brake{1,i} = velocity_before_brake;
    cmd_vels{1,i} = cmdvel_near;
end

% avg vel, control vel and pos in the range of threshold (2m)
avg_vel_abs = zeros(1,len_period);
avg_vel_before_brake = zeros(1,len_period);
avg_cmd_vel = zeros(1,len_period);
min_dis_in_range = zeros(1,len_period);
avg_dis_in_range = zeros(1,len_period);
% avg_min_dis_in_range = 0;
% avg_cmd_vel_in_range = 0;
% avg_vel_abs_in_range = 0;
count_non_empty = 0;
for i = 1:len_period
    i_vel = velocities{1,i};
    i_vel_before_brake = velocities_before_brake{1,i};
    i_distance = distances_in_range{1,i};
    i_cmdvel = cmd_vels{1,i};
    if isempty(i_vel)
        continue;
    else
        count_non_empty = count_non_empty + 1;
        avg_vel_abs(1,i) = sum(sqrt(i_vel(:,1).*i_vel(:,1) + i_vel(:,2).*i_vel(:,2)))/length(i_vel(:,1));
        if isempty(i_vel_before_brake)
            avg_vel_before_brake(1,i) = 0;
        else
            avg_vel_before_brake(1,i) = sum(sqrt(i_vel_before_brake(:,1).*i_vel_before_brake(:,1) + ...
                i_vel_before_brake(:,2).*i_vel_before_brake(:,2)))/length(i_vel_before_brake(:,1));
        end
        min_dis_in_range(1,i) = min(i_distance(:,1));
        avg_dis_in_range(1,i) = sum(i_distance(:,1)) / length(i_distance(:,1));
        avg_cmd_vel(1,i) = sum(i_cmdvel(:,1))/length(i_cmdvel(:,1));
    end
end
avg_min_dis_in_range = sum(min_dis_in_range)/count_non_empty;
avg_avg_dis_in_range = sum(avg_dis_in_range)/count_non_empty;
avg_cmd_vel_in_range = sum(avg_cmd_vel)/count_non_empty;
avg_vel_abs_in_range = sum(avg_vel_abs)/count_non_empty;
avg_vel_decline_percentage = (avg_vel_before_brake- avg_vel_abs)./avg_vel_before_brake;
% avg_vel_decline_percentage(isinf(avg_vel_decline_percentage)) = 0;

valid_index = ~isnan(avg_vel_decline_percentage) & ~isinf(avg_vel_decline_percentage);
avg_vel_decline_percentage_in_range = ...
    sum((avg_vel_before_brake(valid_index) - avg_vel_abs(valid_index))./ ...
    avg_vel_before_brake(valid_index)) / length(valid_index);
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











