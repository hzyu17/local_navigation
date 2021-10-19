function [avg_vel_abs,avg_min_dis_in_range,avg_avg_dis_in_range,avg_cmd_vel,...
          vel_decline_percent_in_range,cmd_vel_decline_percent_in_range] = ...
                avgPeopleNearestDist_real(periods,robotPos,robotVel,controlVel,controlAngular,flagAutoManual,...
                min_person_dist,range_near,flagSeePeople)
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
threshold_angular_vel = 0.1;
num_periods = length(periods(:,1)); % number of periods
period_not_see_people = zeros(length(robotVel(:,1)));
period_not_see_people(1:periods(1,1)) = 1;
for i = 2: num_periods
    period_not_see_people(periods(i-1,2):periods(i,1)) = 1;
end
controlAngular_copy = zeros(size(controlAngular));
controlAngular_copy(abs(controlAngular) < threshold_angular_vel) = 1;
% 自动控制，没有看到人，同时也没有在转弯时候的index
period_auto_not_see_people_and_low_angular = flagAutoManual & controlAngular_copy & period_not_see_people;

cell_velocities = cell(1,num_periods);
cell_cmd_vels = cell(1,num_periods);
cell_velocities_for_comparison = cell(1,num_periods);
cell_cmd_vel_for_comparison = cell(1,num_periods);
cell_distances_in_range = cell(1,num_periods);
% 没有看到人的速度
vel_not_see_people = robotVel(period_not_see_people==1,:);
% 没有看到人的平均速度
avg_vel_not_see_people = sum(sqrt(vel_not_see_people(:,1).*vel_not_see_people(:,1) + ...
                             vel_not_see_people(:,2).*vel_not_see_people(:,2)))./length(vel_not_see_people(:,2));
% 没有看到人，同时也没有在转弯时候的速度
vel_not_see_people_and_low_angular = robotVel(period_auto_not_see_people_and_low_angular,:);
% 没有看到人，同时也没有在转弯时候的平均速度
avg_vel_not_see_people_and_low_angular = sum(sqrt(vel_not_see_people_and_low_angular(:,1).*vel_not_see_people_and_low_angular(:,1) + ...
                             vel_not_see_people_and_low_angular(:,2).*vel_not_see_people_and_low_angular(:,2)))./length(vel_not_see_people_and_low_angular(:,2));
range_for_comparison = 10; % 用来计算遇到人之前（后）的平均速度的帧数

for i = 1:num_periods
    flag_reach_range_for_comparison = false;
    velocity_near = [];
    distance_near = [];
    cmdvel_near = [];
    i_velocity_for_comparison = [];
    i_cmdVel_for_comparison = [];
    cmdVel_for_comparison_near = [];
    velocity_for_comparison_near = [];
    i_period_len = periods(i,2) - periods(i,1) + 1; % length of the ith period
    near_person_dists = min_person_dist(periods(i,1):periods(i,2), : ); % [i_period, 10]
    % position vectors
    i_robot_pos = robotPos( periods(i,1):periods(i,2), : );
    i_velocity = robotVel(periods(i,1):periods(i,2), : );
    i_counter = 1;
    
    while ~flag_reach_range_for_comparison
        if flagSeePeople(periods(i,1)-i_counter,1) == 1
            i_velocity_for_comparison = [i_velocity_for_comparison;robotVel(periods(i,1)-i_counter,:)];
            i_cmdVel_for_comparison = [i_cmdVel_for_comparison;controlVel(periods(i,1)-i_counter,:)];
            i_counter = i_counter + 1;
            if i_counter >= range_for_comparison
                flag_reach_range_for_comparison = true;
            end
        else
            i_counter = i_counter + 1;
            continue;
        end
    end
%     i_velocity_for_comparison = [ robotVel(periods(i,1)-range_for_comparison:periods(i,1),: );
%                                                       robotVel(periods(i,2):periods(i,2)+range_for_comparison, : )];
    i_cmdVel = controlVel(periods(i,1):periods(i,2), : );
%     i_cmdVel_for_comparison = [ controlVel(periods(i,1)-range_for_comparison:periods(i,1),: );
%                                                       controlVel(periods(i,2):periods(i,2)+range_for_comparison, : )];
    % velocities when near people
    for j = 1:length(near_person_dists(:,1))
        if near_person_dists(j,1) < range_near
            velocity_near = [velocity_near; i_velocity(j,:)];
%             velocity_for_comparison_near = [velocity_for_comparison_near; i_velocity_for_comparison(j,:)];
            distance_near = [distance_near; near_person_dists(j,1)];
            cmdvel_near = [cmdvel_near; i_cmdVel(j,:)];
%             cmdVel_for_comparison_near = [cmdVel_for_comparison_near; i_cmdVel_for_comparison];
        end
    end
    cell_distances_in_range{1,i} = distance_near;
    cell_velocities{1,i} = velocity_near;
    cell_cmd_vels{1,i} = cmdvel_near;
    cell_cmd_vel_for_comparison{1,i} = i_cmdVel_for_comparison;
    cell_velocities_for_comparison{1,i} = i_velocity_for_comparison;
end

% avg vel, control vel and pos in the range of threshold (2m)
avg_vel_abs = zeros(1,num_periods);
avg_cmd_vel = zeros(1,num_periods);
min_dis_in_range = zeros(1,num_periods);
avg_dis_in_range = zeros(1,num_periods);
count_non_empty = 0;
for i = 1:num_periods
    i_vel = cell_velocities{1,i};
    i_velComparison = cell_velocities_for_comparison{1,i};
    i_distance = cell_distances_in_range{1,i};
    i_cmdvel = cell_cmd_vels{1,i};
    i_cmdvelComparison = cell_cmd_vel_for_comparison{1,i};
    
    if isempty(i_vel)
        continue;
    else
        count_non_empty = count_non_empty + 1;
        avg_vel_abs(1,i) = sum(sqrt(i_vel(:,1).*i_vel(:,1) + i_vel(:,2).*i_vel(:,2)))/length(i_vel(:,1));
        min_dis_in_range(1,i) = min(i_distance(:,1));
        avg_dis_in_range(1,i) = sum(i_distance(:,1)) / length(i_distance(:,1));
        avg_cmd_vel(1,i) = sum(i_cmdvel(:,1))/length(i_cmdvel(:,1));
        avg_cmd_vel_for_comparison(1,i) = sum(i_cmdvelComparison(:,1))/length(i_cmdvelComparison(:,1));
        avg_vel_for_comparison(1,i) = sum(sqrt(i_velComparison(:,1).*i_velComparison(:,1) + ...
                                          i_velComparison(:,2).*i_velComparison(:,2))) ...
                                          /length(i_velComparison(:,1));
    end
end
avg_avg_dis_in_range = sum(avg_dis_in_range)/count_non_empty;
avg_min_dis_in_range = sum(min_dis_in_range)/count_non_empty;
avg_cmd_vel_in_range = sum(avg_cmd_vel)/count_non_empty;
avg_cmd_vel_comparison_in_range = sum(avg_cmd_vel_for_comparison)/count_non_empty;
avg_vel_abs_in_range = sum(avg_vel_abs)/count_non_empty;
avg_vel_abs_for_comparison_in_range = sum(avg_vel_for_comparison)/count_non_empty;
% vel_decline_percent_in_range = (avg_vel_not_see_people - avg_vel_abs)./ ...
%     avg_vel_not_see_people;
vel_decline_percent_in_range = (avg_vel_not_see_people_and_low_angular - avg_vel_abs)./ ...
                                avg_vel_not_see_people_and_low_angular;
cmd_vel_decline_percent_in_range = (avg_cmd_vel_for_comparison - avg_cmd_vel)./ ...
    avg_cmd_vel_for_comparison;
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











