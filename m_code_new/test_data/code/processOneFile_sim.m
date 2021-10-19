function [ avg_cmd_vel, avg_vel_abs, avg_min_dis_in_range, avg_avg_dis_in_range,robotPos, ...
                 avg_vel_decline_percentage_in_range,total_duration] = ...
    processOneFile_sim( filename, flag_type, range_near )
%processOneFile process one trial from reading data to data processing 

%% load data
%  data order: 
% timestamp, flag Auto/manual, total duration Auto, 4commands, control_vel, 
% control_angular, pos robot(x,y), vel robot(x,y), yaw robot, people_near,
% pos person_standing, pos person_standing_0, pos actor_01, pos actor_02, pos actor_03,
% pos actor_04, pos actor_05, pos actor_06, pos actor_07, pos actor_08

% read data
data = importfile(filename);
len = length(data(2:end,1));


%% parse data
timestamp = data(2:end,1);
flagAutoManual = data(2:end,2);
durationAuto = data(2:end,3);
total_duration = durationAuto(end,1);
commandDir = data(2:end,4);
controlVel = data(2:end,5);
controlAngular = data(2:end,6);

% robot
robotPos = data(2:end,7:8);
robotVel = data(2:end,9:10);
robotYaw = data(2:end,11);
flagSeePeople = data(2:end,12);

% people
peopleStanding = data(2:end,13:16); % 2 people 
actors = data(2:end,17:end); % 8 actors


%% data pre-processing 
% find valid period of seeing people
countTimeOfMeeting = 0; % number of seeing people periods 
periods = []; % periods(start, end) of continuously seeing people
newFlagPeopleNear = []; % flag after delay process

% only when under auto mode the calculation of distance to people is valid
for i = 1:len
    if flagAutoManual(i,1) == 0
        flagSeePeople(i,1) = 0;
    end
end

% finite state machine
% 有限状态机
[newFlagPeopleNear,countTimeOfMeeting, periods] = countMeetPeople(flagSeePeople);

%% distance to people near and velocity when people in range of 'range_near' 
% range_near = 2;
% avgPeopleNearestDist_sim:计算最后想要的一系列平均值结果的函数
%参数periods: 每一行是一个遇到人的区间，(n*2),第一列是这次遇到人的起点帧数，第二列是终点帧数
[avg_vel_abs,avg_min_dis_in_range,avg_avg_dis_in_range,avg_cmd_vel, ...
    avg_vel_decline_percentage, avg_vel_decline_percentage_in_range] = ...
    avgPeopleNearestDist_sim(periods,robotPos, robotVel, controlVel, peopleStanding, actors, range_near);

% figure 
% grid on
% hold on
% bar(avg_vel_decline_percentage_in_range)
% title(flag_type)

% avg_vel_abs
% avg_min_dis_in_range
% figure 
% hold on 
% grid minor
% plot(avg_cmd_vel,'o','LineWidth',2)
% plot(avg_vel_abs,'*','LineWidth',2)
% legend('avg cmd vel','avg vel abs')

% map & plot
% h = drawRect(robotPos, filename, flag_type);

end

