function [ avg_cmd_vel, avg_vel_abs, avg_min_dis_in_range ] = processOneFile_sim( filename )
%processOneFile process one trial from reading data to data processing 

%% load data
%  data order: 
% timestamp, flag Auto/manual, total duration Auto, 4commands, control_vel, 
% control_angular, pos robot(x,y), vel robot(x,y), yaw robot, people_near,
% pos person_standing, pos person_standing_0, pos actor_01, pos actor_02, pos actor_03,
% pos actor_04, pos actor_05, pos actor_06, pos actor_07, pos actor_08

% read data
data = table2array(importfile(filename));
len = length(data(2:end,1));


%% parse data
timestamp = data(2:end,1);
flagAutoManual = data(2:end,2);
durationAuto = data(2:end,3);
commandDir = data(2:end,4);
controlVel = data(2:end,5);
controlAngular = data(2:end,6);

% robot
robotPos = data(2:end,7:8);
robotVel = data(2:end,9:10);
robotYaw = data(2:end,11);

% people
flagPeopleNear = data(2:end,12);
min_person_dist = data(2:end,13);


%% data pre-processing 
% find valid period of seeing people
countTimeOfMeeting = 0; % number of seeing people periods 
periods = []; % periods(start, end) of continuously seeing people
newFlagPeopleNear = []; % flag after delay process

% only when under auto mode the calculation of distance to people is valid
for i = 1:len
    if flagAutoManual(i,1) == 0
        flagPeopleNear(i,1) = 0;
    end
end

% finite state machine
[newFlagPeopleNear,countTimeOfMeeting, periods] = countMeetPeople(flagPeopleNear);

%% distance to people near and velocity when people in range of 2m  
range_near = 3;
[avg_vel_abs,avg_min_dis_in_range,avg_cmd_vel] = avgPeopleNearestDist_real(periods,robotPos,robotVel,controlVel,min_person_dist,range_near);
% avg_vel_abs
% avg_min_dis_in_range
% figure 
% hold on 
% grid minor
% plot(avg_cmd_vel,'o','LineWidth',2)
% plot(avg_vel_abs,'*','LineWidth',2)
% legend('avg cmd vel','avg vel abs')

end

