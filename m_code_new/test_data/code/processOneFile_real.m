function [ avg_cmd_vel, avg_vel_abs, avg_min_dis_in_range, avg_avg_dis_in_range,...
    robotPos, vel_decline_percent_in_range, cmd_vel_decline_percent_in_range, total_duration] = processOneFile_real( filename, range_near )
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

% people
flagSeePeople = data(2:end,12);
min_person_dist = data(2:end,13);


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
[newFlagPeopleNear,countTimeOfMeeting, periods] = countMeetPeople(flagSeePeople);

%% 画图，测试对行人礼让程度的判断算法的有效性
comparison = zeros(size(flagSeePeople));
for i = 1:length(periods(:,1))
    comparison(periods(i,1):periods(i,2)) = 0.9;
end

num_periods = length(periods(:,1)); % number of periods
period_not_see_people = 1:periods(1,1);
comparison_period_not_see_people = zeros(size(flagSeePeople));
for i = 2: num_periods
    period_not_see_people = [period_not_see_people,periods(i-1,2):periods(i,1)];
end
comparison_period_not_see_people(period_not_see_people)=0.7;
flagAutoManual_copy = flagAutoManual;
flagAutoManual_copy(flagAutoManual_copy == 1) = 0.5;
figure
lgd = {'flagSeePeople','periods'};
plot(flagSeePeople,'*')
title(filename)
grid minor
hold on
plot(comparison,'o')
plot(flagAutoManual_copy,'-')
plot(comparison_period_not_see_people,'+')
legend(lgd)
for i = 1:length(periods(:,1))
    plot([periods(i,1),periods(i,1)],[0.9,1],'r')
    plot([periods(i,2),periods(i,2)],[0.9,1],'r')
end
vels = (robotVel(:,1).*robotVel(:,1) + robotVel(:,2) .* robotVel(:,2));
vels = vels(vels<10);
% for i = 2:length(vels)
%     vels(i) = 0.9 * vels(i-1) + 0.1 * vels(i);
% end
% plot(1:length(vels),(vels.*5)','b')

controlAngular_copy = zeros(size(controlAngular));
controlAngular_copy(abs(controlAngular)<0.2) = 0.5;
% for i = 2:length(controlAngular)
%     controlAngular_copy(i) = 0.9 * controlAngular_copy(i-1) + 0.1 * controlAngular_copy(i);
% end
plot(1:length(controlAngular_copy),(controlAngular_copy)','+')

% command velocity, low-pass filter
controlVel_copy = controlVel(controlVel<10);
for i = 2:length(controlVel)
    controlVel_copy(i) = 0.9 * controlVel_copy(i-1) + 0.1 * controlVel_copy(i);
end
plot(1:length(controlVel_copy),(controlVel_copy.*1.5)','b')

%% distance to people near and velocity when people in range of 2m  
% range_near = 2;
[avg_vel_abs,avg_min_dis_in_range,avg_avg_dis_in_range,avg_cmd_vel,vel_decline_percent_in_range,...
 cmd_vel_decline_percent_in_range] = avgPeopleNearestDist_real( ...
                                     periods,robotPos,robotVel,controlVel,controlAngular,flagAutoManual,...
                                     min_person_dist,range_near,flagSeePeople);
% avg_vel_abs
% avg_min_dis_in_range
% figure 
% hold on 
% grid minor
% plot(avg_cmd_vel,'o','LineWidth',2)
% plot(avg_vel_abs,'*','LineWidth',2)
% legend('avg cmd vel','avg vel abs')

end

