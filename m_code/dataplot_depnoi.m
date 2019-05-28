clear all
close all
clc

%% load data
%  data order: 
% timestamp, flag Auto/manual, total duration Auto, 4commands, control_vel, 
% control_angular, pos robot(x,y), vel robot(x,y), yaw robot, people_near,
% pos person_standing, pos person_standing_0, pos actor_01, pos actor_02, pos actor_03,
% pos actor_04, pos actor_05, pos actor_06, pos actor_07, pos actor_08

data = table2array(importfile('routeCommand_2019_05_13_17_54_27.csv'));
len = length(data(2:end,1));
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
flagPeopleNear = data(2:end,12);
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
        flagPeopleNear(i,1) = 0;
    end
end
[newFlagPeopleNear,countTimeOfMeeting, periods] = countMeetPeople(flagPeopleNear);

%% people near position
[Distances,NearstDists,AvgDist] = avgPeopleNearestDist(periods,robotPos, peopleStanding, actors);

%% map & plot
% h = drawRect(robotPos);
