function [Distances,candidates_min,AvgDist] = avgPeopleNearestDist(periods,robotPos, peopleSatanding, Actors)
%avgNearestDist calculate the average distances between the robot and the
%person when the person is in the sight of the robot
% ---------------  input ------------------- %
% periods: indexes(start, end) of periods of seeing people
% robotPos: positions of the robot
% peopleSatanding: position of the standing people
% Actors: positions of the walking people
% ---------------  output ------------------- %
% Distances: during each period, the distances between the robot and
% peolple
% NearstDist: nearest distance in each period
% AvgDist: average average nearest distance

len = length(periods(:,1));
candidate_pos = {};
Distances = cell(1, len); % period * number of people
min_dists = zeros(len,10);
candidates_index = zeros(len,1);
candidates_min = zeros(len,1);
AvgDist = zeros(len,1);
for i = 1:len
    i_period = periods(i,2) - periods(i,1) + 1;
    dists = zeros(i_period,10); % [i_period,10]
    i_robot_pos = robotPos( periods(i,1):periods(i,2), : );
    i_peopleSatanding = peopleSatanding(periods(i,1):periods(i,2), : );
    i_actor = Actors(periods(i,1):periods(i,2), : );
    dists(:,1:2) = distPoint2Points(i_robot_pos, i_peopleSatanding); 
    dists(:,3:10) = distPoint2Points(i_robot_pos, i_actor); 
    min_dists(i,:) = min(dists,[],1);
    [candidates_min(:,1), candidate_index] = min(min_dists(i,:),[],2);
    candidates_index(:,1) = candidate_index;
    Distances{1,i} = dists;
    AvgDist(i,1) = sum(dists(:,candidate_index)) / i_period;
end
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
