function [newFlagPeopleNear,count, periods] = countMeetPeople(rawFlagPeopleNear)
%countMeetPeople finite state machine to determin the 'real' number of
%times people are near the robot.
breaklen = 0;
delayer = 3;
numberOfPeopleNear = 0;
newFlagPeopleNear = rawFlagPeopleNear;
changeFlag = false;
for i=2:length(rawFlagPeopleNear(:,1))
    rawFlagPeopleNear(i,1) = 0.5 * rawFlagPeopleNear(i,1) + 0.5 * rawFlagPeopleNear(i-1,1);
    if rawFlagPeopleNear(i,1) >= 0.25
        newFlagPeopleNear(i-1:i,1) = 1;
        numberOfPeopleNear = numberOfPeopleNear + 1;
    else
        newFlagPeopleNear(i-1:i,1) = 0;
        rawFlagPeopleNear(i-1:i,1) = 0;
    end
end
threshold = 10;
[count, periods] = Meetings(newFlagPeopleNear, threshold);

% % plot to test the FSM process
% show_arr = zeros(length(newFlagPeopleNear),1);
% for i = 1:length(periods)
%     show_arr(periods(i,1):periods(i,2),1) = 0.5;
% end
% figure
% hold on
% grid minor
% plot(newFlagPeopleNear,'o')
% plot(show_arr, 'Color', 'r')


end

function [count, periods] = Meetings(arr, thres)
%% finite state machine
count = 0;
continuous_one = 0;
change_happened = false;
is_one_period = false;
begin_temp = 1;
periods = [];
for i=2:length(arr)
    if arr(i,1)==1 && arr(i-1,1)==0 % begin of a period of seeing people
        change_happened = true;
        begin_temp = i;
        continuous_one = continuous_one + 1;
        continue;
    end % begin of a period of seeing people
    
    if change_happened % within a period of seeing people
        if arr(i,1)==1 % still seeing
            continuous_one = continuous_one + 1;
            if continuous_one > thres % long period of seeing
                if ~is_one_period
                    is_one_period = true;
                    count = count + 1;
                end                
            end % long period of seeing
            continue;
        else % not seeing people
            continuous_one = 0;
            change_happened = false;
            if is_one_period % end of one continuous period
                periods = [periods; begin_temp, i-1];
                is_one_period = false;
%             else % short break, ignore
%                 continue
            end % end of one continuous period
        end % still seeing
    end % within a period of seeing people
end
end

