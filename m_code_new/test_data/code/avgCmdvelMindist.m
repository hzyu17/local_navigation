function [avg_total_dep_noi,avg_total_dep_sem_noi,avg_total_rgb] = ...
    avgCmdvelMindist(cell_avg_dep_noi,cell_avg_dep_sem_noi,cell_avg_rgb,number_of_files,enableDrawBar,flag_sim_real)
%% avg on command velocities and minimum distances
% row 1: avg actual vel
% row 2: avg cmd vel
% row 3: avg min dist
% row 4: avg avg dist
% row 5: avg decrease in vel
% row 6: avg decline percentage of velocity

avg_dep_noi = zeros(5,number_of_files);
avg_dep_sem = zeros(5,number_of_files);
avg_rgb = zeros(5,number_of_files);

for i=1:number_of_files
    % depth image
    avg_dep_noi(1,i) = sum(cell_avg_dep_noi{1,i})/length(cell_avg_dep_noi{1,i});
    avg_dep_noi(2,i) = sum(cell_avg_dep_noi{2,i})/length(cell_avg_dep_noi{2,i});
    avg_dep_noi(3,i) = sum(cell_avg_dep_noi{3,i})/length(cell_avg_dep_noi{3,i});
    avg_dep_noi(4,i) = sum(cell_avg_dep_noi{4,i})/length(cell_avg_dep_noi{4,i});
    avg_dep_noi(5,i) = sum(cell_avg_dep_noi{6,i})/length(cell_avg_dep_noi{6,i});
    % depth_semantic
    avg_dep_sem(1,i) = sum(cell_avg_dep_sem_noi{1,i})/length(cell_avg_dep_sem_noi{1,i});
    avg_dep_sem(2,i) = sum(cell_avg_dep_sem_noi{2,i})/length(cell_avg_dep_sem_noi{2,i});
    avg_dep_sem(3,i) = sum(cell_avg_dep_sem_noi{3,i})/length(cell_avg_dep_sem_noi{3,i});
    avg_dep_sem(4,i) = sum(cell_avg_dep_sem_noi{4,i})/length(cell_avg_dep_sem_noi{4,i});
    avg_dep_sem(5,i) = sum(cell_avg_dep_sem_noi{6,i})/length(cell_avg_dep_sem_noi{6,i});
    % rgb
    if(~isempty(cell_avg_rgb{1,1})) ~=0
        avg_rgb(1,i) = sum(cell_avg_rgb{1,i})/length(cell_avg_rgb{1,i});
        avg_rgb(2,i) = sum(cell_avg_rgb{2,i})/length(cell_avg_rgb{2,i});
        avg_rgb(3,i) = sum(cell_avg_rgb{3,i})/length(cell_avg_rgb{3,i});
        avg_rgb(4,i) = sum(cell_avg_rgb{4,i})/length(cell_avg_rgb{4,i});
        avg_rgb(5,i) = sum(cell_avg_rgb{6,i})/length(cell_avg_rgb{6,i});
    else
        avg_rgb = zeros(4,1);
    end
end
avg_total_dep_noi = sum(avg_dep_noi,2)/number_of_files;
avg_total_dep_sem_noi = sum(avg_dep_sem,2)/number_of_files;
avg_total_rgb = sum(avg_rgb,2)/number_of_files;
if enableDrawBar
   %% bars
    % bar avg 
    figure 
    grid on
    hold on
    if avg_total_rgb == 0
        bar([avg_dep_sem(1,:)',avg_dep_noi(1,:)'])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'));
    else
        bar([avg_dep_sem(1,:)',avg_dep_noi(1,:)',avg_rgb(1,:)'])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'),strrep('avg_rgb','_','\_'));
    end
    title(['average actual velocities ',flag_sim_real])

    figure 
    grid on
    hold on
    if avg_total_rgb == 0
        bar([avg_dep_sem(2,:)',avg_dep_noi(2,:)'])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'));
    else
        bar([avg_dep_sem(2,:)',avg_dep_noi(2,:)',avg_rgb(2,:)'])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'),strrep('avg_rgb','_','\_'));
    end
    title(['average command velocities ',flag_sim_real])

    figure 
    grid on
    hold on
    if avg_total_rgb == 0
        bar([avg_dep_sem(3,:)',avg_dep_noi(3,:)'])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'));
    else
        bar([avg_dep_sem(3,:)',avg_dep_noi(3,:)',avg_rgb(3,:)'])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'),strrep('avg_rgb','_','\_'));
    end
    title(['average minimum ditances ',flag_sim_real])
    
    figure 
    grid on
    hold on
    if avg_total_rgb == 0
        bar([avg_dep_sem(4,:)',avg_dep_noi(4,:)'])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'));
    else
        bar([avg_dep_sem(4,:)',avg_dep_noi(4,:)',avg_rgb(4,:)'])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'),strrep('avg_rgb','_','\_'));
    end
    title(['average average ditances ',flag_sim_real])

    figure 
    grid on
    hold on
    if avg_total_rgb == 0
        bar([avg_dep_sem(5,:)',avg_dep_noi(5,:)'].*100)
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'));
    else
        bar([avg_dep_sem(5,:)',avg_dep_noi(5,:)',avg_rgb(5,:)'].*100)
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'),strrep('avg_rgb','_','\_'));
    end
    title(['average decline percentage on velocity ',flag_sim_real])
    
    figure 
    grid on
    hold on
    avg_avg_decline_percent = [sum(avg_dep_sem(5,:))/length(avg_dep_sem(5,:))', ...
                sum(avg_dep_noi(5,:))/length(avg_dep_noi(5,:))', ...
                sum(avg_rgb(5,:))/length(avg_rgb(5,:))'].*100;
    if avg_total_rgb == 0
        bar([sum(avg_dep_sem(5,:))/length(avg_dep_sem(5,:))',sum(avg_dep_noi(5,:))/length(avg_dep_noi(5,:))'].*100)
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'));
    else
        bar([avg_avg_decline_percent;0,0,0])
        legend(strrep('avg_dep_sem','_','\_'),strrep('avg_dep','_','\_'),strrep('avg_rgb','_','\_'));
    end
    title(['average decline percentage on velocity ',flag_sim_real])
    
    %% error bar
    var_dep_noi = var(avg_dep_noi,0,2);
    var_dep_sem_noi = var(avg_dep_sem,0,2);
    var_rgb = var(avg_rgb,0,2);

    avg_avg_dep_noi = sum(avg_dep_noi,2)/length(avg_dep_noi(1,:));
    avg_avg_dep_sem_noi = sum(avg_dep_sem,2)/length(avg_dep_sem(1,:));
    avg_avg_rgb = sum(avg_rgb,2)/length(avg_rgb(1,:));
    %% 
    figure
    bar([avg_dep_noi(1,:)',avg_dep_sem(1,:)',avg_dep_noi(1,:)'])
    errorbar([avg_avg_dep_noi(1,:),avg_avg_dep_sem_noi(1,:),avg_avg_rgb(1,:)],...
        [var_dep_noi(1,:),var_dep_sem_noi(1,:),avg_avg_rgb(1,:)],'k','LineStyle','none')
    
end

end

