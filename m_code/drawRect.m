function drawRect(vec,filename,flag_type) 
%%drawRec: draw map, trajectory and obstacles
% ---------------- input ------------------%
% vec: robot trajectory
% filename: experiment corresponding file name, serves as the title of...
% the picture
% flag type: rgb/depth/depth_semantic

% ---------------- output ----------------- %
% h: handle of the picture
figure
hold on
grid minor
% draw map
rectangle('Position',[-16, -12, 32, 24],'LineWidth',4,'EdgeColor','k'); 
rectangle('Position',[-14, -9, 6, 4],'LineWidth',4,'EdgeColor','k'); 
rectangle('Position',[-14, -2, 6, 5],'LineWidth',4,'EdgeColor','k'); 
rectangle('Position',[-14, 6, 6, 3],'LineWidth',4,'EdgeColor','k'); 
rectangle('Position',[-5, -9, 8, 4],'LineWidth',4,'EdgeColor','k'); 
rectangle('Position',[-5, -2, 8, 11],'LineWidth',4,'EdgeColor','k'); 
rectangle('Position',[7, -9, 6, 4],'LineWidth',4,'EdgeColor','k'); 
rectangle('Position',[7, -2, 6, 4],'LineWidth',4,'EdgeColor','k'); 
rectangle('Position',[7, 4.5, 6, 4.5],'LineWidth',4,'EdgeColor','k'); 

% draw people in the map
% people standing01 (-4.42, -0.23)
fill([4.42-0.7, 4.42+0.7, 4.42],[-0.23-0.7, -0.23-0.7, -0.23+0.7],'r','facealpha',0.6)
% people standing01 (15, 0)
fill([15.2-0.7, 15.2+0.7, 15.2],[-0.7, -0.7, 0.7],'r','facealpha',0.6)
%actor 01
fill ([-10 -5.2 -5.2 -6.2, -6.2, -10],[-3, -3, 9, 9, -2.2,-2.2],'r','facealpha',0.6);  
%actor 02
fill ([10 3.2 3.2 4.2, 4.2, 10],[3.3, 3.3, 9, 9, 4.3,4.3],'r','facealpha',0.6);  
%actor 03
fill ([6.8 6.8 1 1, 5.8, 5.8],[-7, -3.8, -3.8, -4.8, -4.8, -7],'r','facealpha',0.6);  
%actor 04
fill ([-7.8 -7.8 -6.8 -6.8],[-1.5, 8.5, 8.5, -1.5],'r','facealpha',0.6);  
%actor 05
fill ([7.5 14.2 14.2 13.2 13.2 7.5],[-3.2, -3.2, 5, 5, -2.2, -2.2],'r','facealpha',0.6);  
%actor 06
fill ([-13.5 -2.5 -2.5 -13.5],[11.8 11.8 10.8 10.8],'r','facealpha',0.6);  
%actor 07
fill ([-10 -2 -2 -10],[-9.2 -9.2 -10 -10],'r','facealpha',0.6);  
%actor 08
fill ([2.5 13.5 13.5 2.5],[-11.8 -11.8 -10.8 -10.8],'r','facealpha',0.6);  

% stairs 
fill ([14.5 15.8 15.8 14.5],[-10.8 -10.8 -6.8 -6.8],'g','facealpha',0.4);  
rectangle('Position',[14.5, -10.8, 1.3, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
rectangle('Position',[14.5, -9.8, 1.3, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
rectangle('Position',[14.5, -8.8, 1.3, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
rectangle('Position',[14.5, -7.8, 1.3, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 

% bookshelf (-1.43, 11.4)
fill ([-2.5 -0.5 -0.5 -2.5],[10.8 10.8 11.8 11.8],'g','facealpha',0.4);  
rectangle('Position',[-2.5, 10.8, 2, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
line([-2.5, -0.5],[10.8, 11.8],'LineWidth',0.8); 
line([-2.5, -0.5],[11.8, 10.8],'LineWidth',0.8); 

% bookshelf (9.82, 9.55)
fill ([9 11 11 9],[9.2 9.2 10.2 10.2],'g','facealpha',0.4);  
rectangle('Position',[9, 9.2, 2, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
line([9, 11],[9.2, 10.2],'LineWidth',0.8); 
line([9, 11],[10.2, 9.2],'LineWidth',0.8); 

% bookshelf (-1.7, -4.5)
fill ([-2.7 -0.7 -0.7 -2.7],[-4.8 -4.8 -3.8 -3.8 ],'g','facealpha',0.4);  
rectangle('Position',[-2.7, -4.8, 2, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
line([-2.7 -0.7],[-4.8 -3.8],'LineWidth',0.8); 
line([-2.7 -0.7],[-3.8 -4.8],'LineWidth',0.8); 

% cabinet (3.32808 0.603444)
fill ([3.32808-0.2 4.02808 4.02808 3.32808-0.2],[0.603444-1 0.603444-1 0.603444+1 0.603444+1 ],'g','facealpha',0.4);  
rectangle('Position',[3.32808-0.3, 0.603444-1, 0.9, 2],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
line([3.32808-0.2 4.02808],[0.603444-1 0.603444+1],'LineWidth',0.8); 
line([3.32808-0.2 4.02808],[0.603444+1 0.603444-1],'LineWidth',0.8); 

% cabinet_0 (15.0029 -11.5691)
fill ([14.5 15.8 15.8 14.5],[-11.8 -11.8 -10.8 -10.8],'g','facealpha',0.4);  
rectangle('Position',[14.5, -11.8, 1.3, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
line([14.5 15.8],[-11.8 -10.8],'LineWidth',0.8); 
line([14.5 15.8],[-10.8 -11.8],'LineWidth',0.8); 

% % cafe_table (3.58109 -0.150822)
% fill ([2, 4, 4, 2],[-1 -1 0 0],'g','facealpha',0.4);  
% rectangle('Position',[2, -1, 2, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
% line([2, 4],[-1 0],'LineWidth',0.8); 
% line([2, 4],[0 -1],'LineWidth',0.8); 

% cardboard_box_1_2 (2.06837 11.687)
fill ([1.06 3.06 3.06 1.06],[10.8 10.8 11.8 11.8],'g','facealpha',0.4);  
rectangle('Position',[1.06, 10.8, 2, 1],'LineWidth',0.8,'EdgeColor',[0 0 0]); 
line([1.06 3.06],[10.8 11.8],'LineWidth',0.8); 
line([1.06 3.06],[11.8 10.8],'LineWidth',0.8);

% draw trajectory
plot(vec(:,1),vec(:,2),'--','LineWidth',1 ,'Color','b');
title([flag_type , '_', filename(end-22:end-4)],'Interpreter','none');

end