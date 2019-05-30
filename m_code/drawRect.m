function h = drawRect(vec,filename,flag_type) 
figure 
hold on
grid minor
h = plot(vec(:,1),vec(:,2),'LineWidth',1 ,'Color','r');
title([flag_type , '_', filename(end-22:end-4)],'Interpreter','none');
rectangle('Position',[-16,-12,32,24],'LineWidth',4,'EdgeColor','b'); 
rectangle('Position',[-14,-9,6,4],'LineWidth',4,'EdgeColor','b'); 
rectangle('Position',[-14,-2,6,5],'LineWidth',4,'EdgeColor','b'); 
rectangle('Position',[-14,6,6,3],'LineWidth',4,'EdgeColor','b'); 
rectangle('Position',[-5,-9,8,4],'LineWidth',4,'EdgeColor','b'); 
rectangle('Position',[-5,-2,8,11],'LineWidth',4,'EdgeColor','b'); 
rectangle('Position',[7,-9,5,4],'LineWidth',4,'EdgeColor','b'); 
rectangle('Position',[7,-2,5,4],'LineWidth',4,'EdgeColor','b'); 
rectangle('Position',[7,4.5,5,4.5],'LineWidth',4,'EdgeColor','b'); 
end %º¯ÊýÎ²
