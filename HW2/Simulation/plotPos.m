function plotPos(posx,posy,posz,bounds)
% posx/y/z is the x/y/z coordinates to be plotted
% bounds is 6 element vector of min and max x/y/z values to set axis limits

low = min(bounds);
high = max(bounds);
plot3(posx,posy,posz,'ko-','LineWidth',3)
hold on;
%plot last coordinate in red (end effector)
plot3(posx(numel(posx)),posy(numel(posy)),posz(numel(posz)),'ro-','LineWidth',3)
hold off;
grid on;grid minor;
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
axis([low high low high low high])
drawnow 

end