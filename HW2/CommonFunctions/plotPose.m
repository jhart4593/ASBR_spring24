function plotPose(pose)
    scalingFactor = .05;
    pos = pose(1:3,4);
    xaxis = pos+pose(1:3,1)*scalingFactor; 
    yaxis = pos+pose(1:3,2)*scalingFactor; 
    zaxis = pos+pose(1:3,3)*scalingFactor; 

    plot3(pos(1),pos(2),pos(3),'k*');
    hold on;
    plot3([pos(1) xaxis(1)], [pos(2) xaxis(2)], [pos(3) xaxis(3)],'r','LineWidth',3)
    plot3([pos(1) yaxis(1)], [pos(2) yaxis(2)], [pos(3) yaxis(3)],'g','LineWidth',3)
    plot3([pos(1) zaxis(1)], [pos(2) zaxis(2)], [pos(3) zaxis(3)],'b','LineWidth',3)
    xlabel('x Axis')
    ylabel('y Axis')
    zlabel('z Axis')
    grid on; grid minor;
end