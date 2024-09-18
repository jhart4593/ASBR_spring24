function plotOptResults(infoCell, labels)
colors = [0 0.4470 0.7410; 
    0.8500 0.3250 0.0980;
    0.9290 0.6940 0.1250;
    0.6350 0.0780 0.1840;
    0.4660 0.6740 0.1880;
    0 0 0;
    1 0 1];

Markers = {'o', '*', '+', 'square', 'x','^','v'};

% Plot Joint Angles
figure
for i = 1:numel(infoCell)
    info = infoCell{i};
    indx = i; 
    
    subplot(2,2,1)
    jIndx = 1; 
    plot(info.x(:,jIndx),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
    hold on    
    ylabel(labels.yAxisJoint)
    title('Shoulder Elev/Dep')
    grid on; grid minor

    subplot(2,2,2)
    jIndx = 2; 
    plot(info.x(:,jIndx),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
    hold on    
    title('Shoulder Pro/Ret')
    grid on; grid minor

    subplot(2,2,3)
    jIndx = 3; 
    plot(info.x(:,jIndx),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
    hold on    
    ylabel(labels.yAxisJoint)
    xlabel(labels.xAxisJoint)
    title('Shoulder Pro/Ret Virtual')
    grid on; grid minor

    subplot(2,2,4)
    jIndx = 4; 
    plot(info.x(:,jIndx),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
    hold on    
    xlabel(labels.xAxisJoint)
    title('Shoulder Abb/Abd')
    grid on; grid minor
end
subplot(2,2,2)
legend(labels.legend)
figure
for i = 1:numel(infoCell)
    info = infoCell{i};
    indx = i; 
    
    subplot(2,2,1)
    jIndx = 5; 
    plot(info.x(:,jIndx),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
    hold on    
    ylabel(labels.yAxisJoint)
    title('Shoulder Int/Ext Rot')
    grid on; grid minor

    subplot(2,2,2)
    jIndx = 6; 
    plot(info.x(:,jIndx),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
    hold on    
    title('Shoulder Flex/Ext')
    grid on; grid minor

    subplot(2,2,3)
    jIndx = 7; 
    plot(info.x(:,jIndx),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
    hold on    
    ylabel(labels.yAxisJoint)
    xlabel(labels.xAxisJoint)
    title('Elbow Flex/Ext')
    grid on; grid minor

    subplot(2,2,4)
    jIndx = 8; 
    plot(info.x(:,jIndx),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
    hold on    
    xlabel(labels.xAxisJoint)
    title('Wrist Pro/Supp')
    grid on; grid minor
end
subplot(2,2,4)
legend(labels.legend)

% Plot tool tip pos
figure; 
for i = 1:numel(infoCell)
    info = infoCell{i};
    indx = i; 
    toolTipPos = zeros(3,size(info.x,1)); 
    for j = 1:size(info.x,1)
        toolTipPos(:,j) = calculateToolTip(info.SScrews,info.M,info.toolLength,info.x(j,:)'*pi/180);
    end
    subplot(3,1,1)
    plot(j,info.toolTipGoal(1),'s','MarkerSize',10,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor','r')
    hold on  
    plot(toolTipPos(1,:),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
      
    grid on; grid minor
    ylabel('X Position (m)')
    title('Tool Tip Position')

    subplot(3,1,2)
    plot(j,info.toolTipGoal(2),'s','MarkerSize',10,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor','r')
    hold on 
    plot(toolTipPos(2,:),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
   
    grid on; grid minor
    ylabel('Y Position (m)')

    subplot(3,1,3)
    plot(j,info.toolTipGoal(3),'s','MarkerSize',10,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor','r')
    hold on 
    plot(toolTipPos(3,:),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)

    grid on; grid minor
    ylabel('Z Position (m)')
    xlabel('Iteration')
end
subplot(3,1,3)
legend('',labels.legend{1},'',labels.legend{2},'',labels.legend{3},'',labels.legend{4},'',labels.legend{5},'',labels.legend{6})

% Plot tool tip orientation
figure; 
for i = 1:numel(infoCell)
    info = infoCell{i};
    indx = i; 
    toolTipAxis = zeros(3,size(info.x,1)); 
    for j = 1:size(info.x,1)
        [~,toolTipAxis(:,j)]= calculateToolTip(info.SScrews,info.M,info.toolLength,info.x(j,:)'*pi/180);
    end
    subplot(3,1,1)
    plot(j,info.initalOrien(1),'s','MarkerSize',10,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor','r')
    hold on  
    plot(toolTipAxis(1,:),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
      
    grid on; grid minor
    ylabel('X Direction')
    title('Tool Tip Orientation')

    subplot(3,1,2)
    plot(j,info.initalOrien(2),'s','MarkerSize',10,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor','r')
    hold on 
    plot(toolTipAxis(2,:),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)
   
    grid on; grid minor
    ylabel('Y Direction')

    subplot(3,1,3)
    plot(j,info.initalOrien(3),'s','MarkerSize',10,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor','r')
    hold on 
    plot(toolTipAxis(3,:),['-' Markers{indx}],'Color',colors(indx,:),'LineWidth',2)

    grid on; grid minor
    ylabel('Z Direction (m)')
    xlabel('Iteration')
end
subplot(3,1,3)
legend('',labels.legend{1},'',labels.legend{2},'',labels.legend{3},'',labels.legend{4},'',labels.legend{5},'',labels.legend{6})

end