function [JointFrame,RobotFrame] = getHarmonyZeroPose(dh_table)

n_frames = size(dh_table,1); % Number of rows
dh_table = [dh_table(:,1:2), [dh_table(end,3:4); dh_table(1:end-1,3:4)]]; 
for j = 1:n_frames
    Rx = htm(dh_table(j,4), 'x', 0, 0); 
    Tx = htm(0, 0, dh_table(j,3), 'x'); 
    Rz = htm(dh_table(j,1),'z',0,0); 
    Tz = htm(0, 0, dh_table(j,2), 'z');

    JointFrame.pose{j} = Rx.pose*Tx.pose*Rz.pose*Tz.pose; 
    JointFrame.orientation{j} = JointFrame.pose{j}(1:3,1:3); 
    JointFrame.position{j} = JointFrame.pose{j}(1:3,4);
    
    if j == 1 
        RobotFrame.pose{j} = JointFrame.pose{j}; 
        RobotFrame.orientation{j} = RobotFrame.pose{j}(1:3,1:3); 
        RobotFrame.position{j} = RobotFrame.pose{j}(1:3,4);
    else
        RobotFrame.pose{j} = RobotFrame.pose{j-1}*JointFrame.pose{j}; 
        RobotFrame.orientation{j} = RobotFrame.pose{j}(1:3,1:3); 
        RobotFrame.position{j} = RobotFrame.pose{j}(1:3,4);
    end
    
end

end