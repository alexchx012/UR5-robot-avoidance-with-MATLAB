function control_arm(trajMsg)
    % 创建轨迹发布者
    trajPub = rospublisher('/custom_arm/arm_controller/command', 'trajectory_msgs/JointTrajectory');
    
    % 发送轨迹命令
    disp('发送轨迹命令到机械臂...');
    send(trajPub, trajMsg);
    
    % 等待轨迹执行完成
    % 计算轨迹执行时间
    if ~isempty(trajMsg.Points)
        executionTime = trajMsg.Points(end).TimeFromStart.Sec + ...
                        trajMsg.Points(end).TimeFromStart.Nsec * 1e-9;
        disp(['等待轨迹执行完成，预计时间: ', num2str(executionTime), ' 秒']);
        pause(executionTime + 1);  % 额外等待1秒确保执行完成
        disp('轨迹执行完成');
    else
        warning('轨迹消息中没有点');
    end
end