function trajMsg = generate_trajectory(path, jointNames)
    % 检查输入参数
    if isempty(path) || isempty(path.States)
        error('路径为空，无法生成轨迹');
    end
    
    % 获取路径点
    waypoints = path.States';  % 转置为 numJoints x numPoints
    
    % 生成梯形速度轨迹
    numSamples = max(100, size(waypoints, 2) * 5);  % 轨迹点数量
    [q, qd, qdd, tvec] = trapveltraj(waypoints, numSamples);
    
    % 创建轨迹消息
    trajMsg = rosmessage('trajectory_msgs/JointTrajectory');
    trajMsg.JointNames = jointNames;
    
    % 填充轨迹点
    for i = 1:numSamples
        point = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        point.Positions = q(:,i)';
        point.Velocities = qd(:,i)';
        point.Accelerations = qdd(:,i)';
        point.TimeFromStart = rosduration(tvec(i));
        trajMsg.Points(i) = point;
    end
    
    disp(['生成了包含 ', num2str(numSamples), ' 个点的轨迹']);
end