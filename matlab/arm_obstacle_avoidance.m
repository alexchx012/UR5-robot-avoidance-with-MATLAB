% === [重要] 消息类型同步与生成说明 ===
% 每次修改ROS端的srv/msg文件后，必须在MATLAB命令行执行：
%   rosgenmsg('路径/到/arm_obstacle_avoidance')
% 并重启MATLAB，确保自定义消息类型与ROS端完全一致。
% 否则会导致服务调用死锁或字段缺失！
% =====================================

function arm_obstacle_avoidance()
    % arm_obstacle_avoidance: 使用ROS服务进行基本避障任务。
    % 该脚本通过调用 /request_robot_plan 服务来获取路径规划。

    disp('启动基本避障任务 (基于ROS服务)...');

    % 1. 初始化与ROS的连接和基本设置
    ros_master_uri = connect_to_ros();
    if isempty(ros_master_uri)
        error('arm_obstacle_avoidance: 无法连接到ROS主节点。请确保roscore、Gazebo仿真、planning_service.py 和 matlab_ros_planner.m 正在运行。');
    end

    robot = create_robot_model(false); % false表示不显示图形，避免重复

    jointNames = cell(1, numel(robot.homeConfiguration));
    for i = 1:numel(robot.homeConfiguration)
        jointNames{i} = robot.Bodies{i}.Joint.Name;
    end

    jointStateSub = rossubscriber('/custom_arm/joint_states', 'sensor_msgs/JointState');
    pointCloudSub = rossubscriber('/camera_camera/depth/points', 'sensor_msgs/PointCloud2', 'BufferSize', 1);

    disp('arm_obstacle_avoidance: 正在创建到 /request_robot_plan 服务的客户端...');
    try
        planClient = rossvcclient('/request_robot_plan', 'arm_obstacle_avoidance/RequestPlan', 'Timeout', 30); % 增加服务调用超时
        waitForServer(planClient,'Timeout',15); % 等待服务实际可用
        disp('arm_obstacle_avoidance: 服务客户端已成功创建并连接。');
    catch ME
        disp(ME);
        error('arm_obstacle_avoidance: 创建服务客户端 /request_robot_plan 失败。请检查ROS服务和MATLAB规划节点状态。');
    end

    % 2. 获取初始状态
    disp('arm_obstacle_avoidance: 等待接收初始关节状态...');
    initialJointStateMsg = receive(jointStateSub, 15); % 增加等待时间
    if isempty(initialJointStateMsg)
        error('arm_obstacle_avoidance: 未能接收到初始关节状态。请检查 /custom_arm/joint_states 话题。');
    end

    currentJointValues = zeros(1, length(jointNames));
    % 修复：homeConfiguration返回数值数组，直接使用
    homeConfig = robot.homeConfiguration; % 获取默认配置（数值数组）
    
    % 直接使用数值数组作为配置
    currentConfigStruct = homeConfig; % 数值数组
    
    % 更新关节值
    for i = 1:length(jointNames)
        jointIdx = find(strcmp(initialJointStateMsg.Name, jointNames{i}));
        if ~isempty(jointIdx)
            currentJointValues(i) = initialJointStateMsg.Position(jointIdx);
            currentConfigStruct(i) = initialJointStateMsg.Position(jointIdx); % 直接赋值给数值数组
        else
            disp(['arm_obstacle_avoidance: 警告 - 未找到关节 ', jointNames{i}, ' 的状态，使用默认值。']);
            if i <= length(currentConfigStruct)
                currentJointValues(i) = currentConfigStruct(i); % 使用默认值
            end
        end
    end
    disp(['arm_obstacle_avoidance: 初始关节角度: [', num2str(currentJointValues), ']']);

    currentEeTform_matrix = getTransform(robot, currentConfigStruct, 'tool0');
    current_ee_pos_srv = tform2trvec(currentEeTform_matrix);
    current_ee_quat_srv = tform2quat(currentEeTform_matrix);

    % 3. 定义目标位姿
    goalPosition_target = [0.7, 0.3, 0.2];  % [x, y, z] 米
    goalOrientation_target_quat = [1, 0, 0, 0]; % W, X, Y, Z (默认姿态，根据IK调整)

    % 4. 主尝试循环
    maxAttempts = 3;
    attempt = 1;
    reachedGoal = false;

    while attempt <= maxAttempts && ~reachedGoal
        disp(['arm_obstacle_avoidance: 尝试 ', num2str(attempt), '/', num2str(maxAttempts)]);

        % 4.1 获取最新的点云数据
        disp('arm_obstacle_avoidance: 获取最新的点云数据...');
        latestPointCloudMsg = receive(pointCloudSub, 10); 
        if isempty(latestPointCloudMsg)
            warning('arm_obstacle_avoidance: 未能获取点云数据，跳过此次尝试。');
            attempt = attempt + 1;
            pause(1);
            continue;
        end

        % 4.2 构建服务请求 (完整规划)
        planReq = rosmessage(planClient);
        planReq.ValidateEnvironmentOnly = false; % 请求完整规划
        planReq.CurrentObstacleMap = latestPointCloudMsg;
        planReq.CurrentJointValues = currentJointValues;

        planReq.CurrentEePose = rosmessage('geometry_msgs/PoseStamped');
        planReq.CurrentEePose.Header.FrameId = 'base_link'; 
        planReq.CurrentEePose.Header.Stamp = rostime('now');
        planReq.CurrentEePose.Pose.Position.X = current_ee_pos_srv(1);
        planReq.CurrentEePose.Pose.Position.Y = current_ee_pos_srv(2);
        planReq.CurrentEePose.Pose.Position.Z = current_ee_pos_srv(3);
        planReq.CurrentEePose.Pose.Orientation.W = current_ee_quat_srv(1);
        planReq.CurrentEePose.Pose.Orientation.X = current_ee_quat_srv(2);
        planReq.CurrentEePose.Pose.Orientation.Y = current_ee_quat_srv(3);
        planReq.CurrentEePose.Pose.Orientation.Z = current_ee_quat_srv(4);

        planReq.GoalEePose = rosmessage('geometry_msgs/PoseStamped');
        planReq.GoalEePose.Header.FrameId = 'base_link'; 
        planReq.GoalEePose.Header.Stamp = rostime('now');
        planReq.GoalEePose.Pose.Position.X = goalPosition_target(1);
        planReq.GoalEePose.Pose.Position.Y = goalPosition_target(2);
        planReq.GoalEePose.Pose.Position.Z = goalPosition_target(3);
        planReq.GoalEePose.Pose.Orientation.W = goalOrientation_target_quat(1);
        planReq.GoalEePose.Pose.Orientation.X = goalOrientation_target_quat(2);
        planReq.GoalEePose.Pose.Orientation.Y = goalOrientation_target_quat(3);
        planReq.GoalEePose.Pose.Orientation.Z = goalOrientation_target_quat(4);

        % 4.3 调用服务
        disp('arm_obstacle_avoidance: 发送完整规划请求至服务...');
        try
            % 调试：检查服务请求中的 GoalEePose
            disp('检查服务请求中的 GoalEePose:');
            disp(['  GoalEePose 类型: ', class(planReq.GoalEePose)]);
            if isempty(planReq.GoalEePose)
                disp('  GoalEePose 为空！');
            else
                disp('  GoalEePose 内容:');
                disp(planReq.GoalEePose);
                if isempty(planReq.GoalEePose.Pose)
                    disp('  GoalEePose.Pose 为空！');
                else
                    disp('  GoalEePose.Pose 内容:');
                    disp(planReq.GoalEePose.Pose);
                    if isempty(planReq.GoalEePose.Pose.Position)
                        disp('  GoalEePose.Pose.Position 为空！');
                    else
                        disp(['  GoalEePose.Pose.Position: X=', num2str(planReq.GoalEePose.Pose.Position.X), ...
                                                    ', Y=', num2str(planReq.GoalEePose.Pose.Position.Y), ...
                                                    ', Z=', num2str(planReq.GoalEePose.Pose.Position.Z)]);
                    end
                    if isempty(planReq.GoalEePose.Pose.Orientation)
                        disp('  GoalEePose.Pose.Orientation 为空！');
                    else
                        disp(['  GoalEePose.Pose.Orientation: W=', num2str(planReq.GoalEePose.Pose.Orientation.W), ...
                                                        ', X=', num2str(planReq.GoalEePose.Pose.Orientation.X), ...
                                                        ', Y=', num2str(planReq.GoalEePose.Pose.Orientation.Y), ...
                                                        ', Z=', num2str(planReq.GoalEePose.Pose.Orientation.Z)]);
                    end
                end
            end
            planResp = call(planClient, planReq, 'Timeout', 75); % 规划可能非常耗时
        catch ME_call
            disp(ME_call);
            warning('arm_obstacle_avoidance: 调用规划服务失败或超时。');
            warning('请检查MATLAB端和ROS端的srv/msg类型是否完全一致，且已正确rosgenmsg并重启MATLAB。');
            attempt = attempt + 1;
            pause(2); 
            continue;
        end

        % 4.4 处理服务响应
        if ~isempty(planResp) && planResp.Success
            disp('arm_obstacle_avoidance: 规划服务成功返回路径。');
            trajMsg = planResp.PlannedTrajectory;

            if isempty(trajMsg.JointNames) || isempty(trajMsg.Points)
                warning('arm_obstacle_avoidance: 服务返回的轨迹为空，无法执行。');
                attempt = attempt + 1;
                continue;
            end

            % 4.5 执行轨迹
            disp('arm_obstacle_avoidance: 控制机械臂执行轨迹...');
            control_arm(trajMsg); % control_arm 函数来自章节 5.6

            % 4.6 检查是否到达目标
            disp('arm_obstacle_avoidance: 等待轨迹执行完毕后检查状态...');
            % control_arm 内部有 pause，这里可以再稍等一下确保状态更新
            pause(1.5); 
            finalJointStateMsg = receive(jointStateSub, 10);
            if isempty(finalJointStateMsg)
                warning('arm_obstacle_avoidance: 未能接收到最终关节状态。');
            else
                % 更新 currentJointValues 和 currentConfigStruct 以获取当前EE位姿
                for i = 1:length(jointNames)
                    jointName = jointNames{i};
                     [isMember, idx] = ismember(jointName, finalJointStateMsg.Name);
                    if isMember
                        currentJointValues(i) = finalJointStateMsg.Position(idx);
                        currentConfigStruct(i).JointPosition = finalJointStateMsg.Position(idx);
                    end
                end

                currentEETransform_check = getTransform(robot, currentConfigStruct, 'tool0');
                currentPosition_check = tform2trvec(currentEETransform_check);
                distanceToGoal = norm(currentPosition_check - goalPosition_target);
                disp(['arm_obstacle_avoidance: 当前末端位置: [', num2str(currentPosition_check), '], 距离目标: ', num2str(distanceToGoal), ' 米']);

                goal_tolerance = 0.08; % 目标容差8cm
                if distanceToGoal < goal_tolerance
                    disp(['arm_obstacle_avoidance: 成功到达目标位置 (容差: ', num2str(goal_tolerance), 'm)!']);
                    reachedGoal = true;
                    break; 
                else
                    disp('arm_obstacle_avoidance: 尚未到达目标位置。');
                    % 更新当前EE位姿，为下一次尝试做准备（如果不是因为规划本身失败）
                    current_ee_pos_srv = currentPosition_check;
                    current_ee_quat_srv = tform2quat(currentEETransform_check);
                end
            end
        else % 服务调用不成功或规划本身失败
            if ~isempty(planResp)
                disp(['arm_obstacle_avoidance: 规划失败，服务消息: ', planResp.Message]);
            else
                disp('arm_obstacle_avoidance: 规划服务未返回有效响应。');
            end
        end

        attempt = attempt + 1;
        if ~reachedGoal
            disp('arm_obstacle_avoidance: 等待2秒后重试...');
            pause(2);
        end
    end 

    if ~reachedGoal
        disp('arm_obstacle_avoidance: 达到最大尝试次数，未能到达目标位置。');
    end

    disp('arm_obstacle_avoidance: 基本避障任务结束。');
    % rosshutdown; % 由 main.m 或用户决定何时关闭
end