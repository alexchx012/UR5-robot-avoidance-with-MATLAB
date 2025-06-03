function active_obstacle_avoidance()
    % active_obstacle_avoidance: 实现主动避障，通过ROS服务进行规划、环境验证和重规划。

    disp('启动主动避障任务 (基于ROS服务)...');

    % 1. 初始化
    ros_master_uri = connect_to_ros();
    if isempty(ros_master_uri)
        error('active_obstacle_avoidance: 无法连接到ROS。确保roscore, Gazebo, planning_service.py, matlab_ros_planner.m 运行中。');
    end

    robot = create_robot_model(false); 

    jointNames = cell(1, numel(robot.homeConfiguration));
    for i = 1:numel(robot.homeConfiguration)
        jointNames{i} = robot.Bodies{i}.Joint.Name;
    end

    jointStateSub = rossubscriber('/custom_arm/joint_states', 'sensor_msgs/JointState');
    % 调整 BufferSize 和 monitoringRate
    % 增加BufferSize以减少消息丢失的可能性
    pointCloudSub = rossubscriber('/camera_camera/depth/points', 'sensor_msgs/PointCloud2', 'BufferSize', 5);
    trajPub = rospublisher('/custom_arm/arm_controller/command', 'trajectory_msgs/JointTrajectory');

    disp('active_obstacle_avoidance: 正在创建到 /custom_arm/arm_controller/state 的订阅器...');
    try
        armControllerStateSub = rossubscriber('/custom_arm/arm_controller/state', 'control_msgs/JointTrajectoryControllerState', 'BufferSize', 5);
        disp('active_obstacle_avoidance: 控制器状态订阅器已创建。');
    catch ME_state_sub
        disp(ME_state_sub);
        error('active_obstacle_avoidance: 创建控制器状态订阅器 /custom_arm/arm_controller/state 失败。请检查控制器是否发布此话题。');
    end

    disp('active_obstacle_avoidance: 正在创建 /target_pose 交互式目标点订阅器...');
    try
        targetSub = rossubscriber('/target_pose', 'geometry_msgs/PoseStamped', 'BufferSize', 1);
        disp('active_obstacle_avoidance: /target_pose 订阅器已创建。');
    catch ME_target_sub
        disp(ME_target_sub);
        warning('active_obstacle_avoidance: 创建 /target_pose 订阅器失败。交互式目标点功能将不可用。');
        targetSub = []; % 将 targetSub 置为空，以便后续代码可以检查
    end
    
    disp('active_obstacle_avoidance: 正在创建到 /request_robot_plan 服务的客户端...');
    
    try
        planClient = rossvcclient('/request_robot_plan', 'arm_obstacle_avoidance/RequestPlan', 'Timeout', 30);
        waitForServer(planClient,'Timeout',15);
        disp('active_obstacle_avoidance: 服务客户端已成功创建并连接。');
    catch ME
        disp(ME);
        error('active_obstacle_avoidance: 创建服务客户端失败。');
    end

    disp('active_obstacle_avoidance: 获取初始手臂状态...');
    % 使用 robot.homeConfiguration 作为 prevConfigStruct 的初始值
    [currentJointValues, currentConfigStruct, currentEePoseStruct, initStateSuccess] = ...
        get_current_arm_state_active(jointStateSub, robot, jointNames, robot.homeConfiguration);

    if ~initStateSuccess
        error('active_obstacle_avoidance: 未能成功获取初始手臂状态。请检查ROS话题和机器人状态。');
    end
    current_ee_pos_for_srv = currentEePoseStruct.position;
    current_ee_quat_for_srv = currentEePoseStruct.orientation_quat;
    disp(['active_obstacle_avoidance: 初始关节角度: [', num2str(currentJointValues), ']']);
    disp(['active_obstacle_avoidance: 初始末端执行器位置: [', num2str(current_ee_pos_for_srv), ']']);

    goalPosition_target = [0.7, 0.3, 0.2];
    goalOrientation_target_quat = [1, 0, 0, 0]; 

    monitoringRate = 5;  % Hz (环境监测频率)
    goal_tolerance = 0.08; % 8cm
    % 解决方案 2: 监控关节速度所需的参数
    stop_velocity_threshold_rad_s = 0.02; % 关节速度低于此值视为停止 (rad/s)
    consecutive_stopped_checks_needed = 5; % 需要连续检测到低速的次数
    max_wait_time_for_stop_s = 3.0;    % 等待机械臂停止的最大时间 (秒)

    % 解决方案 1: 轨迹执行超时参数
    trajectory_execution_timeout_factor = 1.8; % 轨迹执行超时因子 (理论时间 * 此因子)
    trajectory_min_execution_timeout_s = 5.0; % 任何轨迹的最小执行超时时间 (秒)

    maxPlannerAttempts = 10; % 总的规划尝试（包括重规划）
    plannerAttempt = 1;
    reachedGoal = false;
    pathExecutionInterrupted = false; % 标记路径执行是否因环境变化而中断

    % 2. 主动避障循环
    while ~reachedGoal && plannerAttempt <= maxPlannerAttempts
    	% 检查并处理来自RViz的交互式目标点
        if ~isempty(targetSub) % 仅当订阅器成功创建时才尝试接收
            targetMsg = receive(targetSub, 0.01); % 非阻塞接收, 短超时 (0.01秒)
            if ~isempty(targetMsg)
                newGoalPosition = [targetMsg.Pose.Position.X, ...
                                   targetMsg.Pose.Position.Y, ...
                                   targetMsg.Pose.Position.Z];
                newGoalOrientationQuat = [targetMsg.Pose.Orientation.W, ...
                                          targetMsg.Pose.Orientation.X, ...
                                          targetMsg.Pose.Orientation.Y, ...
                                          targetMsg.Pose.Orientation.Z];

                % 检查目标是否真的改变了，避免不必要的日志刷新
                if ~isequal(goalPosition_target, newGoalPosition) || ~isequal(goalOrientation_target_quat, newGoalOrientationQuat)
                    goalPosition_target = newGoalPosition;
                    goalOrientation_target_quat = newGoalOrientationQuat;
                    disp(['active_obstacle_avoidance: 收到新的交互式目标点: Pos=[', num2str(goalPosition_target), ...
                          '], Quat=[', num2str(goalOrientation_target_quat), ']']);
                    % 当收到新目标时，可以考虑重置规划尝试计数器或采取其他逻辑
                    % 例如，如果机器人正在移动，可能需要先发送停止指令
                    % 此处简单更新目标，下一次规划将使用新目标
                end
            end
        end
        disp(['===== active_obstacle_avoidance: 规划尝试 ', num2str(plannerAttempt), '/', num2str(maxPlannerAttempts), ' =====']);
        pathExecutionInterrupted = false; % 重置中断标记

        % 2.1 获取当前状态用于规划
        disp('active_obstacle_avoidance: 获取当前点云和关节状态用于规划...');
        latestPointCloudForPlanning = receive(pointCloudSub, 10);
        if isempty(latestPointCloudForPlanning)
            warning('active_obstacle_avoidance: 未能获取点云数据进行规划，跳过此次尝试。');
            plannerAttempt = plannerAttempt + 1; pause(1); continue;
        end

        % 使用 currentConfigStruct (它在循环开始时或上次成功更新后被设置) 作为 prevConfigStruct
        [currentJointValues, currentConfigStruct, currentEePoseStruct, planningStateSuccess] = ...
            get_current_arm_state_active(jointStateSub, robot, jointNames, currentConfigStruct);

        if ~planningStateSuccess
            warning('active_obstacle_avoidance: 未能成功获取用于规划的当前手臂状态，可能使用上次的旧值。');
            % currentJointValues, currentConfigStruct, currentEePoseStruct 仍然包含来自 get_current_arm_state_active 的（可能是旧的）值
        end
        current_ee_pos_for_srv = currentEePoseStruct.position;
        current_ee_quat_for_srv = currentEePoseStruct.orientation_quat;
        disp(['active_obstacle_avoidance: 用于规划的关节角度: [', num2str(currentJointValues), ']']);
        disp(['active_obstacle_avoidance: 用于规划的末端执行器位置: [', num2str(current_ee_pos_for_srv), ']']);

        % 2.2 请求完整路径规划
        planReqFull = rosmessage(planClient);
        planReqFull.ValidateEnvironmentOnly = false;
        planReqFull.CurrentObstacleMap = latestPointCloudForPlanning;
        planReqFull.CurrentJointValues = currentJointValues;
        planReqFull.CurrentEePose = populate_pose_stamped_msg_active('base_link', current_ee_pos_for_srv, current_ee_quat_for_srv);
        planReqFull.GoalEePose = populate_pose_stamped_msg_active('base_link', goalPosition_target, goalOrientation_target_quat);


        disp('active_obstacle_avoidance: 发送完整规划请求...');
        try
            planRespFull = call(planClient, planReqFull, 'Timeout', 75);
        catch ME_call_full
            disp(ME_call_full);
            warning('active_obstacle_avoidance: 调用完整规划服务失败或超时。');
            plannerAttempt = plannerAttempt + 1; pause(2); continue;
        end

        if isempty(planRespFull) || ~planRespFull.Success || isempty(planRespFull.PlannedTrajectory.JointNames) || isempty(planRespFull.PlannedTrajectory.Points)
            if ~isempty(planRespFull)
                disp(['active_obstacle_avoidance: 完整规划失败，服务消息: ', planRespFull.Message]);
            else
                disp('active_obstacle_avoidance: 完整规划服务未返回有效轨迹。');
            end
            plannerAttempt = plannerAttempt + 1; pause(1); continue;
        end

        disp('active_obstacle_avoidance: 完整规划成功，获取到轨迹。');
        activeTrajMsg = planRespFull.PlannedTrajectory;

        % 2.3 执行轨迹并进行环境监测 (基于控制器反馈和超时)
        disp('active_obstacle_avoidance: 开始执行轨迹并监测环境...');
        send(trajPub, activeTrajMsg); % 发送轨迹给控制器

        theoreticalTrajectoryExecutionTime_s = 0;
        if ~isempty(activeTrajMsg.Points)
             theoreticalTrajectoryExecutionTime_s = activeTrajMsg.Points(end).TimeFromStart.Sec + activeTrajMsg.Points(end).TimeFromStart.Nsec*1e-9;
        end
        % 计算实际的执行超时时间
        effectiveTrajectoryTimeout_s = max(trajectory_min_execution_timeout_s, theoreticalTrajectoryExecutionTime_s * trajectory_execution_timeout_factor);
        disp(['active_obstacle_avoidance: 轨迹理论执行时间: ', num2str(theoreticalTrajectoryExecutionTime_s), 's. 监测超时设为: ', num2str(effectiveTrajectoryTimeout_s), 's']);

        trajectoryMonitoringStartTime = tic;
        lastControllerStatusTime = rostime('now'); % 用于判断控制器状态是否在更新

        pathExecutionInterrupted = false; % 确保在此处定义或重置

        while toc(trajectoryMonitoringStartTime) < effectiveTrajectoryTimeout_s
            % --- 检查控制器状态 ---
            controllerStateMsg = armControllerStateSub.LatestMessage; % 非阻塞获取最新状态
            
            trajectoryDone = false; 
            controllerOperational = true;

            if ~isempty(controllerStateMsg)
                % --- 防御性编程：检查控制器状态消息结构 ---
                expectedFields = struct(...
                    'Header', {{'Stamp'}}, ...
                    'Error', {{'Positions'}}, ...
                    'Actual', {{'Velocities', 'Positions'}}, ...
                    'GoalId', {{'Id'}} ...
                );
                
                missingInfo = {}; % 用于存储缺失信息的单元格数组

                if ~isprop(controllerStateMsg, 'Header')
                    missingInfo{end+1} = 'Header';
                elseif ~isprop(controllerStateMsg.Header, 'Stamp')
                    missingInfo{end+1} = 'Header.Stamp';
                end

                if ~isprop(controllerStateMsg, 'Error')
                    missingInfo{end+1} = 'Error';
                elseif ~isprop(controllerStateMsg.Error, 'Positions')
                    missingInfo{end+1} = 'Error.Positions';
                end

                if ~isprop(controllerStateMsg, 'Actual')
                    missingInfo{end+1} = 'Actual';
                else
                    if ~isprop(controllerStateMsg.Actual, 'Velocities')
                        missingInfo{end+1} = 'Actual.Velocities';
                    end
                    if ~isprop(controllerStateMsg.Actual, 'Positions')
                        missingInfo{end+1} = 'Actual.Positions';
                    end
                end
                
                if ~isprop(controllerStateMsg, 'GoalId')
                    missingInfo{end+1} = 'GoalId';
                elseif ~isprop(controllerStateMsg.GoalId, 'Id')
                    missingInfo{end+1} = 'GoalId.Id';
                end

                if ~isempty(missingInfo)
                    errMsg = sprintf('控制器状态消息结构与预期不符。缺失或结构错误的字段: %s。', strjoin(missingInfo, ', '));
                    disp(['错误: active_obstacle_avoidance: [控制器状态] ' errMsg]);
                    error('程序中止: %s', errMsg);
                end
                % --- 消息结构检查完毕 ---

                lastControllerStatusTime = controllerStateMsg.Header.Stamp; % 此处安全，因为 Header.Stamp 已确认存在

                % 判断轨迹是否仍在执行 (基于已验证的字段)：
                if ~isempty(controllerStateMsg.Error.Positions) && ~isempty(controllerStateMsg.Actual.Velocities)
                    joint_errors = abs(controllerStateMsg.Error.Positions);
                    joint_velocities = abs(controllerStateMsg.Actual.Velocities);
                    
                    if all(joint_errors < 0.05) && all(joint_velocities < stop_velocity_threshold_rad_s)
                        if ~isempty(activeTrajMsg.Points)
                            lastTrajPointPositions = activeTrajMsg.Points(end).Positions;
                            currentActualPositions = controllerStateMsg.Actual.Positions;
                            if length(lastTrajPointPositions) == length(currentActualPositions)
                                diff_to_last_point = abs(currentActualPositions - lastTrajPointPositions);
                                if all(diff_to_last_point < goal_tolerance)
                                    disp('active_obstacle_avoidance: [控制器状态] 接近轨迹终点且稳定。');
                                    trajectoryDone = true; 
                                end
                            end
                        else
                             trajectoryDone = true; 
                        end
                    end
                else
                    disp('active_obstacle_avoidance: [控制器状态] Error.Positions 或 Actual.Velocities 为空，可能控制器行为特殊或轨迹刚开始。');
                end
                
                % 检查是否有活动的goal_id (更可靠的判断方式)
                % isprop(controllerStateMsg, 'GoalId') && isprop(controllerStateMsg.GoalId, 'Id') 之前已校验过
                if ~isempty(controllerStateMsg.GoalId.Id)
                    trajectoryDone = false; 
                    disp('active_obstacle_avoidance: [控制器状态] 控制器有活动的Goal ID，仍在执行。');
                elseif isempty(controllerStateMsg.GoalId.Id) && ~trajectoryDone
                    disp('active_obstacle_avoidance: [控制器状态] 控制器无活动Goal ID。');
                    trajectoryDone = true; % 假设完成或中止
                end

            else % 如果长时间未收到控制器状态消息
                 % 确保 lastControllerStatusTime 在此作用域已定义，如果之前从未收到过 controllerStateMsg，
                 % lastControllerStatusTime 可能未初始化。在循环开始前初始化 lastControllerStatusTime = rostime('now');
                if (rostime('now') - lastControllerStatusTime) > rosduration(2.0) 
                    warning('active_obstacle_avoidance: [控制器状态] 长时间未收到控制器状态更新。');
                    controllerOperational = false;
                    trajectoryDone = true; 
                end
            end

            if trajectoryDone || ~controllerOperational
                if controllerOperational
                    disp('active_obstacle_avoidance: 控制器报告轨迹执行完毕或稳定。');
                else
                    disp('active_obstacle_avoidance: 控制器操作异常或长时间无状态更新。');
                end
                break; % 跳出监测循环
            end

            % --- 环境监测逻辑 (在轨迹仍在执行时) ---
            pause(1/monitoringRate); % 控制监测频率

            disp('active_obstacle_avoidance: [监测中] 获取最新点云进行验证...');
            latestPointCloudForValidation = receive(pointCloudSub, 1.0/monitoringRate * 0.8); 

            if ~isempty(latestPointCloudForValidation)
                valReq = rosmessage(planClient);
                valReq.ValidateEnvironmentOnly = true;
                valReq.CurrentObstacleMap = latestPointCloudForValidation;
                
                % --- 获取并校验用于环境验证的当前关节状态 ---
                disp('active_obstacle_avoidance: [监测中] 获取用于验证的当前关节状态...');
                % 使用 currentConfigStruct (它在循环开始时或上次成功更新后被设置) 作为 prevConfigStruct
                [tempCurrentJointValuesVal, tempCurrentConfigStructVal, tempEePoseStructVal, validationStateSuccess] = ...
                    get_current_arm_state_active(jointStateSub, robot, jointNames, currentConfigStruct);

                if ~validationStateSuccess
                    warning('active_obstacle_avoidance: [监测中] 未能成功获取用于环境验证的当前手臂状态。跳过此次验证。');
                    % 在这里决定是 continue; 跳过本次监测循环，还是允许使用可能不完全准确的旧值。
                    % 为安全起见，如果状态获取失败，最好跳过验证或发出更严重的警告。
                    % 此处选择继续，但后续代码需要知道 tempEePoseStructVal 可能不是最新的。
                    % （或者，可以在 get_current_arm_state_active 内部决定是否 error out）
                else
                    disp('active_obstacle_avoidance: [监测中] 成功获取用于验证的手臂状态。');
                end
                % tempCurrentJointValuesVal, tempEePoseStructVal 现在包含了最新的（或尽力获取的）状态

                valReq.CurrentJointValues = tempCurrentJointValuesVal; % 使用从辅助函数获取的值

                % 使用辅助函数填充 CurrentEePose
                valReq.CurrentEePose = populate_pose_stamped_msg_active('base_link', tempEePoseStructVal.position, tempEePoseStructVal.orientation_quat);

                % GoalEePose 保持不变 (使用原规划的目标)
                valReq.GoalEePose = planReqFull.GoalEePose;

                disp('active_obstacle_avoidance: [监测中] 发送环境验证请求...');
                try
                    valResp = call(planClient, valReq, 'Timeout', 20); 
                catch ME_call_val
                    disp(ME_call_val);
                    warning('active_obstacle_avoidance: [监测中] 调用环境验证服务失败或超时。继续执行当前轨迹。');
                    continue; 
                end

                if ~isempty(valResp) && valResp.Success
                    if valResp.EnvironmentHasChangedSignificantly
                        disp('active_obstacle_avoidance: [监测中] 检测到环境显著变化！正在停止机械臂...');
                        pathExecutionInterrupted = true;
                        stopTrajMsg = create_stop_trajectory_active(jointStateSub, jointNames, robot, currentConfigStruct);
						send(trajPub, stopTrajMsg);
                        
                        disp('active_obstacle_avoidance: 等待机械臂完全停止...');
                        consecutive_stopped_count = 0;
                        stop_check_begin_time = tic;
                        arm_confirmed_stopped = false;

                        while toc(stop_check_begin_time) < max_wait_time_for_stop_s
                            currentStopCheckStateMsg = receive(jointStateSub, 0.05); 
                            if ~isempty(currentStopCheckStateMsg) && isprop(currentStopCheckStateMsg, 'Velocity') && ~isempty(currentStopCheckStateMsg.Velocity) && ...
                               isprop(currentStopCheckStateMsg, 'Name') % 确保 Name 字段存在
                                velocities_abs = abs(currentStopCheckStateMsg.Velocity); 
                                relevant_velocities_low = true;
                                for j_idx = 1:length(jointNames)
                                     [isMemberStop, stop_idx] = ismember(jointNames{j_idx}, currentStopCheckStateMsg.Name);
                                     if isMemberStop && stop_idx > 0 && stop_idx <= length(velocities_abs)
                                         if velocities_abs(stop_idx) > stop_velocity_threshold_rad_s
                                             relevant_velocities_low = false;
                                             break;
                                         end
                                     else 
                                         relevant_velocities_low = false;
                                         break;
                                     end
                                end

                                if relevant_velocities_low
                                    consecutive_stopped_count = consecutive_stopped_count + 1;
                                    if consecutive_stopped_count >= consecutive_stopped_checks_needed
                                        disp('active_obstacle_avoidance: 机械臂已确认速度停止。');
                                        arm_confirmed_stopped = true;
                                        break; 
                                    end
                                else
                                    consecutive_stopped_count = 0; 
                                end
                            else
                                consecutive_stopped_count = 0; 
                                if isempty(currentStopCheckStateMsg)
                                    disp('active_obstacle_avoidance: [停止检查] 未收到关节状态。');
                                elseif ~isprop(currentStopCheckStateMsg, 'Velocity') || isempty(currentStopCheckStateMsg.Velocity)
                                     disp('active_obstacle_avoidance: [停止检查] 关节状态消息缺少速度信息。');
                                elseif ~isprop(currentStopCheckStateMsg, 'Name')
                                     disp('active_obstacle_avoidance: [停止检查] 关节状态消息缺少名称信息。');
                                end
                            end
                            pause(0.1); 
                        end

                        if ~arm_confirmed_stopped
                            warning('active_obstacle_avoidance: 等待机械臂速度停止超时或未能确认。');
                        end
                        pause(0.5); 
                        disp('active_obstacle_avoidance: 机械臂已尝试停止。准备重新规划。');
                        break; 
                    else
                        disp('active_obstacle_avoidance: [监测中] 环境稳定，继续执行轨迹。');
                    end
                elseif ~isempty(valResp) && ~valResp.Success % 验证服务调用本身成功，但返回验证操作失败
                    warning('active_obstacle_avoidance: [监测中] 环境验证服务报告操作失败: %s。继续执行当前轨迹。', valResp.Message);
                else % valResp 为空 (超时或严重错误)
                    warning('active_obstacle_avoidance: [监测中] 环境验证服务未返回有效响应。继续执行当前轨迹。');
                end
            else
                disp('active_obstacle_avoidance: [监测中] 未能获取点云进行验证，继续执行。');
            end
        end % end monitoring while loop (toc(trajectoryMonitoringStartTime) < effectiveTrajectoryTimeout_s)

        if toc(trajectoryMonitoringStartTime) >= effectiveTrajectoryTimeout_s && ~pathExecutionInterrupted && ~trajectoryDone
            warning('active_obstacle_avoidance: 轨迹执行监测超时。');
            % 在超时后，也应该尝试停止机械臂，以防失控
            if ~pathExecutionInterrupted % 避免重复发送停止
                disp('active_obstacle_avoidance: 因监测超时，尝试停止机械臂...');
                pathExecutionInterrupted = true; % 标记为中断以触发重规划或结束
                stopTrajMsgOnTimeout = create_stop_trajectory_active(jointStateSub, jointNames, robot, currentConfigStruct);
				send(trajPub, stopTrajMsgOnTimeout);
                pause(1.0); % 等待停止命令生效
            end
        end
        % ... (后续的轨迹执行完毕或被中断后，检查状态逻辑不变) ...

        % 2.4 轨迹执行完毕或被中断后，检查状态
        if pathExecutionInterrupted
            disp('active_obstacle_avoidance: 路径执行被中断，将尝试重新规划。');
            plannerAttempt = plannerAttempt + 1;
            pause(1); % 短暂等待再开始新的规划尝试
            continue; % 返回主循环顶部进行重规划
        end

        disp('active_obstacle_avoidance: 轨迹执行完毕（或未被中断）。检查是否到达目标...');
        pause(1.0); % 确保控制器有时间更新最终状态
        disp('active_obstacle_avoidance: 获取最终手臂状态以检查目标到达情况...');
        [finalJointValues, finalConfigStruct, finalEePoseStruct, finalStateSuccess] = ...
            get_current_arm_state_active(jointStateSub, robot, jointNames, currentConfigStruct); % 使用 currentConfigStruct 作为 prev

        if ~finalStateSuccess
            warning('active_obstacle_avoidance: 未能成功获取最终手臂状态。无法准确判断是否到达目标。');
        else
            currentJointValues = finalJointValues; % 更新 currentJointValues 供下一次迭代起始
            currentConfigStruct = finalConfigStruct; % 更新 currentConfigStruct 供下一次迭代起始

            finalEePos = finalEePoseStruct.position;
            distanceToGoal = norm(finalEePos - goalPosition_target);
            disp(['active_obstacle_avoidance: 最终末端位置: [', num2str(finalEePos), '], 距离目标: ', num2str(distanceToGoal), ' 米']);

            if distanceToGoal < goal_tolerance
                disp(['active_obstacle_avoidance: 成功到达目标位置 (容差: ', num2str(goal_tolerance), 'm)!']);
                reachedGoal = true;
                % break; % 从while循环跳出，如果这是在while循环的末尾，break会生效
            else
                disp('active_obstacle_avoidance: 未到达目标位置。');
            end
        end
        % 如果 reachedGoal 为 true，外部的 while ~reachedGoal 会处理循环退出

        plannerAttempt = plannerAttempt + 1;
        if ~reachedGoal
            disp('active_obstacle_avoidance: 等待1秒后进入下一次规划尝试...');
            pause(1);
        end
    end % end main while loop

    if ~reachedGoal
        disp(['active_obstacle_avoidance: 达到最大规划尝试次数 (', num2str(maxPlannerAttempts), ')，未能到达目标位置。']);
    end

    disp('active_obstacle_avoidance: 主动避障任务结束。');
    % rosshutdown; % 由main.m或用户控制
end

% 辅助函数：创建停止轨迹
% 确保此函数在 active_obstacle_avoidance 内部或MATLAB路径上可以被调用
function stopTrajMsg = create_stop_trajectory_active(jointStateSub_in, jointNamesList_in, robotModel_in, prevConfigStruct_in)
    % jointStateSub_in: ROS订阅器
    % jointNamesList_in: 关节名称
    % robotModel_in: 机器人模型
    % prevConfigStruct_in: 用于获取状态失败时的回退配置

    disp('create_stop_trajectory_active: 获取当前关节状态以生成停止轨迹...');
    [currentStopJointValues, ~, ~, stopStateSuccess] = ...
        get_current_arm_state_active(jointStateSub_in, robotModel_in, jointNamesList_in, prevConfigStruct_in);

    if ~stopStateSuccess
        warning('create_stop_trajectory_active: 获取关节状态失败。停止轨迹可能基于旧值或默认值。');
        % currentStopJointValues 将包含基于 prevConfigStruct_in 的值
    end

    stopTrajMsg = rosmessage('trajectory_msgs/JointTrajectory');
    stopTrajMsg.JointNames = jointNamesList_in;
    point = rosmessage('trajectory_msgs/JointTrajectoryPoint');

    point.Positions = currentStopJointValues; % 使用获取到的或回退的当前位置
    point.Velocities = zeros(1, length(jointNamesList_in)); % 速度和加速度为0
    point.Accelerations = zeros(1, length(jointNamesList_in));
    point.TimeFromStart = rosduration(0.25); % 快速到达当前位置（即停止）,原0.5
    stopTrajMsg.Points = point;
    disp('create_stop_trajectory_active: 停止轨迹已生成。');
end

function [currentJointValues_out, currentConfigStruct_out, currentEePoseStruct_out, success_out] = get_current_arm_state_active(jointStateSub, robotModel, jointNamesList, prevConfigStruct)
    % jointStateSub: ROS订阅器到 /custom_arm/joint_states
    % robotModel: rigidBodyTree模型
    % jointNamesList: 关节名称单元数组
    % prevConfigStruct: 上一个配置结构体，用于在接收失败时提供默认值或部分更新
    
    success_out = false;
    % 使用相对短的超时时间，因为此函数可能在循环中被频繁调用
    jointStateMsg = receive(jointStateSub, 2.0); 
    
    % 初始化输出，基于上一个状态或模型默认
    currentConfigStruct_out = prevConfigStruct; 
    numJoints = length(jointNamesList);
    currentJointValues_out = zeros(1, numJoints);
    for j_idx = 1:numJoints
        currentJointValues_out(j_idx) = prevConfigStruct(j_idx).JointPosition;
    end

    if ~isempty(jointStateMsg)
        if isprop(jointStateMsg, 'Name') && isprop(jointStateMsg, 'Position') && ~isempty(jointStateMsg.Name) && ~isempty(jointStateMsg.Position)
            valid_joints_in_msg = 0;
            for i = 1:numJoints
                jointName = jointNamesList{i};
                [isMember, idx] = ismember(jointName, jointStateMsg.Name);
                if isMember && idx > 0 && idx <= length(jointStateMsg.Position)
                    currentJointValues_out(i) = jointStateMsg.Position(idx);
                    currentConfigStruct_out(i).JointPosition = jointStateMsg.Position(idx);
                    valid_joints_in_msg = valid_joints_in_msg + 1;
                else
                    % 若特定关节未在消息中找到，则保留其在prevConfigStruct中的值
                    currentJointValues_out(i) = prevConfigStruct(i).JointPosition; 
                    currentConfigStruct_out(i).JointPosition = prevConfigStruct(i).JointPosition;
                    warning('get_current_arm_state_active: 关节 %s 在接收到的JointState消息中未找到或索引无效。使用先前值。', jointName);
                end
            end
            
            % 仅当所有预期的关节都在消息中找到并更新时，才认为完全成功
            if valid_joints_in_msg == numJoints
                success_out = true;
            else
                warning('get_current_arm_state_active: 未能在JointState消息中更新所有预期的关节。');
            end
        else
            warning('get_current_arm_state_active: 接收到的JointState消息缺少 Name 或 Position 字段，或它们为空。');
        end
    else
        warning('get_current_arm_state_active: 未能接收到关节状态消息。将使用基于先前配置的值。');
        % currentJointValues_out 和 currentConfigStruct_out 已基于 prevConfigStruct 初始化
    end
    
    % 计算EE位姿
    eeTform_internal = getTransform(robotModel, currentConfigStruct_out, 'tool0');
    currentEePoseStruct_out.position = tform2trvec(eeTform_internal);
    currentEePoseStruct_out.orientation_quat = tform2quat(eeTform_internal); % [w, x, y, z]
end

function poseStampedMsg_out = populate_pose_stamped_msg_active(frameId_in, positionVec_in, orientationQuat_in)
    % frameId_in: 字符串，例如 'base_link'
    % positionVec_in: 1x3 double, [x, y, z]
    % orientationQuat_in: 1x4 double, [w, x, y, z]
    
    poseStampedMsg_out = rosmessage('geometry_msgs/PoseStamped');
    poseStampedMsg_out.Header.FrameId = frameId_in;
    poseStampedMsg_out.Header.Stamp = rostime('now'); % 使用当前ROS时间
    
    poseStampedMsg_out.Pose.Position.X = positionVec_in(1);
    poseStampedMsg_out.Pose.Position.Y = positionVec_in(2);
    poseStampedMsg_out.Pose.Position.Z = positionVec_in(3);
    
    poseStampedMsg_out.Pose.Orientation.W = orientationQuat_in(1);
    poseStampedMsg_out.Pose.Orientation.X = orientationQuat_in(2);
    poseStampedMsg_out.Pose.Orientation.Y = orientationQuat_in(3);
    poseStampedMsg_out.Pose.Orientation.Z = orientationQuat_in(4);
end