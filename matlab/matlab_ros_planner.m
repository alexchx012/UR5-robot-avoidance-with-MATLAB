function matlab_ros_planner()
    % --- Persistent 变量声明 ---
    persistent ROBOT_MODEL_SINGLETON;
    persistent ROBOT_MODEL_FOR_ACTIVE_AVOIDANCE;
    persistent matlabPlanningRequestSubscriber;

    % matlab_ros_planner.m
    % 此MATLAB脚本作为一个ROS节点运行，用于接收规划请求，执行路径规划，并发布结果。

    disp('MATLAB规划节点脚本：正在初始化...');

    % --- 全局机器人模型 ---
    if isempty(ROBOT_MODEL_SINGLETON)
        disp('MATLAB规划节点：首次加载机器人模型...');
        try
            % 调用修改后的 create_robot_model 函数，禁用图形输出。
            ROBOT_MODEL_SINGLETON = create_robot_model(false); % 'false' 表示禁用图形
            disp('MATLAB规划节点：机器人模型已成功加载（图形已禁用）。');
            if isempty(ROBOT_MODEL_FOR_ACTIVE_AVOIDANCE) && ~isempty(ROBOT_MODEL_SINGLETON)
                ROBOT_MODEL_FOR_ACTIVE_AVOIDANCE = ROBOT_MODEL_SINGLETON;
                disp('MATLAB规划节点：机器人模型已复制用于主动避障变化检测。');
            end
        catch ME_model_load
            ROBOT_MODEL_SINGLETON = []; % 标记模型加载失败
            disp('MATLAB规划节点 严重错误：加载机器人模型失败。后续规划请求将无法处理。');
            disp(['错误详情: ', ME_model_load.message]);
        end
    end

    % --- 初始化ROS节点 ---
    disp('MATLAB规划节点：正在尝试初始化ROS节点...');
    try
        % 检查ROS Master是否可达以及节点是否已初始化
        % 如果rosinit已经运行并且连接正常，再次调用通常无害或会提示已连接
        rosnode list;
        disp('MATLAB规划节点：ROS节点已成功初始化或已连接。');
    catch ME_ros_init
        disp('MATLAB规划节点 严重错误：ROS节点初始化失败。请确保ROS Master正在运行并且网络配置正确。');
        disp(['错误详情: ', ME_ros_init.message]);
        % 根据实际需求，这里可以选择 return 来终止脚本，或者允许脚本在没有ROS连接的情况下继续（如果部分功能不依赖ROS）
        % 对于此规划器，ROS是核心，所以应该终止
        return; 
    end

    % --- ROS 发布器: 用于发布规划结果 ---
    % 发布到 /matlab_planning/planning_status 主题, 消息类型为 arm_obstacle_avoidance/MatlabPlanningResult
    try
        matlabPlanningResultPublisher = rospublisher('/matlab_planning/planning_status', ...
                                                'arm_obstacle_avoidance/MatlabPlanningResult');
        disp('MATLAB规划节点：已创建结果发布器 /matlab_planning/planning_status。');
    catch ME_publisher_create
        disp('MATLAB规划节点 严重错误：创建结果发布器 /matlab_planning/planning_status 失败。');
        disp('请确保自定义消息类型 arm_obstacle_avoidance/MatlabPlanningResult 已正确生成并在MATLAB中可用，且MATLAB已连接到ROS。');
        disp('如有srv/msg变动，请在MATLAB命令行执行 rosgenmsg 并重启MATLAB。');
        disp(['错误详情: ', ME_publisher_create.message]);
        return; % 如果发布器创建失败，脚本无法正常工作
    end
    % --- ROS 发布器: 用于发布碰撞检查可视化标记 ---
    try
        collisionVizPublisher = rospublisher('/collision_visualization', 'visualization_msgs/MarkerArray');
        disp('MATLAB规划节点：已创建碰撞可视化发布器 /collision_visualization。');
    catch ME_collision_pub_create
        disp('MATLAB规划节点 警告：创建碰撞可视化发布器 /collision_visualization 失败。');
        disp('碰撞检查的可视化将不可用。');
        disp(['错误详情: ', ME_collision_pub_create.message]);
        collisionVizPublisher = []; % 将其置空，以便后续代码可以检查
    end
    % --- ROS 订阅器: 用于接收来自 planning_service.py 的规划请求数据 ---
    % 订阅 /matlab_planning/request_data 主题, 消息类型为 arm_obstacle_avoidance/RequestPlanRequest
    try
        if isempty(matlabPlanningRequestSubscriber)
            matlabPlanningRequestSubscriber = rossubscriber('/matlab_planning/request_data', ...
                                                        'arm_obstacle_avoidance/RequestPlanRequest', ...
                                                        @matlabPlanningRequestCallback); % 指定回调函数名称
            disp('MATLAB规划节点：已创建请求订阅器 /matlab_planning/request_data。');
            disp('MATLAB规划节点正在运行，等待规划请求...');
        end
    catch ME_subscriber_create
        disp('MATLAB规划节点 严重错误：创建请求订阅器 /matlab_planning/request_data 失败。');
        disp('请确保服务消息类型 arm_obstacle_avoidance/RequestPlanRequest 在MATLAB中可用，且MATLAB已连接到ROS。');
        disp('如有srv/msg变动，请在MATLAB命令行执行 rosgenmsg 并重启MATLAB。');
        disp(['错误详情: ', ME_subscriber_create.message]);
        return; % 如果订阅器创建失败，脚本无法正常工作
    end

    % 主循环 (保持脚本运行以接收回调)
    % --- 回调函数定义 ---
    function matlabPlanningRequestCallback(~, rosRequestMsg)
        disp('matlabPlanningRequestCallback已触发');
        % 此回调函数处理来自 /matlab_planning/request_data 主题的规划或验证请求。
        % rosRequestMsg 类型为 arm_obstacle_avoidance/RequestPlanRequest (自动生成)

        % 解决方案A: 将状态变量移入回调函数内部并声明为persistent
        persistent PREVIOUS_OBSTACLES_WITH_IDS_CB; 
        persistent NEXT_OBSTACLE_ID_COUNTER_CB;    

        % 初始化回调函数内部的persistent NEXT_OBSTACLE_ID_COUNTER_CB (如果为空)
        if isempty(NEXT_OBSTACLE_ID_COUNTER_CB)
            NEXT_OBSTACLE_ID_COUNTER_CB = 1;
        end

        % ROBOT_MODEL_SINGLETON 和 ROBOT_MODEL_FOR_ACTIVE_AVOIDANCE 是脚本级 persistent 变量,
        if rosRequestMsg.ValidateEnvironmentOnly
            disp('MATLAB规划节点：收到环境验证请求。');
        else
            disp('MATLAB规划节点：收到完整路径规划请求。');
        end

        rosResultMsg = rosmessage(matlabPlanningResultPublisher); % 类型: MatlabPlanningResult

        % 增强的模型加载检查
        model_error_message = '';
        if isempty(ROBOT_MODEL_SINGLETON)
            model_error_message = 'MATLAB规划器错误：主机器人模型 ROBOT_MODEL_SINGLETON 不可用。';
        elseif rosRequestMsg.ValidateEnvironmentOnly && isempty(ROBOT_MODEL_FOR_ACTIVE_AVOIDANCE)
            model_error_message = 'MATLAB规划器错误：用于环境验证的机器人模型 ROBOT_MODEL_FOR_ACTIVE_AVOIDANCE 不可用。';
        end

        if ~isempty(model_error_message)
            disp(model_error_message);
            rosResultMsg.success = false;
            rosResultMsg.message = model_error_message;
            rosResultMsg.environment_has_changed_significantly = false; % 默认为false，因为无法验证
            try
                rospublish(matlabPlanningResultPublisher, rosResultMsg);
            catch ME_pub_err
                disp(['MATLAB规划节点：发布模型错误状态失败。详情: ', ME_pub_err.message]);
            end
            return;
        end

        try
            currentObstacleMapRosMsg = rosRequestMsg.CurrentObstacleMap;

            % 总是处理点云以获取当前的障碍物原始检测结果 (不含ID)
            disp('MATLAB规划节点：正在处理点云数据以获取原始障碍物...');
            % process_point_cloud 现在返回包含 .collisionObject 和 .centroid 的结构体数组
            [currentRawObstacleStructs, ~] = process_point_cloud(currentObstacleMapRosMsg, ROBOT_MODEL_SINGLETON, 'EnableGraphics', false); 
            disp(['MATLAB规划节点：点云处理完毕，识别到 ', num2str(length(currentRawObstacleStructs)), ' 个原始障碍物结构体。']);

            % 为当前障碍物分配ID (追踪)
            disp('MATLAB规划节点：正在为当前障碍物分配ID (追踪)...');
            % assign_and_track_obstacle_ids 是一个即将定义的新辅助函数
            % 它会更新 NEXT_OBSTACLE_ID_COUNTER
            [currentObstaclesWithAssignedIDs, NEXT_OBSTACLE_ID_COUNTER_CB] = ...
        assign_and_track_obstacle_ids(PREVIOUS_OBSTACLES_WITH_IDS_CB, currentRawObstacleStructs, NEXT_OBSTACLE_ID_COUNTER_CB);
            disp(['MATLAB规划节点：ID分配/追踪完成，当前有效障碍物数量: ', num2str(length(currentObstaclesWithAssignedIDs))]);

            if rosRequestMsg.ValidateEnvironmentOnly
                % --- 执行环境变化验证 ---
                % PREVIOUS_OBSTACLES_WITH_IDS_CB 是上次规划后带有ID的障碍物列表
                % currentObstaclesWithAssignedIDs 是当前帧处理并分配/追踪ID后的障碍物列表
                if isempty(PREVIOUS_OBSTACLES_WITH_IDS_CB) && ~isempty(currentObstaclesWithAssignedIDs)
                    disp('MATLAB规划节点：先前无障碍物记录，但当前检测到障碍物，视为环境变化。');
                    rosResultMsg.environment_has_changed_significantly = true;
                elseif isempty(PREVIOUS_OBSTACLES_WITH_IDS_CB) && isempty(currentObstaclesWithAssignedIDs)
                    disp('MATLAB规划节点：先前无障碍物记录，当前也无障碍物，视为环境未变化。');
                    rosResultMsg.environment_has_changed_significantly = false;
                else % PREVIOUS_OBSTACLES_WITH_IDS_CB 不为空
                    disp('MATLAB规划节点：执行基于ID的障碍物变化检测...');
                    % 定义阈值 (这些可以从外部配置或作为服务参数传入)
                    positionChangeThreshold_m = 0.07;       % 位置变化容忍度 (米)
                    sizeChangeThreshold_m = 0.04;         % 尺寸变化容忍度 (米)
                    orientationChangeThreshold_rad = deg2rad(15); % 方向变化容忍度 (弧度)
                    newObstacleCountThreshold = 1;          % 新出现多少个障碍物算显著变化
                    disappearedObstacleCountThreshold = 1;  % 消失多少个障碍物算显著变化
                    
                    % 调用修改后的 detect_obstacle_changes_planner_version
                    environmentChanged = detect_obstacle_changes_planner_version(...
                                                                PREVIOUS_OBSTACLES_WITH_IDS_CB, ...
                                                                currentObstaclesWithAssignedIDs, ...
                                                                positionChangeThreshold_m, ...
                                                                sizeChangeThreshold_m, ...
                                                                orientationChangeThreshold_rad, ...
                                                                newObstacleCountThreshold, ...
                                                                disappearedObstacleCountThreshold);
                    rosResultMsg.environment_has_changed_significantly = environmentChanged;
                    if environmentChanged
                        disp('MATLAB规划节点：检测到环境发生显著变化。');
                    else
                        disp('MATLAB规划节点：未检测到环境发生显著变化。');
                    end
                end
                rosResultMsg.success = true; % 验证操作本身成功
                rosResultMsg.message = '环境验证完成。';
                % PlannedTrajectory 默认为空
            else
                % --- 执行完整路径规划 ---
                disp('MATLAB规划节点：开始完整路径规划流程...');
                currentJointValuesArray = double(rosRequestMsg.CurrentJointValues);
                goalEePoseRosMsg = rosRequestMsg.GoalEePose;

                % 调试：显示类型和内容
                disp(['goalEePoseRosMsg 类型: ', class(goalEePoseRosMsg)]);
                disp(goalEePoseRosMsg);

                % 如果是cell，取第一个元素
                if iscell(goalEePoseRosMsg)
                    goalEePoseRosMsg = goalEePoseRosMsg{1};
                end

                % 如果是空，报错
                if isempty(goalEePoseRosMsg)
                    error('goalEePoseRosMsg为空，无法获取目标末端位姿。');
                end

                % 如果不是struct也不是对象，报错并输出内容
                if ~(isstruct(goalEePoseRosMsg) || isobject(goalEePoseRosMsg))
                    error(['goalEePoseRosMsg类型异常，无法点操作。实际类型: ', class(goalEePoseRosMsg)]);
                end

                % *** 增加对嵌套字段的详细检查和调试输出 ***
                disp('--- 详细检查 GoalEePose 嵌套字段 ---');
                if isfield(goalEePoseRosMsg, 'Pose') && (isstruct(goalEePoseRosMsg.Pose) || isobject(goalEePoseRosMsg.Pose))
                    disp('  GoalEePose.Pose 字段存在且类型正确。');
                    disp(['  GoalEePose.Pose 类型: ', class(goalEePoseRosMsg.Pose)]);
                    disp(goalEePoseRosMsg.Pose);

                    if isfield(goalEePoseRosMsg.Pose, 'Position') && (isstruct(goalEePoseRosMsg.Pose.Position) || isobject(goalEePoseRosMsg.Pose.Position))
                         disp('    GoalEePose.Pose.Position 字段存在且类型正确。');
                         disp(['    GoalEePose.Pose.Position 类型: ', class(goalEePoseRosMsg.Pose.Position)]);
                         % 尝试显示 Position 的具体数值
                         if isfield(goalEePoseRosMsg.Pose.Position, 'X')
                             disp(['      Position.X: ', num2str(goalEePoseRosMsg.Pose.Position.X)]);
                         end
                          if isfield(goalEePoseRosMsg.Pose.Position, 'Y')
                             disp(['      Position.Y: ', num2str(goalEePoseRosMsg.Pose.Position.Y)]);
                         end
                          if isfield(goalEePoseRosMsg.Pose.Position, 'Z')
                             disp(['      Position.Z: ', num2str(goalEePoseRosMsg.Pose.Position.Z)]);
                         end
                    else
                         disp('    GoalEePose.Pose.Position 字段不存在或类型错误。');
                    end

                    if isfield(goalEePoseRosMsg.Pose, 'Orientation') && (isstruct(goalEePoseRosMsg.Pose.Orientation) || isobject(goalEePoseRosMsg.Pose.Orientation))
                         disp('    GoalEePose.Pose.Orientation 字段存在且类型正确。');
                         disp(['    GoalEePose.Pose.Orientation 类型: ', class(goalEePoseRosMsg.Pose.Orientation)]);
                         % 尝试显示 Orientation 的具体数值
                         if isfield(goalEePoseRosMsg.Pose.Orientation, 'W')
                             disp(['      Orientation.W: ', num2str(goalEePoseRosMsg.Pose.Orientation.W)]);
                         end
                          if isfield(goalEePoseRosMsg.Pose.Orientation, 'X')
                             disp(['      Orientation.X: ', num2str(goalEePoseRosMsg.Pose.Orientation.X)]);
                         end
                          if isfield(goalEePoseRosMsg.Pose.Orientation, 'Y')
                             disp(['      Orientation.Y: ', num2str(goalEePoseRosMsg.Pose.Orientation.Y)]);
                         end
                          if isfield(goalEePoseRosMsg.Pose.Orientation, 'Z')
                             disp(['      Orientation.Z: ', num2str(goalEePoseRosMsg.Pose.Orientation.Z)]);
                         end
                    else
                         disp('    GoalEePose.Pose.Orientation 字段不存在或类型错误。');
                    end

                else
                     disp('  GoalEePose.Pose 字段不存在或类型错误。');
                end
                 disp('--- GoalEePose 详细检查结束 ---');

                % 检查嵌套字段是否存在且有效
                if ~(isfield(goalEePoseRosMsg, 'Pose') && isstruct(goalEePoseRosMsg.Pose) || isobject(goalEePoseRosMsg.Pose)) || ...
                   ~(isfield(goalEePoseRosMsg.Pose, 'Position') && (isstruct(goalEePoseRosMsg.Pose.Position) || isobject(goalEePoseRosMsg.Pose.Position))) || ...
                   ~(isfield(goalEePoseRosMsg.Pose, 'Orientation') && (isstruct(goalEePoseRosMsg.Pose.Orientation) || isobject(goalEePoseRosMsg.Pose.Orientation)))
                    error('goalEePoseRosMsg的嵌套字段（Pose, Position, Orientation）为空或无效。');
                end

                % 现在安全访问
                goalPositionVector = [goalEePoseRosMsg.Pose.Position.X, ...
                                      goalEePoseRosMsg.Pose.Position.Y, ...
                                      goalEePoseRosMsg.Pose.Position.Z];
                goalOrientationQuat = [goalEePoseRosMsg.Pose.Orientation.W, ...
                                       goalEePoseRosMsg.Pose.Orientation.X, ...
                                       goalEePoseRosMsg.Pose.Orientation.Y, ...
                                       goalEePoseRosMsg.Pose.Orientation.Z];

                numRobotJoints = numel(ROBOT_MODEL_SINGLETON.homeConfiguration);
                if length(currentJointValuesArray) ~= numRobotJoints
                    error('MATLAB规划器错误：接收到的current_joint_values数组长度与机器人模型关节数量不匹配。');
                end
                currentConfigVector = currentJointValuesArray(:)';

                disp('MATLAB规划节点：正在使用逆运动学计算目标关节配置...');
                goalTargetTransform = trvec2tform(goalPositionVector);
                goalTargetTransform(1:3, 1:3) = quat2rotm(goalOrientationQuat);

                ikSolverInstance = inverseKinematics('RigidBodyTree', ROBOT_MODEL_SINGLETON);
                ikSolverWeights = [0.25 0.25 0.25 1 1 1];
                initialGuessConfigStruct = ROBOT_MODEL_SINGLETON.homeConfiguration;
                for j_idx = 1:numRobotJoints
                    initialGuessConfigStruct(j_idx).JointPosition = currentConfigVector(j_idx);
                end
                robotEndEffectorName = 'tool0';
                [goalConfigAsStruct, ikSolutionInfo] = ikSolverInstance(robotEndEffectorName, goalTargetTransform, ikSolverWeights, initialGuessConfigStruct);

                if ~strcmpi(ikSolutionInfo.Status, 'success')
                    errorMsgIK = sprintf('MATLAB规划器错误：逆运动学未能找到解。状态: %s。', ikSolutionInfo.Status);
                    disp(errorMsgIK); % 记录错误到控制台
                    rosResultMsg.success = false;
                    rosResultMsg.message = errorMsgIK;
                    % rosResultMsg.planned_trajectory 保持为空
                    rosResultMsg.environment_has_changed_significantly = false; % 规划失败，环境变化状态未评估或不适用
                    try
                        rospublish(matlabPlanningResultPublisher, rosResultMsg);
                    catch ME_pub_err
                        disp(['MATLAB规划节点：发布IK失败状态失败。详情: ', ME_pub_err.message]);
                    end
                    return; % 从回调函数提前返回
                end
                goalConfigVector = zeros(1, numRobotJoints);
                for j_idx = 1:numRobotJoints
                    goalConfigVector(j_idx) = goalConfigAsStruct(j_idx).JointPosition;
                end
                disp('MATLAB规划节点：逆运动学成功。');

                disp('MATLAB规划节点：正在执行RRT路径规划...');
                % 提取 collisionObject 列表给 plan_path
                obstacleCollisionObjectsForPlanner = cell(1, length(currentObstaclesWithAssignedIDs));
                for obs_idx = 1:length(currentObstaclesWithAssignedIDs)
                    obstacleCollisionObjectsForPlanner{obs_idx} = currentObstaclesWithAssignedIDs{obs_idx}.collisionObject;
                end

                disp('MATLAB规划节点：正在执行RRT路径规划 (使用当前识别的障碍物)...');
                [plannedPathRRT, rrtSuccessFlag] = plan_path(ROBOT_MODEL_SINGLETON, currentConfigVector, goalConfigVector, obstacleCollisionObjectsForPlanner, false);

                if ~rrtSuccessFlag
                    errorMsgRRT = 'MATLAB规划器错误：RRT路径规划算法失败。';
                    disp(errorMsgRRT); % 记录错误到控制台
                    rosResultMsg.success = false;
                    rosResultMsg.message = errorMsgRRT;
                    % rosResultMsg.planned_trajectory 保持为空
                    rosResultMsg.environment_has_changed_significantly = false; % 规划失败，环境变化状态未评估或不适用
                    try
                        rospublish(matlabPlanningResultPublisher, rosResultMsg);
                    catch ME_pub_err
                        disp(['MATLAB规划节点：发布RRT失败状态失败。详情: ', ME_pub_err.message]);
                    end
                    return; % 从回调函数提前返回
                end
                disp('MATLAB规划节点：RRT路径规划成功。');

                if ~isempty(plannedPathRRT) && ~isempty(collisionVizPublisher) && ~isempty(ROBOT_MODEL_SINGLETON)
                    disp('MATLAB规划节点：对最终路径上的采样点进行碰撞检查并可视化...');
                    numPointsOnPath = size(plannedPathRRT, 1);
                    % 选择路径上的几个点进行可视化，例如起点、终点和中间几个点
                    % 避免可视化过多点导致RViz卡顿
                    numSamplesToVisualize = min(10, numPointsOnPath); % 最多可视化10个点
                    sampleIndices = round(linspace(1, numPointsOnPath, numSamplesToVisualize));

                    % 获取用于碰撞检查的障碍物列表 (这些是规划时使用的障碍物)
                    % obstacleCollisionObjectsForPlanner 已在前面定义并传递给了 plan_path
                    % 此处再次确认其可用性
                    activeObstacleSetForCheck = {};
                    if exist('obstacleCollisionObjectsForPlanner','var') && iscell(obstacleCollisionObjectsForPlanner)
                        activeObstacleSetForCheck = obstacleCollisionObjectsForPlanner;
                    else
                        disp('MATLAB规划节点 警告: obstacleCollisionObjectsForPlanner 未定义或不是cell数组，无法对路径点进行碰撞可视化。');
                    end

                    if ~isempty(activeObstacleSetForCheck)
                        for k_path_vis = 1:length(sampleIndices)
                            path_idx = sampleIndices(k_path_vis);
                            config_on_path = plannedPathRRT(path_idx, :);

                            % 将行向量转换为 rigidBodyTree 需要的结构体形式
                            currentConfigStructForCheck = ROBOT_MODEL_SINGLETON.homeConfiguration;
                            for j_struct_idx = 1:length(config_on_path)
                                if j_struct_idx <= length(currentConfigStructForCheck)
                                    currentConfigStructForCheck(j_struct_idx).JointPosition = config_on_path(j_struct_idx);
                                end
                            end

                            % 执行实际的碰撞检查
                            % checkCollision(robotModel, configuration, obstaclesCellArray, 'Exhaustive', 'on')
                            % 'Exhaustive','on' 会检查所有连杆对，可能较慢，但更全面
                            % 默认情况下，checkCollision 会检查机器人与给定障碍物的碰撞
                            % 以及机器人自身的自碰撞（如果障碍物列表为空或未提供）
                            % 此处我们检查机器人与环境障碍物的碰撞
                            isPathPointColliding = checkCollision(ROBOT_MODEL_SINGLETON, currentConfigStructForCheck, activeObstacleSetForCheck);
                            % checkCollision 返回一个 NxM 逻辑矩阵，N是机器人连杆数，M是障碍物数量
                            % 如果任何一个元素为 true，则表示该连杆与对应障碍物碰撞
                            % 或者，如果想得到一个单一的碰撞状态（机器人是否与任何障碍物碰撞）：
                            % isPathPointColliding = any(checkCollision(ROBOT_MODEL_SINGLETON, currentConfigStructForCheck, activeObstacleSetForCheck), 'all');
                            % 对于自碰撞: isSelfColliding = any(checkCollision(ROBOT_MODEL_SINGLETON, currentConfigStructForCheck), 'all'); (不传入障碍物)

                            % 这里我们简化为：只要与环境中的任何一个障碍物发生碰撞，就认为是碰撞
                            finalCollisionState = false; % 默认为无碰撞
                            if ~isempty(isPathPointColliding) && islogical(isPathPointColliding)
                                if any(isPathPointColliding(:)) % 检查是否有任何一个碰撞发生
                                    finalCollisionState = true;
                                end
                            end
                            % （可选）加入自碰撞检查
                            % isSelfColliding = any(checkCollision(ROBOT_MODEL_SINGLETON, currentConfigStructForCheck), 'all');
                            % finalCollisionState = finalCollisionState || isSelfColliding;

                            % 调用可视化函数
                            uniqueMarkerId = path_idx + floor(posixtime(datetime('now'))*100); % 保证ID的独特性
                            visualize_single_config_collision_status(ROBOT_MODEL_SINGLETON, config_on_path, finalCollisionState, uniqueMarkerId, collisionVizPublisher);
                            pause(0.05); % 短暂暂停以便RViz有时间渲染
                        end
                        disp('MATLAB规划节点：最终路径点的碰撞状态可视化（部分点）已发送。');
                    end
                end
                % --- 碰撞检查与可视化结束 ---

                disp('MATLAB规划节点：正在生成 trajectory_msgs/JointTrajectory 格式的ROS消息...');
                robotJointNamesList = cell(1, numRobotJoints);
                for j_idx = 1:numRobotJoints
                    robotJointNamesList{j_idx} = ROBOT_MODEL_SINGLETON.Bodies{j_idx}.Joint.Name; % 确保 ROBOT_MODEL_SINGLETON 在此作用域可用
                end
                trajectoryRosMsg = generate_trajectory_from_states(plannedPathRRT, robotJointNamesList);
                disp('MATLAB规划节点：轨迹ROS消息已生成。');

                rosResultMsg.success = true;
                rosResultMsg.message = 'MATLAB规划器：路径已成功规划。';
                rosResultMsg.planned_trajectory = trajectoryRosMsg;
                rosResultMsg.environment_has_changed_significantly = false; % 对于新规划的路径，相对于当前地图，环境自然是"未变化"的

                % 存储当前带有ID的障碍物列表，用于下一次环境变化检测
                PREVIOUS_OBSTACLES_WITH_IDS_CB = currentObstaclesWithAssignedIDs;
                disp('MATLAB规划节点：已更新用于变化检测的带ID障碍物基准列表。');
            end

        catch ME_planning_pipeline
            disp('MATLAB规划节点 在处理流程中发生错误。');
            disp(['错误信息: ', ME_planning_pipeline.message]);
            if ~isempty(ME_planning_pipeline.stack)
                disp('错误栈追踪:');
                for k_err = 1:length(ME_planning_pipeline.stack)
                    disp(ME_planning_pipeline.stack(k_err));
                end
            end
            rosResultMsg.success = false;
            rosResultMsg.message = ['MATLAB规划器错误：', ME_planning_pipeline.message];
            rosResultMsg.environment_has_changed_significantly = false; % 出错时默认为false
        end

        try
            rospublish(matlabPlanningResultPublisher, rosResultMsg);
            if rosResultMsg.success
                disp(['MATLAB规划节点：已成功发布处理结果（状态：成功, 环境变化: ', num2str(rosResultMsg.environment_has_changed_significantly), '）。']);
            else
                disp(['MATLAB规划节点：已发布处理结果（状态：失败）：', rosResultMsg.message]);
            end
        catch ME_publish_response
            disp('MATLAB规划节点 严重错误：发布最终处理结果失败。');
            disp(['错误详情: ', ME_publish_response.message]);
        end
    end

    % --------------------------------------------------------------------
    % 辅助函数: 检测障碍物变化 (基于ID追踪)
    % --------------------------------------------------------------------
    function changed = detect_obstacle_changes_planner_version(oldObsWithIDs, newObsWithIDs, ...
                                                            position_thresh_m, size_thresh_m, orientation_thresh_rad, ...
                                                            new_obstacle_count_trigger, disappeared_obstacle_count_trigger)
        % oldObsWithIDs, newObsWithIDs: 单元数组，每个元素是 {id, collisionObject, centroid}
        % *_thresh_*: 各种变化的阈值
        % new_obstacle_count_trigger: 新出现多少个障碍物就认为环境显著变化
        % disappeared_obstacle_count_trigger: 消失多少个障碍物就认为环境显著变化

        changed = false;

        if isempty(oldObsWithIDs) && isempty(newObsWithIDs)
            disp('detect_changes_id: 无旧障碍物也无新障碍物，环境未变。');
            changed = false;
            return;
        end
        
        if isempty(oldObsWithIDs) && ~isempty(newObsWithIDs)
            if length(newObsWithIDs) >= new_obstacle_count_trigger
                disp(['detect_changes_id: 原无障碍物，现出现 ', num2str(length(newObsWithIDs)), ' 个，超过阈值 ', num2str(new_obstacle_count_trigger), '。环境变化。']);
                changed = true;
            else
                disp(['detect_changes_id: 原无障碍物，现出现 ', num2str(length(newObsWithIDs)), ' 个，未超阈值。环境未变。']);
                changed = false;
            end
            return;
        end

        if ~isempty(oldObsWithIDs) && isempty(newObsWithIDs)
            if length(oldObsWithIDs) >= disappeared_obstacle_count_trigger
                disp(['detect_changes_id: 原有障碍物 ', num2str(length(oldObsWithIDs)), ' 个，现全部消失，超过阈值 ', num2str(disappeared_obstacle_count_trigger), '。环境变化。']);
                changed = true;
            else
                disp(['detect_changes_id: 原有障碍物 ', num2str(length(oldObsWithIDs)), ' 个，现全部消失，未超阈值。环境未变。']);
                changed = false;
            end
            return;
        end

        % 创建ID到索引的映射，方便查找
        oldMap = containers.Map('KeyType','double','ValueType','any');
        for i = 1:length(oldObsWithIDs)
            oldMap(oldObsWithIDs{i}.id) = oldObsWithIDs{i};
        end

        newMap = containers.Map('KeyType','double','ValueType','any');
        for i = 1:length(newObsWithIDs)
            newMap(newObsWithIDs{i}.id) = newObsWithIDs{i};
        end

        allOldIDs = cellfun(@(c) c.id, oldObsWithIDs);
        allNewIDs = cellfun(@(c) c.id, newObsWithIDs);

        commonIDs = intersect(allOldIDs, allNewIDs);
        newlyAppearedIDs = setdiff(allNewIDs, allOldIDs);
        disappearedIDs = setdiff(allOldIDs, allNewIDs);

        % 检查共同障碍物的变化
        for i = 1:length(commonIDs)
            id = commonIDs(i);
            oldObsStruct = oldMap(id);
            newObsStruct = newMap(id);
            
            oldCo = oldObsStruct.collisionObject;
            newCo = newObsStruct.collisionObject;

            % 1. 比较类型 (如果类型可以变化的话，目前都是collisionBox)
            if ~strcmp(class(oldCo), class(newCo))
                disp(['detect_changes_id: ID ', num2str(id), ' 类型从 ', class(oldCo), ' 变为 ', class(newCo), '。环境变化。']);
                changed = true; return;
            end

            % 2. 比较位置 (基于质心，因为Pose可能是相对于物体几何中心的)
            pos_diff = norm(oldObsStruct.centroid - newObsStruct.centroid);
            if pos_diff > position_thresh_m
                disp(['detect_changes_id: ID ', num2str(id), ' 位置变化 (', num2str(pos_diff*1000, '%.1f'), 'mm) > 阈值 (', num2str(position_thresh_m*1000, '%.1f'), 'mm)。环境变化。']);
                changed = true; return;
            end

            % 3. 比较尺寸 (仅针对 collisionBox)
            if isa(oldCo, 'collisionBox') && isa(newCo, 'collisionBox')
                oldDims = [oldCo.X, oldCo.Y, oldCo.Z];
                newDims = [newCo.X, newCo.Y, newCo.Z];
                % 比较每个维度或者总体积/范数等
                if any(abs(oldDims - newDims) > size_thresh_m) % 如果任何一个维度变化超过阈值
                    size_diff_norm = norm(oldDims - newDims); % 或者用范数比较
                    disp(['detect_changes_id: ID ', num2str(id), ' Box尺寸变化 (范数 ', num2str(size_diff_norm, '%.3f'),'m) > 阈值 (', num2str(size_thresh_m, '%.3f'),'m)。环境变化。']);
                    changed = true; return;
                end
            % 可以在此添加对 collisionCylinder, collisionSphere 的尺寸比较
            elseif isa(oldCo, 'collisionCylinder') && isa(newCo, 'collisionCylinder')
                if abs(oldCo.Radius - newCo.Radius) > size_thresh_m || abs(oldCo.Length - newCo.Length) > size_thresh_m
                    disp(['detect_changes_id: ID ', num2str(id), ' Cylinder尺寸变化 > 阈值。环境变化。']);
                    changed = true; return;
                end
            elseif isa(oldCo, 'collisionSphere') && isa(newCo, 'collisionSphere')
                if abs(oldCo.Radius - newCo.Radius) > size_thresh_m
                    disp(['detect_changes_id: ID ', num2str(id), ' Sphere尺寸变化 > 阈值。环境变化。']);
                    changed = true; return;
                end
            end
            
            % 4. 比较方向 (从Pose矩阵中提取旋转部分)
            oldRotm = oldCo.Pose(1:3, 1:3); 
            newRotm = newCo.Pose(1:3, 1:3);
            % 计算两个旋转矩阵之间的角度差 (轴角表示法的角度)
            relativeRotm = newRotm * oldRotm'; % R_new * R_old_transpose
            [~, angle_diff_rad_val] = rotm2axang(relativeRotm); % 返回 [axis, angle]
            
            % angle_diff_rad_val 可能为负，取绝对值比较
            if abs(angle_diff_rad_val) > orientation_thresh_rad 
                disp(['detect_changes_id: ID ', num2str(id), ' 方向变化 (', num2str(rad2deg(abs(angle_diff_rad_val)), '%.1f'), 'deg) > 阈值 (', num2str(rad2deg(orientation_thresh_rad), '%.1f'), 'deg)。环境变化。']);
                changed = true; return;
            end
        end

        % 检查新出现的障碍物数量
        if length(newlyAppearedIDs) >= new_obstacle_count_trigger
            disp(['detect_changes_id: 新出现障碍物数量 (', num2str(length(newlyAppearedIDs)), ') >= 阈值 (', num2str(new_obstacle_count_trigger), ')。环境变化。']);
            changed = true; return;
        end

        % 检查消失的障碍物数量
        if length(disappearedIDs) >= disappeared_obstacle_count_trigger
            disp(['detect_changes_id: 消失障碍物数量 (', num2str(length(disappearedIDs)), ') >= 阈值 (', num2str(disappeared_obstacle_count_trigger), ')。环境变化。']);
            changed = true; return;
        end
        
        % 如果以上都没有触发变化，则认为环境未显著变化
        if ~changed
            disp('detect_changes_id: 未检测到显著的环境变化。');
        end
    end


    function trajMsg = generate_trajectory_from_states(pathStates, jointNames)
        % pathStates: 路径点矩阵，每行是一个关节配置
        % jointNames: 关节名称单元数组

        if isempty(pathStates)
            error('路径状态为空，无法生成轨迹');
        end

        waypoints = pathStates';  % 转置为 numJoints x numPoints

        numSamples = max(100, size(waypoints, 2) * 5);
        if size(waypoints, 2) == 1 % 如果只有一个路径点 (例如，IK结果直接是目标)
            % 创建一个只有单点的轨迹，或者根据需求复制该点作为起点和终点
            q = waypoints;
            qd = zeros(size(waypoints));
            qdd = zeros(size(waypoints));
            tvec = [0, 0.1]; % 短时间
            q = [q,q]; % 复制点
            qd = [qd,qd];
            qdd = [qdd,qdd];
            numSamples = 2;
        else
            [q, qd, qdd, tvec] = trapveltraj(waypoints, numSamples);
        end

        trajMsg = rosmessage('trajectory_msgs/JointTrajectory');
        trajMsg.JointNames = jointNames;

        for i_traj = 1:numSamples
            point = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            point.Positions = q(:,i_traj)';
            point.Velocities = qd(:,i_traj)';
            point.Accelerations = qdd(:,i_traj)';
            point.TimeFromStart = rosduration(tvec(i_traj));
            trajMsg.Points(i_traj) = point;
        end

        disp(['生成了包含 ', num2str(numSamples), ' 个点的轨迹']);
    end

    % --------------------------------------------------------------------
    % 辅助函数: 分配并追踪障碍物ID (基于卡尔曼滤波和匈牙利算法的鲁棒追踪)
    % --------------------------------------------------------------------
    function [currentTracks, updatedNextID] = assign_and_track_obstacle_ids(prevTracks, currentRawObstacleStructs, nextIDSeed, dt, frameCount)
        % assign_and_track_obstacle_ids: 对检测到的障碍物进行追踪和ID分配。
        %
        % 输入:
        %   prevTracks (struct array): 上一帧的轨迹结构体数组。每个结构体包含:
        %                               .id (double) - 轨迹ID
        %                               .kf (kalmanFilter) - 卡尔曼滤波器对象
        %                               .age (double) - 轨迹的年龄 (帧数)
        %                               .totalVisibleCount (double) - 总共被看到的次数
        %                               .consecutiveInvisibleCount (double) - 连续未被看到的次数
        %                               .dims (1x3 double) - [L, W, H] for collisionBox
        %                               .orientation (3x3 double) - 旋转矩阵
        %                               .collisionObject (collisionShape) - 障碍物的碰撞体
        %                               .lastDetectionCentroid (1x3 double) - 上次检测到的质心
        %   currentRawObstacleStructs (cell array): 当前帧原始检测到的障碍物结构体单元数组。
        %                                      每个单元是 {collisionObject, centroid}。
        %   nextIDSeed (double): 用于生成新轨迹ID的起始计数器。
        %   dt (double): 自上一帧以来的时间间隔 (秒)。
        %   frameCount (double): 当前处理的帧计数或时间戳，用于调试或特定逻辑。
        %
        % 输出:
        %   currentTracks (struct array): 当前帧更新后的轨迹结构体数组。
        %   updatedNextID (double): 更新后的轨迹ID计数器。

        % --- 初始化和追踪参数 ---
        updatedNextID = nextIDSeed;
        numPrevTracks = length(prevTracks);
        numDetections = length(currentRawObstacleStructs);

        % 轨迹生命周期管理参数 (这些参数的调优见后续说明章节)
        gateThresholdMahalanobis = 2.5; % 马氏距离门控阈值 (基于卡方分布，例如2.5对应约95%置信区间对于2D)
        costOfNonAssignment = 10;       % 未分配的代价，用于matchpairs函数
        invisibleForTooLong = 3;        % 轨迹连续多少帧未匹配到检测则删除 (帧数)
        ageThresholdForDeletion = 1;    % 对于年龄较小的轨迹，如果很快就丢失，可以更快删除
        minHitsForReliableTrack = 3;    % 至少需要多少次成功匹配才认为是一个比较可靠的轨迹

        % 代价函数权重 (这些参数的调优见后续说明章节)
        w_pos_normalized = 0.6; % 位置代价权重 (马氏距离已包含不确定性，此处权重针对组合)
        w_size = 0.2;           % 尺寸代价权重
        w_orient = 0.2;         % 方向代价权重

        currentTracks = []; % 初始化当前帧的轨迹列表

        % --- 步骤 1: 对已存在轨迹进行状态预测 (卡尔曼滤波器) ---
        if numPrevTracks > 0
            for i = 1:numPrevTracks
                % 预测轨迹i在当前时刻的状态
                % predict函数会更新 prevTracks(i).kf 内部的状态和协方差
                prevTracks(i).predictedState = predict(prevTracks(i).kf, dt);
                prevTracks(i).predictedCovariance = prevTracks(i).kf.StateCovariance; % 获取预测后的协方差
            end
        end

        % --- 步骤 2: 构建代价矩阵用于匈牙利算法 ---
        costMatrix = Inf(numPrevTracks, numDetections); % 初始化代价矩阵

        if numPrevTracks > 0 && numDetections > 0
            for i = 1:numPrevTracks % 遍历每一个旧轨迹
                track = prevTracks(i);
                predictedCentroid = track.predictedState(1:3)'; % KF状态的前3个元素是位置 [cx, cy, cz]

                for j = 1:numDetections % 遍历每一个新检测
                    detection = currentRawObstacleStructs{j};
                    measuredCentroid = detection.centroid;
                    measuredCollisionObject = detection.collisionObject;

                    % 2a. 门控: 使用马氏距离进行初步筛选
                    % S = H * P_pred * H' + R
                    % H = [eye(3) zeros(3,3)]; % 测量矩阵，因为我们只测量位置
                    % R = track.kf.MeasurementNoise; % 从KF对象获取测量噪声协方差
                    % innovationCov = H * track.predictedCovariance * H' + R;
                    % mahalanobisDist = sqrt(mahal(measuredCentroid, predictedCentroid, innovationCov));
                    % 上述mahal函数直接用可能会有问题，我们手动计算平方马氏距离
                    measurementResidual = measuredCentroid - predictedCentroid;
                    H_matrix = [eye(3) zeros(3,3)]; % 测量矩阵
                    residualCovariance = H_matrix * track.predictedCovariance * H_matrix' + track.kf.MeasurementNoise;
                    
                    % 检查 residualCovariance 是否正定且可逆
                    if det(residualCovariance) <= 1e-9 % 或者用chol分解检查
                        % disp(['警告: 轨迹 ', num2str(track.id), ' 的残差协方差矩阵奇异或接近奇异，跳过此检测对。']);
                        mahalanobisDistSq = Inf; % 若奇异，则代价无穷大
                    else
                        mahalanobisDistSq = measurementResidual * (residualCovariance \ measurementResidual'); % 平方马氏距离
                    end


                    if mahalanobisDistSq > gateThresholdMahalanobis^2 % 使用平方进行比较，避免开方
                        costMatrix(i,j) = Inf; % 超出验证门，代价设为无穷大
                        continue;
                    end

                    % 2b. 计算详细代价 (对于门内的候选对)
                    cost_pos = sqrt(mahalanobisDistSq); % 使用马氏距离作为位置代价的一部分

                    % 提取尺寸和方向用于计算代价 (需要辅助函数)
                    [trackDims, trackOrientationMatrix] = extractGeometricFeatures(track.collisionObject); % 使用上一帧的实际检测物体
                    [detectionDims, detectionOrientationMatrix] = extractGeometricFeatures(measuredCollisionObject);

                    % 尺寸代价 (归一化体积差异或维度差异)
                    if isempty(trackDims) || isempty(detectionDims) || numel(trackDims) ~= numel(detectionDims)
                        cost_size = 1.0; % 如果无法比较尺寸，则惩罚较大 (0到1范围)
                    else
                        % 示例：归一化L2范数差异
                        size_diff_norm = norm(trackDims - detectionDims);
                        size_mean_norm = norm((trackDims + detectionDims)/2);
                        if size_mean_norm < 1e-3, size_mean_norm = 1e-3; end %避免除零
                        cost_size = min(1.0, size_diff_norm / size_mean_norm); % 限制在[0,1]
                    end

                    % 方向代价 (0到1范围, 0表示方向一致, 1表示方向差异最大)
                    if isempty(trackOrientationMatrix) || isempty(detectionOrientationMatrix)
                        cost_orient = 1.0; % 如果任一方向无效
                    else
                        R_relative = detectionOrientationMatrix * trackOrientationMatrix';
                        [~, angle_diff_rad] = rotm2axang(R_relative);
                        cost_orient = abs(angle_diff_rad) / pi; % 归一化到[0,1] (0到pi的角度差)
                    end
                    
                    % 总代价: 加权和
                    % 注意：马氏距离本身是有统计意义的，直接加权可能不完全严谨，
                    % 但在实际工程中常作为一种组合方式。
                    % 此处将马氏距离视为一种归一化的位置差异指标。
                    normalized_pos_cost = min(1.0, cost_pos / gateThresholdMahalanobis); % 将马氏距离也归一化到[0,1]附近
                    
                    costMatrix(i,j) = w_pos_normalized * normalized_pos_cost + ...
                                    w_size * cost_size + ...
                                    w_orient * cost_orient;
                end
            end
        end

        % --- 步骤 3: 使用匈牙利算法 (matchpairs) 进行分配 ---
        if numPrevTracks > 0 && numDetections > 0
            % matchpairs(CostMatrix, CostOfNonAssignment, 'max true for max cost, false for min cost');
            % 我们希望最小化代价，所以第三个参数应该是 'min' (matchpairs 默认行为) 或不指定
            % matchpairs 的输出:
            %   matches: P-by-2 matrix, P是匹配对的数量。每行 [trackIdx, detectionIdx]
            %   unassignedTracks: 未匹配上的轨迹索引
            %   unassignedDetections: 未匹配上的检测索引
            [matches, unassignedTracksIdx, unassignedDetectionsIdx] = matchpairs(costMatrix, costOfNonAssignment);
        else % 如果没有轨迹或没有检测
            matches = zeros(0,2); % 空匹配
            unassignedTracksIdx = (1:numPrevTracks)';
            unassignedDetectionsIdx = (1:numDetections)';
        end

        % --- 步骤 4: 更新匹配上的轨迹 ---
        matchedTrackIDs = [];
        for k = 1:size(matches, 1)
            trackIdx = matches(k,1);
            detectionIdx = matches(k,2);

            trackToUpdate = prevTracks(trackIdx);
            detectionForUpdate = currentRawObstacleStructs{detectionIdx};
            
            % 使用检测到的质心更新KF
            measuredCentroid = detectionForUpdate.centroid;
            % correct 函数会更新 trackToUpdate.kf 内部的状态和协方差
            trackToUpdate.correctedState = correct(trackToUpdate.kf, measuredCentroid', dt); % measurement z needs to be column vector
            
            % 更新轨迹的其他属性
            trackToUpdate.age = trackToUpdate.age + 1;
            trackToUpdate.totalVisibleCount = trackToUpdate.totalVisibleCount + 1;
            trackToUpdate.consecutiveInvisibleCount = 0; % 重置不可见计数
            trackToUpdate.collisionObject = detectionForUpdate.collisionObject; % 更新为最新的碰撞体
            [trackToUpdate.dims, trackToUpdate.orientation] = extractGeometricFeatures(trackToUpdate.collisionObject);
            trackToUpdate.lastDetectionCentroid = measuredCentroid;

            currentTracks = [currentTracks; trackToUpdate];
            matchedTrackIDs = [matchedTrackIDs; trackToUpdate.id];
        end

        % --- 步骤 5: 处理未匹配的轨迹 (可能删除) ---
        for k = 1:length(unassignedTracksIdx)
            trackIdx = unassignedTracksIdx(k);
            trackToManage = prevTracks(trackIdx);

            trackToManage.consecutiveInvisibleCount = trackToManage.consecutiveInvisibleCount + 1;
            trackToManage.age = trackToManage.age + 1; % 年龄继续增长

            % 删除逻辑
            % 过于老旧且长时间不可见，或者非常年轻但很快就丢失
            shouldDelete = false;
            if trackToManage.consecutiveInvisibleCount > invisibleForTooLong
                shouldDelete = true;
                disp(['MATLAB规划节点(追踪): 轨迹 ', num2str(trackToManage.id), ' 因长时间(',num2str(trackToManage.consecutiveInvisibleCount),')不可见而被删除。年龄: ', num2str(trackToManage.age), ', 总可见: ', num2str(trackToManage.totalVisibleCount)]);
            end
            % （可选）对于不够可靠的轨迹，如果丢失也较快删除
            if trackToManage.totalVisibleCount < minHitsForReliableTrack && trackToManage.consecutiveInvisibleCount > ageThresholdForDeletion
                shouldDelete = true;
                disp(['MATLAB规划节点(追踪): 年轻轨迹 ', num2str(trackToManage.id), ' 因很快丢失而被删除。年龄: ', num2str(trackToManage.age), ', 总可见: ', num2str(trackToManage.totalVisibleCount)]);
            end


            if ~shouldDelete
                % 如果不删除，则保留该轨迹，其状态是基于预测的
                % KF对象内部已经通过predict更新了状态和协方差
                trackToManage.correctedState = trackToManage.predictedState; % 将预测状态视为当前状态
                % collisionObject, dims, orientation 保持上一帧成功检测时的值
                currentTracks = [currentTracks; trackToManage];
            end
        end

        % --- 步骤 6: 初始化新的轨迹 (来自未匹配的检测) ---
        for k = 1:length(unassignedDetectionsIdx)
            detectionIdx = unassignedDetectionsIdx(k);
            newDetection = currentRawObstacleStructs{detectionIdx};

            % 创建新的轨迹结构体
            newTrack.id = updatedNextID;
            updatedNextID = updatedNextID + 1;

            % 初始化卡尔曼滤波器
            % 状态向量: [cx, cy, cz, vx, vy, vz]'
            % 初始位置来自检测，初始速度设为0 (或基于最近几帧粗略估计，此处简化为0)
            initialState = [newDetection.centroid(1); newDetection.centroid(2); newDetection.centroid(3); 0; 0; 0];
            
            % 定义初始状态协方差 P0 (具体数值见调优章节)
            % 通常位置不确定性来自检测噪声，速度不确定性初始较大
            initialStateCovariance = diag([0.1^2, 0.1^2, 0.1^2, 0.5^2, 0.5^2, 0.5^2]); % 例: 位置标准差0.1m, 速度标准差0.5m/s

            % 定义过程噪声协方差 Q (具体数值见调优章节)
            % Q = G * G' * sigma_acc^2, G = [dt^2/2; dt^2/2; dt^2/2; dt; dt; dt]
            % 更简单的方式是直接定义对角阵，主要影响速度分量
            sigma_pos_proc = 0.05; % 过程噪声中位置的标准差 (m) - 体现模型未建模的随机位移
            sigma_vel_proc = 0.2;  % 过程噪声中速度的标准差 (m/s) - 体现模型未建模的加速度
            processNoiseCov = diag([sigma_pos_proc^2, sigma_pos_proc^2, sigma_pos_proc^2, ...
                                    sigma_vel_proc^2, sigma_vel_proc^2, sigma_vel_proc^2]);
            
            % 定义测量噪声协方差 R (具体数值见调优章节)
            % 假设测量的是[cx, cy, cz]
            sigma_meas_pos = 0.05; % 传感器对质心位置测量的标准差 (m)
            measurementNoiseCov = diag([sigma_meas_pos^2, sigma_meas_pos^2, sigma_meas_pos^2]);

            newTrack.kf = kalmanFilter(@cvkf_motionModel, @cvkf_measurementModel, initialState, ...
                                    'StateCovariance', initialStateCovariance, ...
                                    'ProcessNoise', processNoiseCov, ...
                                    'MeasurementNoise', measurementNoiseCov, ...
                                    'StateTransitionFcn', @(x, dt_kf) cvkf_stateTransition(x, dt_kf, frameCount), ... % 传递dt和frameCount
                                    'MeasurementFcn', @cvkf_measurement);
            
            newTrack.age = 1;
            newTrack.totalVisibleCount = 1;
            newTrack.consecutiveInvisibleCount = 0;
            newTrack.collisionObject = newDetection.collisionObject;
            [newTrack.dims, newTrack.orientation] = extractGeometricFeatures(newTrack.collisionObject);
            newTrack.predictedState = initialState; % 初始预测即为初始状态
            newTrack.correctedState = initialState; % 初始修正也为初始状态
            newTrack.lastDetectionCentroid = newDetection.centroid;

            currentTracks = [currentTracks; newTrack];
            disp(['MATLAB规划节点(追踪): 初始化新轨迹 ID: ', num2str(newTrack.id)]);
        end
        
        % (可选) 对currentTracks按ID排序，方便调试查看
        if ~isempty(currentTracks)
            [~, sortIdx] = sort([currentTracks.id]);
            currentTracks = currentTracks(sortIdx);
        end

        disp(['MATLAB规划节点(追踪): 追踪完成。当前有效轨迹数量: ', num2str(length(currentTracks)), ...
            ', 下一个可用ID: ', num2str(updatedNextID)]);

    end

    % --- 卡尔曼滤波器辅助函数 ---
    function x_next = cvkf_motionModel(x_curr, dt_kf_ignored)
        % cvkf_motionModel: 匀速运动模型的状态转移函数 (用于kalmanFilter的Process属性)
        % x_curr: 当前状态 [cx; cy; cz; vx; vy; vz]
        % dt_kf_ignored: 此处dt由StateTransitionFcn的输入参数提供，这里可以忽略
        %
        % 注意: kalmanFilter对象的 'Process' 属性如果直接设为一个函数句柄，
        % 该函数不应依赖外部传入的dt。dt应通过 'StateTransitionFcn' 的方式传入。
        % 但为了兼容性或不同用法，此处保留dt参数，实际由StateTransitionFcn使用。
        % 或者，直接让 'Process' 为空，完全依赖 'StateTransitionFcn'。
        % 为清晰起见，此处 motionModel 假定 dt 已经通过某种方式获得了，
        % 但在实际的 kalmanFilter 调用中，我们使用 StateTransitionFcn 来处理 dt。
        % 因此，这个函数主要用于概念展示，实际的转移由cvkf_stateTransition完成。
        dt = 0.1; % 假设一个名义上的dt，实际由StateTransitionFcn覆盖
        F = [1 0 0 dt 0  0;
            0 1 0 0  dt 0;
            0 0 1 0  0  dt;
            0 0 0 1  0  0;
            0 0 0 0  1  0;
            0 0 0 0  0  1];
        x_next = F * x_curr;
    end

    function F = cvkf_stateTransition(x_curr_ignored, dt_kf, frameCount_ignored)
        % cvkf_stateTransition: 返回匀速运动模型的离散时间状态转移矩阵 F
        % x_curr_ignored: 当前状态 (未使用，因为F不依赖于x_curr)
        % dt_kf: 时间步长
        % frameCount_ignored: 当前帧计数 (未使用，但保留接口以备将来扩展)
        F = [1 0 0 dt_kf 0  0;
            0 1 0 0  dt_kf 0;
            0 0 1 0  0  dt_kf;
            0 0 0 1  0  0;
            0 0 0 0  1  0;
            0 0 0 0  0  1];
    end

    function z_meas = cvkf_measurementModel(x_curr)
        % cvkf_measurementModel: 测量模型函数 (用于kalmanFilter的MeasurementFcn属性)
        % x_curr: 当前状态 [cx; cy; cz; vx; vy; vz]
        % 输出 z_meas: 测量到的量 [cx; cy; cz]
        H = [1 0 0 0 0 0;
            0 1 0 0 0 0;
            0 0 1 0 0 0];
        z_meas = H * x_curr;
    end

    function H_matrix = cvkf_measurement(x_curr_ignored)
        % cvkf_measurement: 返回测量矩阵 H (用于kalmanFilter的MeasurementJacobianFcn，如果测量模型非线性)
        % 或者直接在创建KF时指定。此处提供一个函数形式。
        % 对于线性测量，可以直接在创建KF时使用 MeasurementModel 属性指定H矩阵。
        % 如果 kalmanFilter 的 'MeasurementFcn' 提供了非线性函数，则需要
        % 'MeasurementJacobianFcn'。由于我们的测量模型是线性的 (z = Hx),
        % MeasurementFcn 可以直接返回 Hx, 或者更简单地，在创建 kalmanFilter 对象时
        % 不设置 MeasurementFcn, 而是让 H 矩阵通过 predict 和 correct 的内部机制处理。
        % 此处命名为cvkf_measurement是为了与KF对象的MeasurementFcn参数对应。
        % 然而，更标准的做法是若测量模型为 z = Hx，则 KF 的 measurement update 使用 H。
        % 此函数返回的应是 H 矩阵本身，如果 MeasurementFcn 被设置成这个函数的话，
        % kalmanFilter 对象会期望它返回测量值 z，而不是 H。
        % 更好的方式是在创建 kalmanFilter 时不指定 MeasurementFcn, 而让其使用默认的线性假设，
        % 或者在 correct 函数中显式提供 H 矩阵（如果该函数签名支持）。
        %
        % 在 MATLAB R2018b 及以后版本的 kalmanFilter 对象中，如果测量是线性的 z = Hx + v,
        % 你通常不需要定义 MeasurementFcn 和 MeasurementJacobianFcn。
        % 只需要在 correct(kf, z_meas) 时，kf 对象内部知道 H (通常从创建时推断或默认)。
        % 如果 measurement model 是非线性的 z = h(x) + v, 则需要 MeasurementFcn h(x)
        % 和 MeasurementJacobianFcn (雅可比矩阵 H = dh/dx)。
        %
        % 为了与上面 assign_and_track_obstacle_ids 中手动计算 residualCovariance 的 H_matrix 一致，
        % 这里我们定义一个返回 H 矩阵的函数，尽管它可能不直接用于 kalmanFilter 的 MeasurementFcn。
        H_matrix = [1 0 0 0 0 0;
                    0 1 0 0 0 0;
                    0 0 1 0 0 0];
    end


    % --- 几何特征提取辅助函数 ---
    function [dims, orientationMatrix, shapeType] = extractGeometricFeatures(collisionObj)
        % extractGeometricFeatures: 从 collisionShape 对象中提取尺寸和方向。
        % 输入:
        %   collisionObj (collisionShape): 如 collisionBox, collisionCylinder, collisionSphere
        % 输出:
        %   dims (1xN double): 尺寸向量。
        %       - collisionBox: [Length, Width, Height] (X, Y, Z 维度)
        %       - collisionCylinder: [Radius, Length]
        %       - collisionSphere: [Radius]
        %       - 其他或无法识别: []
        %   orientationMatrix (3x3 double): 旋转矩阵。对于球体，返回 eye(3)。
        %                                  如果无法获取，返回 []。
        %   shapeType (string): "box", "cylinder", "sphere", or "unknown"

        dims = [];
        orientationMatrix = [];
        shapeType = "unknown";

        if isa(collisionObj, 'collisionBox')
            dims = [collisionObj.X, collisionObj.Y, collisionObj.Z]; % X, Y, Z 对应 Length, Width, Height
            orientationMatrix = collisionObj.Pose(1:3, 1:3);
            shapeType = "box";
        elseif isa(collisionObj, 'collisionCylinder')
            dims = [collisionObj.Radius, collisionObj.Length];
            orientationMatrix = collisionObj.Pose(1:3, 1:3);
            shapeType = "cylinder";
        elseif isa(collisionObj, 'collisionSphere')
            dims = [collisionObj.Radius];
            orientationMatrix = eye(3); % 球体对称，方向通常不重要或设为单位阵
            shapeType = "sphere";
        else
            disp('警告: 未知或不支持的碰撞对象类型，无法提取几何特征。');
        end
    end
    % --------------------------------------------------------------------
    % 辅助函数: 可视化单个配置的碰撞检查状态
    % --------------------------------------------------------------------
    function visualize_single_config_collision_status(robotModel, configArray, isColliding, markerId, collisionMarkerPublisher)
        % robotModel: 机械臂的 rigidBodyTree 模型
        % configArray: 要可视化的单个关节配置 (行向量)
        % isColliding (logical): 该配置是否发生碰撞
        % markerId (integer): 用于此标记的唯一ID
        % collisionMarkerPublisher: 用于发布 visualization_msgs/MarkerArray 的 rospublisher 对象

        if isempty(collisionMarkerPublisher)
            return; % 如果发布器未成功创建，则不执行任何操作
        end

        markerArrayMsg = rosmessage(collisionMarkerPublisher); % 创建 MarkerArray 消息

        % 计算末端执行器在给定配置下的位姿
        % 确保configArray是列向量以用于getTransform
        currentConfigStruct = robotModel.homeConfiguration; % 获取结构体形式的配置
        for j_idx_vis = 1:length(configArray)
            if j_idx_vis <= length(currentConfigStruct)
                currentConfigStruct(j_idx_vis).JointPosition = configArray(j_idx_vis);
            end
        end
        eeTform = getTransform(robotModel, currentConfigStruct, 'tool0'); % 假设末端执行器名为 'tool0'

        % 创建可视化标记 (Sphere)
        markerMsg = rosmessage('visualization_msgs/Marker');
        markerMsg.Header.FrameId = 'base_link'; % 假设相对于 base_link 可视化
        markerMsg.Header.Stamp = rostime('now');
        markerMsg.Ns = 'collision_check_status'; % 命名空间
        markerMsg.Id = int32(markerId); % 确保为整数类型
        markerMsg.Type = markerMsg.SPHERE; % 球体标记
        markerMsg.Action = markerMsg.ADD;

        % 设置标记位姿 (球心位于末端执行器位置)
        markerMsg.Pose.Position.X = eeTform(1,4);
        markerMsg.Pose.Position.Y = eeTform(2,4);
        markerMsg.Pose.Position.Z = eeTform(3,4);
        markerMsg.Pose.Orientation.W = 1.0; % 默认方向

        % 设置标记尺寸
        markerMsg.Scale.X = 0.06;
        markerMsg.Scale.Y = 0.06;
        markerMsg.Scale.Z = 0.06;

        % 根据碰撞状态设置颜色
        if isColliding
            markerMsg.Color.R = 1.0; % 红色表示碰撞
            markerMsg.Color.G = 0.0;
            markerMsg.Color.B = 0.0;
        else
            markerMsg.Color.R = 0.0; % 绿色表示无碰撞
            markerMsg.Color.G = 1.0;
            markerMsg.Color.B = 0.0;
        end
        markerMsg.Color.A = 0.8; % 透明度

        markerMsg.Lifetime = rosduration(2.0); % 标记持续时间 (例如2秒)

        markerArrayMsg.Markers(1) = markerMsg; % 将单个标记添加到数组
        send(collisionMarkerPublisher, markerArrayMsg); % 发布标记数组
    end
end
