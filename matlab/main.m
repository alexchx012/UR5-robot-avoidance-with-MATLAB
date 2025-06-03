%% 基于主动避障的机械臂点到点运动规划
% 本脚本实现了一个完整的机械臂主动避障系统，包括：
% 1. 连接到ROS/Gazebo
% 2. 创建机械臂模型
% 3. 接收环境感知数据
% 4. 规划避障路径
% 5. 执行轨迹并监测环境变化
% 6. 在检测到新障碍物时重新规划路径

%% 检查ROS连接状态
try
    rosnode list;
    disp('已连接到ROS主节点');
catch
    disp('未连接到ROS主节点，尝试连接...');
    connect_to_ros();
end

%% 显示菜单
disp('=== 机械臂主动避障系统 ===');
disp('1. 执行基本避障任务');
disp('2. 执行主动避障任务');
disp('3. 测试机械臂控制');
disp('4. 测试点云处理');
disp('5. 退出');

choice = input('请选择功能 (1-5): ');

%% 根据用户选择执行相应功能
switch choice
    case 1
        disp('执行基本避障任务...');
        arm_obstacle_avoidance();
    case 2
        disp('执行主动避障任务...');
        active_obstacle_avoidance();
    case 3
        disp('测试机械臂控制...');
        test_arm_control();
    case 4
        disp('测试点云处理...');
        test_point_cloud_processing();
    case 5
        disp('退出程序');
        % 修复：正确检查ROS连接状态
        try
            rosnode list;
            rosshutdown;
        catch
            % ROS未连接，无需关闭
        end
        return;
    otherwise
        disp('无效选择，请重新运行程序');
end

%% 辅助函数：测试机械臂控制
function test_arm_control()
    % 修复：正确检查ROS连接状态
    try
        rosnode list;
        disp('ROS已连接');
    catch
        disp('ROS未连接，尝试连接...');
        connect_to_ros();
    end
    
    % 创建轨迹发布者
    trajPub = rospublisher('/custom_arm/arm_controller/command', 'trajectory_msgs/JointTrajectory');
    
    % 关节名称
    jointNames = {
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    };
    
    % 创建轨迹消息
    trajMsg = rosmessage(trajPub);
    trajMsg.JointNames = jointNames;
    
    % 创建一系列测试姿态
    poses = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  % 初始位置
        0.5, 0.0, 0.0, 0.0, 0.0, 0.0;  % 旋转肩部
        0.5, 0.5, 0.0, 0.0, 0.0, 0.0;  % 抬起上臂
        0.5, 0.5, 0.5, 0.0, 0.0, 0.0;  % 弯曲肘部
        0.5, 0.5, 0.5, 0.5, 0.0, 0.0;  % 旋转腕部1
        0.5, 0.5, 0.5, 0.5, 0.5, 0.0;  % 旋转腕部2
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5;  % 旋转腕部3
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0   % 返回初始位置
    ];
    
    % 执行每个姿态
    for i = 1:size(poses, 1)
        % 清空轨迹点
        trajMsg.Points = [];
        
        % 创建轨迹点
        point = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        point.Positions = poses(i, :);
        point.Velocities = zeros(1, 6);
        point.Accelerations = zeros(1, 6);
        point.TimeFromStart = rosduration(2.0);
        
        % 添加轨迹点
        trajMsg.Points = point;
        
        % 发送轨迹命令
        disp(['发送姿态 ', num2str(i), '/', num2str(size(poses, 1))]);
        send(trajPub, trajMsg);
        
        % 等待执行完成
        pause(3);
    end
    
    disp('机械臂控制测试完成');
end

%% 辅助函数：测试点云处理
function test_point_cloud_processing()
    % 修复：正确检查ROS连接状态
    try
        rosnode list;
        disp('ROS已连接');
    catch
        disp('ROS未连接，尝试连接...');
        connect_to_ros();
    end
    
    % 创建机械臂模型
    robot = create_robot_model();
    
    % 创建点云订阅者
    pointCloudSub = rossubscriber('/camera_camera/depth/points', 'sensor_msgs/PointCloud2');
    
    % 等待接收点云数据
    disp('等待接收点云数据...');
    pointCloudMsg = receive(pointCloudSub, 10);
    
    % 处理点云数据
    [obstacleObjects, processedPointCloud] = process_point_cloud(pointCloudMsg, robot);
    
    % 显示结果
    disp(['识别到 ', num2str(length(obstacleObjects)), ' 个障碍物']);
    
    % 可视化障碍物和处理后的点云
    figure;
    pcshow(processedPointCloud); % 显示处理后的点云
    title('处理后的点云数据和识别的障碍物'); % 更新标题以反映内容
    hold on;
    axis equal; % 保持坐标轴比例一致，使物体形状正确
    grid on;    % 显示网格
    xlabel('X (m)'); % 添加坐标轴标签
    ylabel('Y (m)');
    zlabel('Z (m)');
    view(3); % 设置为三维视角

    % 显示识别出的障碍物边界框
    if ~isempty(obstacleObjects)
        disp(['正在可视化 ', num2str(length(obstacleObjects)), ' 个识别的障碍物...']);
        for i = 1:length(obstacleObjects)
            obs = obstacleObjects{i};
            if isa(obs, 'collisionBox')
                % 使用 show 函数显示 collisionBox 对象
                % show 函数会自动使用 obs.Pose 来确定其位置和方向
                show(obs); 
                % 您也可以使用 plot 函数并指定颜色和透明度，例如:
                % plot(obs, 'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k');
            elseif isa(obs, 'collisionCylinder') || isa(obs, 'collisionSphere')
                % 如果 process_point_cloud 也输出这些类型，同样处理
                show(obs);
            else
                warning('test_point_cloud_processing: 未知障碍物类型，无法可视化。');
            end
        end
    else
        disp('没有识别到障碍物进行可视化。');
    end
    
    hold off;
    disp('点云处理测试完成。');
end