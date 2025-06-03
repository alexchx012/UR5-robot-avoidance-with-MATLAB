function robot = create_robot_model(enableGraphics)
% create_robot_model: 创建并返回一个基于D-H参数的rigidBodyTree机械臂模型。
    %
    % 输入:
    %   enableGraphics (logical): 可选参数。如果为 true (默认) 或未提供，则显示模型。
    %                             如果为 false，则不执行任何绘图操作。
    %
    % 输出:
    %   robot (rigidBodyTree): 构建好的机械臂模型对象。

    % 参数默认值处理
    if nargin < 1
        enableGraphics = true; % 如果未提供参数，则默认启用图形显示
    end

    % 创建一个刚体树对象 (rigidBodyTree) 用于表示机械臂模型。
    % 'DataFormat', 'column' 指定关节位置等数据以列向量形式存储。
    % 'MaxNumBodies' 预分配空间，可以略大于实际连杆数，以提高添加连杆时的效率。
    robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 10);
    disp('MATLAB (create_robot_model): rigidBodyTree 对象已初始化。');

    % --------------------------------------------------------------------
    % 标准Denavit-Hartenberg (D-H)参数定义
    % --------------------------------------------------------------------
    % 格式: [a, alpha, d, theta_offset]
    % a (link length): 连杆长度 - Z(i-1)轴与Z(i)轴之间的公垂线长度，沿X(i)轴的平移。
    % alpha (link twist): 连杆扭转角 - Z(i-1)轴与Z(i)轴之间的夹角，绕X(i)轴的旋转。
    % d (link offset): 连杆偏距 - X(i-1)轴与X(i)轴之间沿Z(i-1)轴的距离。
    % theta_offset (joint angle offset): 关节角偏置 - X(i-1)轴与X(i)轴之间的夹角，绕Z(i-1)轴的旋转。
    %                                     对于转动关节，实际关节角 = 关节变量 + theta_offset。

    % UR5 Denavit-Hartenberg (D-H)参数定义
    % 这些值直接从UR5的URDF文件 (ur5.urdf.xacro) 中提取或推算得到。
    % URDF中的参数: a = [0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0]
    %                d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
    %                alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
    % 注意: URDF中的 'a' 和 'd' 可能与标准D-H约定中的符号或分配方式略有不同，
    %       需要仔细转换以匹配MATLAB rigidBodyTree的D-H输入格式 [a, alpha, d, theta_offset]。

    d1_val = 0.089159;  % 从base_link到shoulder_link的Z轴偏移 (对应URDF d1)
    a2_val = 0.42500;   % shoulder_link到upper_arm_link的X轴偏移 (对应URDF -a2, 因为MATLAB的a是正值)
    a3_val = 0.39225;   % upper_arm_link到forearm_link的X轴偏移 (对应URDF -a3)
    d4_val = 0.10915;   % forearm_link到wrist_1_link的Z轴偏移 (对应URDF d4)
    d5_val = 0.09465;   % wrist_1_link到wrist_2_link的Z轴偏移 (对应URDF d5)
    d6_val = 0.0823;    % wrist_2_link到wrist_3_link的Z轴偏移 (对应URDF d6)

    gripper_base_offset_val = 0.030; % 与URDF中自定义的gripper_base_offset保持一致
    tcp_offset_val = 0.150;          % 与URDF中自定义的tcp_offset保持一致

    % D-H参数表 - 适配UR5机械臂
    % [a, alpha, d, theta_offset]
    % theta_offset通常为0，除非关节的零位与D-H约定的零位有偏差。
    % UR5的alpha值直接使用。
    dhparams_table = [ %  a_i     alpha_i   d_i         theta_i_offset
                      0,        pi/2,   d1_val,     0;  % 关节 1 (shoulder_pan_joint)
                      a2_val,   0,      0,          0;  % 关节 2 (shoulder_lift_joint) -> URDF a2为负, MATLAB a为正
                      a3_val,   0,      0,          0;  % 关节 3 (elbow_joint) -> URDF a3为负, MATLAB a为正
                      0,        pi/2,   d4_val,     0;  % 关节 4 (wrist_1_joint)
                      0,        -pi/2,  d5_val,     0;  % 关节 5 (wrist_2_joint)
                      0,        0,      d6_val,     0;  % 关节 6 (wrist_3_joint)
    ];

    % --------------------------------------------------------------------
    % 定义连杆和关节的名称 (必须与URDF文件中的名称完全一致)
    % --------------------------------------------------------------------
    linkNames_arm = { % UR5机械臂的活动连杆名称 (与URDF一致, 无前缀)
        'shoulder_link',
        'upper_arm_link',
        'forearm_link',
        'wrist_1_link',
        'wrist_2_link',
        'wrist_3_link'
    };
    
    jointNames_arm = { % UR5机械臂的活动关节名称 (与URDF一致, 无前缀)
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    };
    
    % --------------------------------------------------------------------
    % 循环添加机械臂的基本连杆和关节到模型中
    % --------------------------------------------------------------------
    current_parent_link_name = robot.BaseName; % 获取机器人模型的基座名称 (默认为 'base')

    for i = 1:size(dhparams_table, 1) % 遍历D-H参数表的每一行，对应一个关节和它驱动的连杆
        % 创建一个新的刚体对象 (连杆)
        body_i = rigidBody(linkNames_arm{i}); 
        
        % 创建一个新的关节对象, 类型为旋转关节 ('revolute')
        joint_i = rigidBodyJoint(jointNames_arm{i}, 'revolute'); 
        
        % 使用该行D-H参数设置关节的固定变换。
        % 'dh' 字符串指定了参数遵循标准D-H约定: [a, alpha, d, theta_offset]
        setFixedTransform(joint_i, dhparams_table(i,:), 'dh'); 
        
        % 将创建的关节赋给当前创建的连杆
        body_i.Joint = joint_i; 
        
        % 将当前连杆 (body_i) 添加到机器人模型 (robot) 中，并指定其父连杆
        addBody(robot, body_i, current_parent_link_name); 
        
        % 更新父连杆名称为当前连杆的名称，为下一次迭代做准备
        current_parent_link_name = linkNames_arm{i}; 
    end
    
    % --------------------------------------------------------------------
    % 添加末端执行器安装点 (ee_link) - 作为固定连杆
    % --------------------------------------------------------------------
    % 添加自定义的夹爪基座 (gripper_base_link) - 作为固定连杆
    % UR5的 wrist_3_link 是法兰盘，我们将自定义的 gripper_base_link 固定到它上面。
    
    body_gripper_base = rigidBody('gripper_base_link'); % 创建名为 'gripper_base_link' 的刚体对象
    joint_to_gripper_base = rigidBodyJoint('gripper_base_joint', 'fixed'); % 创建连接到 gripper_base_link 的固定关节

    % 定义从 wrist_3_link (UR5法兰盘) 到 gripper_base_link 的静态变换。
    % 这对应于URDF中 <joint name="gripper_base_joint"> 的 <origin>。
    % URDF中定义: <origin xyz="0 0 ${gripper_base_offset}" rpy="0 0 0"/>
    tform_wrist3_to_gripper_base = trvec2tform([0, 0, gripper_base_offset_val]);
    
    setFixedTransform(joint_to_gripper_base, tform_wrist3_to_gripper_base);
    body_gripper_base.Joint = joint_to_gripper_base;
    addBody(robot, body_gripper_base, linkNames_arm{end}); % 添加 gripper_base_link 到 wrist_3_link

    % --------------------------------------------------------------------
    % 添加工具中心点 (tool0) - 作为固定连杆
    % --------------------------------------------------------------------
    % tool0 代表机械臂末端工具的实际操作点 (TCP - Tool Center Point)。
    % tool0 通过一个固定关节连接到 gripper_base_link。
    
    body_tool0 = rigidBody('tool0'); % 创建名为 'tool0' 的刚体对象
    joint_to_tool0 = rigidBodyJoint('tool0_joint', 'fixed'); % 创建连接到 tool0 的固定关节

    % 定义从 gripper_base_link 到 tool0 的静态变换。
    % 这对应于URDF中 <joint name="tcp_joint"> 的 <origin>。
    % URDF中定义: <origin xyz="0 0 ${tcp_offset}" rpy="0 0 0"/>
    tform_gripper_to_tcp_translation = [0, 0, tcp_offset_val];
    tform_gripper_to_tcp = trvec2tform(tform_gripper_to_tcp_translation);
    
    setFixedTransform(joint_to_tool0, tform_gripper_to_tcp);
    body_tool0.Joint = joint_to_tool0;
    addBody(robot, body_tool0, 'gripper_base_link'); % 添加 tool0 到 gripper_base_link

    % --------------------------------------------------------------------
    % 设置关节的运动范围 (PositionLimits)
    % --------------------------------------------------------------------
    % 关节限制的顺序应与 jointNames_arm 和 dhparams_table 表的顺序一致。
    % 单位为弧度。这些值应根据仿真需求或通用机械臂的典型范围设定。
    % 设置关节的运动范围 (PositionLimits) - 与URDF中为UR5设定的限制一致
    % 这些限制是在 custom_arm.urdf.xacro 中通过 <xacro:ur5_robot ... /> 宏传递的。
    joint_limits_table = [ % [lower_limit, upper_limit] in radians
        -2*pi, 2*pi;         % shoulder_pan_joint
        -pi/2, pi/2;         % shoulder_lift_joint
        -0.75*pi, 0.75*pi;   % elbow_joint
        -2*pi, 2*pi;         % wrist_1_joint
        -2*pi, 2*pi;         % wrist_2_joint
        -2*pi, 2*pi          % wrist_3_joint
    ];
    
    % 遍历机器人模型中的活动连杆 (Bodies cell array) 并设置其关节的限制。
    % rigidBodyTree 中的 Bodies 列表包含了所有添加的连杆 (不包括基座)。
    % 其顺序与添加时的顺序一致。
    for i = 1:length(jointNames_arm) % 假设活动关节数量与jointNames_arm长度相同
        % robot.Bodies{i} 对应于按顺序添加的第 i 个活动连杆。
        % 这个连杆的 .Joint 属性就是其连接到父连杆的关节。
        robot.Bodies{i}.Joint.PositionLimits = joint_limits_table(i,:);
    end
    
    % --------------------------------------------------------------------
    % (可选) 显示构建好的机器人模型以供检查
    % --------------------------------------------------------------------
    figure; % 创建一个新的图形窗口
    % 'Frames','on'显示坐标系; 'Visuals','on'显示视觉模型(如果URDF定义了并被MATLAB正确解析)
    % homeConfiguration(robot) 返回所有关节角为0的配置 (如果theta_offset不为0，则实际角度为theta_offset)
    show(robot, homeConfiguration(robot), 'Frames', 'on', 'Visuals', 'on'); 
    hold on; % 保持当前图形，以便添加更多绘图元素
    % 在图形中绘制一个世界坐标系参考标记 (可选，用于对比)
    plotTransforms([0,0,0], eul2quat([0,0,0]), "FrameSize", 0.25); 
    text(0,0,0.3, 'MATLAB_World_Origin'); % 使用 text 替代 FrameId
    axis equal; % 设置各轴比例相同，避免变形
    title('MATLAB中的UR5机械臂模型 (D-H参数定义)'); % 图形标题
    xlabel('X 轴 (m)'); % X轴标签
    ylabel('Y 轴 (m)'); % Y轴标签
    zlabel('Z 轴 (m)'); % Z轴标签
    view(135, 25); % 设置三维图形的观察视角 (方位角135度，仰角25度)
    grid on; % 显示网格

    % 在命令行窗口输出提示信息
    disp('MATLAB机械臂仿真模型 (robot) 创建完成');
end

% --------------------------------------------------------------------
% 辅助函数：rt2tr (Rotation matrix and translation vector to homogeneous TRansform)
% --------------------------------------------------------------------
% 功能: 将3x3旋转矩阵R和3x1平移向量t组合成4x4齐次变换矩阵T。
% 输入:
%   R: 3x3 旋转矩阵
%   t: 3x1 或 1x3 平移向量
% 输出:
%   T: 4x4 齐次变换矩阵
function T = rt2tr(R, t)
    % 检查输入参数的维度是否正确
    if ~isequal(size(R), [3,3]) % 检查R是否为3x3矩阵
        error('输入错误: 旋转矩阵 R 必须是 3x3。'); % 抛出错误
    end
    if ~(numel(t) == 3) % 检查t是否包含3个元素
        error('输入错误: 平移向量 t 必须包含3个元素。'); % 抛出错误
    end
    
    T = eye(4); % 初始化T为4x4单位矩阵
    T(1:3, 1:3) = R; % 将旋转矩阵R赋给T的左上角3x3部分 (旋转部分)
    T(1:3, 4) = t(:); % 将平移向量t (通过t(:)确保为列向量) 赋给T的最后一列的前三行 (平移部分)
end