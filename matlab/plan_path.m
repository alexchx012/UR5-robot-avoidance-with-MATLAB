function [pathStates, success] = plan_path(robot, startConfigVec, goalConfigVec, obstacleObjectsCell, enableGraphics)
    % plan_path: 使用RRT算法在关节空间中规划避障路径。
    %
    % 输入:
    %   robot (rigidBodyTree): 机械臂模型。
    %   startConfigVec (double row vector): 机械臂的起始关节配置 (弧度)。
    %   goalConfigVec (double row vector): 机械臂的目标关节配置 (弧度)。
    %   obstacleObjectsCell (cell array): 包含场景中障碍物的 collision object (例如 collisionBox) 的单元数组。
    %   enableGraphics (logical): 可选参数。如果为 true (默认) 或未提供，则显示规划路径和障碍物。
    %                             如果为 false，则不执行任何绘图操作。
    %
    % 输出:
    %   pathStates (matrix or []): 规划成功时，返回一个矩阵，每行是一个路径点 (关节配置)。
    %                              规划失败时，返回空数组 []。
    %   success (logical): 指示路径规划是否成功。

    % 参数默认值处理
    if nargin < 5
        enableGraphics = true; % 如果未提供参数，则默认启用图形显示
    end
    if nargin < 4
        obstacleObjectsCell = {}; % 如果未提供障碍物，则默认为空
    end

    disp('MATLAB (plan_path): 开始路径规划...');

    % 创建基于RRT的路径规划器 (manipulatorRRT)
    % manipulatorRRT 构造函数可以直接接受机器人模型和障碍物单元数组
    planner = manipulatorRRT(robot, obstacleObjectsCell);
    disp('MATLAB (plan_path): manipulatorRRT规划器已创建。');

    % 设置规划器参数 (这些参数可能需要根据具体机器人和环境调整)
    planner.MaxConnectionDistance = 0.5;  % RRT树节点间的最大连接距离 (弧度)
    planner.MaxIterations = 5000;         % RRT算法的最大迭代次数
    planner.GoalReachedFcn = @(planner, q, qTarget) all(abs(wrapToPi(q - qTarget)) < 0.01); % 自定义目标到达判断，更严格
    % planner.GoalBias = 0.1; % 朝向目标的采样偏置概率
    % planner.ValidationDistance = 0.05; % 碰撞检测时路径段的采样间隔 (弧度)

    disp(['MATLAB (plan_path): 规划器参数设置 - MaxConnectionDistance: ', num2str(planner.MaxConnectionDistance), ...
          ', MaxIterations: ', num2str(planner.MaxIterations)]);

    % 执行路径规划
    % plan 函数需要起始和目标配置作为行向量
    disp('MATLAB (plan_path): 正在执行 plan() 函数...');
    tic; % 开始计时
    [pthObj, solnInfo] = plan(planner, startConfigVec(:)', goalConfigVec(:)'); % 确保是行向量
    planningDuration = toc; % 结束计时
    disp(['MATLAB (plan_path): plan() 函数执行完毕，耗时 ', num2str(planningDuration), ' 秒。']);

    pathStates = []; % 初始化输出

    % 检查规划结果
    if solnInfo.IsPathFound
        disp('MATLAB (plan_path): 路径规划成功！');
        success = true;

        % (可选) 平滑和插值路径
        % disp('MATLAB (plan_path): 正在平滑和插值路径...');
        % pthObj = shortenPath(pthObj); % （旧版函数，MATLAB R2021a+中可能无）
        % RRT*通常能找到较优路径，但 manipulatorRRT 是基础RRT，可能需要后处理
        % 可以使用 bsplinepolytraj 或其他方法进行平滑，但会增加复杂性。
        % 为保持与原指南逻辑接近，我们先直接使用RRT的输出点。
        % 如果需要更平滑轨迹，应在 generate_trajectory 中处理。
        
        % 提取路径点 (关节配置)
        pathStates = pthObj.PathStates; % R2023a onwards, .PathStates
        if isempty(pathStates) && ~isempty(pthObj.Configs) % Fallback for some versions
            pathStates = pthObj.Configs;
        end
        
        disp(['MATLAB (plan_path): 规划得到的路径包含 ', num2str(size(pathStates,1)), ' 个状态点。']);

        % 根据 enableGraphics 参数控制是否可视化路径
        if enableGraphics && ~isempty(pathStates)
            disp('MATLAB (plan_path): 正在生成规划路径的图形...');
            figure; % 仅在启用图形时创建新窗口
            
            % 显示起始配置的机器人
            show(robot, startConfigVec(:)', 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
            hold on;
            axis equal;
            title('规划的避障路径 (关节空间RRT)');
            xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
            view(135, 25); grid on;

            % 显示障碍物
            if ~isempty(obstacleObjectsCell)
                disp('MATLAB (plan_path): 正在绘制障碍物...');
                for i = 1:length(obstacleObjectsCell)
                    currentObs = obstacleObjectsCell{i};
                    if isa(currentObs, 'collisionMesh') % 网格类型障碍物
                         plot(currentObs, 'FaceColor', [0.8 0.2 0.2], 'FaceAlpha', 0.5);
                    elseif isa(currentObs, 'collisionBox') || isa(currentObs, 'collisionSphere') || isa(currentObs, 'collisionCylinder')
                         plot(currentObs, 'FaceColor', [0.8 0.2 0.2], 'FaceAlpha', 0.5); % 使用内置plot
                    end
                end
            end
            
            % 显示路径上的机器人动画 (每隔几个点显示一次以加快可视化)
            disp('MATLAB (plan_path): 正在动画演示路径...');
            pathPlotSkip = max(1, floor(size(pathStates,1)/20)); % 最多显示约20帧
            for i = 1:pathPlotSkip:size(pathStates, 1)
                show(robot, pathStates(i,:)', 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
                drawnow; % 刷新图形
                % pause(0.01); % 短暂暂停以观察
            end
            % 显示最终配置
            show(robot, pathStates(end,:)', 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
            hold off;
            disp('MATLAB (plan_path): 路径可视化完成。');
        elseif enableGraphics && isempty(pathStates)
            disp('MATLAB (plan_path): 路径规划成功但路径状态为空，无法可视化。');
        else
            disp('MATLAB (plan_path): 图形输出已禁用。');
        end
    else
        disp('MATLAB (plan_path): 路径规划失败。');
        disp(['MATLAB (plan_path): 失败原因/信息: ', solnInfo.Status]); % manipulatorRRT 的 solnInfo.Status 可能不直接提供文本原因
        success = false;
        pathStates = []; % 确保失败时返回空
    end
end