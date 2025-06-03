function [obstacleStructs, processedPointCloudForVis] = process_point_cloud(pointCloudMsg, robot, varargin)
% process_point_cloud: 处理ROS点云消息，提取障碍物信息。
%
% 输入:
%   pointCloudMsg (sensor_msgs/PointCloud2): 来自ROS的原始点云消息。
%   robot (rigidBodyTree): 机械臂模型，用于潜在的自滤波（本版本未实现）。
%   varargin: 可选参数名值对，用于覆盖默认处理参数。支持的参数包括:
%             'EnableGraphics' (logical, default: true)
%             'GridStep' (double, default: 0.025) - 降采样网格大小 (米)
%             'MaxDistanceToPlane' (double, default: 0.03) - RANSAC地面移除: 点到平面最大距离 (米)
%             'ExpectedNormal' (1x3 double, default: [0,0,1]) - RANSAC地面移除: 期望法向量
%             'MaxAngularDeviation' (double, default: deg2rad(10)) - RANSAC地面移除: 最大角度偏差 (弧度)
%             'MinGroundPlanePoints' (double, default: 50) - RANSAC地面移除: 最小内点数
%             'UseRANSACFallback' (logical, default: true) - 是否在RANSAC失败时使用Z轴阈值法
%             'ZGroundThreshold' (double, default: 0.025) - Z轴阈值法: 地面高度阈值 (米)
%             'MinClusterDistance' (double, default: 0.06) - 欧几里得聚类: 最小聚类距离 (米)
%             'MinPointsPerCluster' (double, default: 10) - 欧几里得聚类: 每聚类最小点数
%             'MaxPointsPerCluster' (double, default: 25000) - 欧几里得聚类: 每聚类最大点数
%
% 输出:
%   obstacleStructs (cell array): 包含障碍物结构体 (含 .collisionObject 和 .centroid) 的单元数组。
%   processedPointCloudForVis (pointCloud): 处理后的点云对象，可用于外部可视化。

% --- 解析输入参数 ---
defaultParams = struct(...
    'EnableGraphics', true, ...
    'GridStep', 0.025, ...
    'MaxDistanceToPlane', 0.03, ...
    'ExpectedNormal', [0, 0, 1], ...
    'MaxAngularDeviation', deg2rad(10), ...
    'MinGroundPlanePoints', 50, ...
    'UseRANSACFallback', true, ...
    'ZGroundThreshold', 0.025, ...
    'MinClusterDistance', 0.06, ...
    'MinPointsPerCluster', 10, ...
    'MaxPointsPerCluster', 25000 ...
);

% 使用 inputParser 进行参数解析和验证
p = inputParser;
p.FunctionName = 'process_point_cloud'; % 用于错误消息

% 添加必需参数
% 添加必需参数
addRequired(p, 'pointCloudMsg', @(x) isa(x, 'ros.msg.sensor_msgs.PointCloud2') || isa(x, 'pointCloud') || isstruct(x)); % 允许ROS消息、点云对象或结构体
addRequired(p, 'robot', @(x) isa(x, 'rigidBodyTree') || isempty(x)); % robot 参数在本函数当前实现中未直接使用，但保留接口

% 添加可选参数 (从 varargin 中解析)
fields = fieldnames(defaultParams);
for k=1:length(fields)
    addParameter(p, fields{k}, defaultParams.(fields{k}));
end

parse(p, pointCloudMsg, robot, varargin{:});

% 将解析后的参数赋给局部变量
enableGraphics        = p.Results.EnableGraphics;
gridStep              = p.Results.GridStep;
maxDistanceToPlane    = p.Results.MaxDistanceToPlane;
expectedNormal        = p.Results.ExpectedNormal;
maxAngularDeviation   = p.Results.MaxAngularDeviation;
minGroundPlanePoints  = p.Results.MinGroundPlanePoints;
useRANSACFallback     = p.Results.UseRANSACFallback;
zGroundThreshold      = p.Results.ZGroundThreshold;
minClusterDistance    = p.Results.MinClusterDistance;
minPointsPerCluster   = p.Results.MinPointsPerCluster;
maxPointsPerCluster   = p.Results.MaxPointsPerCluster;

% --- 函数主体开始 ---
disp('MATLAB (process_point_cloud): 开始处理点云数据...');

% 将ROS点云消息转换为MATLAB点云对象
try
    % 检查pointCloudMsg的类型并相应处理
    if isa(pointCloudMsg, 'pointCloud')
        % 已经是MATLAB pointCloud对象
        ptCloud = pointCloudMsg;
        disp('MATLAB (process_point_cloud): 输入已经是pointCloud对象。');
    elseif isa(pointCloudMsg, 'ros.msg.sensor_msgs.PointCloud2')
        % 是ROS PointCloud2消息，需要转换
        try
            disp('MATLAB (process_point_cloud): 尝试将ROS PointCloud2消息转换为点云...');
            xyzPoints = rosReadXYZ(pointCloudMsg);
            if isempty(xyzPoints) || size(xyzPoints,1) < minPointsPerCluster
                warning('MATLAB (process_point_cloud): 点云数据为空或点数过少。');
                obstacleStructs = {};
                processedPointCloudForVis = pointCloud(zeros(0,3,'single')); 
                return;
            end
            ptCloud = pointCloud(xyzPoints);
            disp(['MATLAB (process_point_cloud): 成功从ROS消息创建点云，包含 ', num2str(size(xyzPoints,1)), ' 个点。']);
        catch ME_ros_convert
            error('无法将ROS PointCloud2消息转换为点云: %s', ME_ros_convert.message);
        end
    elseif isstruct(pointCloudMsg)
        % 处理结构体类型的输入
        if isfield(pointCloudMsg, 'XYZ') && ~isempty(pointCloudMsg.XYZ)
            ptCloud = pointCloud(pointCloudMsg.XYZ);
            disp('MATLAB (process_point_cloud): 从结构体创建点云对象。');
        else
            error('输入结构体缺少有效的XYZ字段');
        end
    else
        % 不支持的类型
        error('输入类型 %s 不受支持。支持的类型: pointCloud, ros.msg.sensor_msgs.PointCloud2, 或带XYZ字段的struct', class(pointCloudMsg));
    end
catch ME_read_pc
    warning(ME_read_pc.identifier, 'MATLAB (process_point_cloud): 无法读取或转换点云数据。错误: %s', ME_read_pc.message);
    obstacleStructs = {};
    processedPointCloudForVis = pointCloud(zeros(0,3,'single'));
    return;
end
disp(['MATLAB (process_point_cloud): 点云对象已准备好，包含 ', num2str(ptCloud.Count), ' 个点。']);

% 降采样以提高处理速度
ptCloudDownsampled = pcdownsample(ptCloud, 'gridAverage', gridStep);
disp(['MATLAB (process_point_cloud): 点云已降采样，网格大小: ', num2str(gridStep), ' 米。剩余点数: ', num2str(ptCloudDownsampled.Count)]);

% 移除离群点 (使用 pcdenoise 的默认参数，或者也可以参数化这些)
% 为保持与原版一致，此处不参数化pcdenoise内部参数
[ptCloudDenoised, ~] = pcdenoise(ptCloudDownsampled);
disp(['MATLAB (process_point_cloud): 点云已去噪。剩余点数: ', num2str(ptCloudDenoised.Count)]);

% 地面移除
ransacSuccess = false; 
nonGroundCloud = ptCloudDenoised; % 默认情况下，如果没有地面移除，则所有点都是非地面点

if ptCloudDenoised.Count > minGroundPlanePoints % 只有点数足够多时才尝试RANSAC
    disp('MATLAB (process_point_cloud): 尝试使用RANSAC拟合并移除地面平面...');
    try
        [planeModel, inlierIndices, outlierIndices] = pcfitplane(ptCloudDenoised, ...
                                                                 maxDistanceToPlane, ...
                                                                 expectedNormal, ...
                                                                 maxAngularDeviation);

        if ~isempty(planeModel) && numel(inlierIndices) >= minGroundPlanePoints
            ransacSuccess = true;
            disp(['MATLAB (process_point_cloud): RANSAC成功拟合地面平面。模型参数 (A,B,C,D): [', ...
                  num2str(planeModel.Parameters(1)), ', ', num2str(planeModel.Parameters(2)), ...
                  ', ', num2str(planeModel.Parameters(3)), ', ', num2str(planeModel.Parameters(4)), ']']);
            nonGroundCloud = select(ptCloudDenoised, outlierIndices);
            disp(['MATLAB (process_point_cloud): 已使用RANSAC移除地面点。剩余非地面点数: ', num2str(nonGroundCloud.Count)]);
        else
            if isempty(planeModel)
                warning('MATLAB (process_point_cloud): RANSAC未能拟合出平面模型。');
            else
                warning('MATLAB (process_point_cloud): RANSAC拟合的平面内点数 (%d) 少于最小要求的地面点数 (%d)。', numel(inlierIndices), minGroundPlanePoints);
            end
        end
    catch ME_ransac
        warning(ME_ransac.identifier, 'MATLAB (process_point_cloud): RANSAC平面拟合过程中发生错误: %s', ME_ransac.message);
    end
else
     disp('MATLAB (process_point_cloud): 点云数量不足以进行RANSAC地面拟合，跳过RANSAC。');
end

if ~ransacSuccess && useRANSACFallback
    disp('MATLAB (process_point_cloud): RANSAC地面移除失败或未执行，将回退到基于Z轴阈值的原始方法。');
    % 需要确保 nonGroundCloud 是基于 ptCloudDenoised 进行操作
    nonGroundIndices = ptCloudDenoised.Location(:,3) > zGroundThreshold;
    nonGroundCloud = select(ptCloudDenoised, nonGroundIndices);
    disp(['MATLAB (process_point_cloud): 已使用Z轴阈值法移除地面点。阈值 Z > ', num2str(zGroundThreshold), ' 米。剩余非地面点数: ', num2str(nonGroundCloud.Count)]);
elseif ~ransacSuccess && ~useRANSACFallback
    disp('MATLAB (process_point_cloud): RANSAC地面移除失败或未执行，且未启用回退策略。将使用去噪后的所有点进行聚类。');
    nonGroundCloud = ptCloudDenoised; % 使用去噪后的所有点
end

if isempty(nonGroundCloud.Location) || nonGroundCloud.Count < minPointsPerCluster
    warning('MATLAB (process_point_cloud): 移除地面点后，点云为空或点数过少以进行聚类。');
    obstacleStructs = {};
    processedPointCloudForVis = nonGroundCloud;
    if nonGroundCloud.Count == 0 
         processedPointCloudForVis = pointCloud(zeros(0,3,'single'));
    end
    return;
end

% 欧几里得聚类
[labels, numClusters] = pcsegdist(nonGroundCloud, minClusterDistance, ...
                                 [minPointsPerCluster, maxPointsPerCluster]);
disp(['MATLAB (process_point_cloud): 点云已分割成 ', num2str(numClusters), ' 个聚类。']);

obstacleStructs = cell(1, numClusters);
validObstacleCount = 0;

for i = 1:numClusters
    clusterIndices = (labels == i);
    clusterCloud = select(nonGroundCloud, clusterIndices);

    minPt = min(clusterCloud.Location, [], 1);
    maxPt = max(clusterCloud.Location, [], 1);
    dimensions = maxPt - minPt;

    if all(dimensions > 1e-3) % 最小维度阈值
        validObstacleCount = validObstacleCount + 1;
        obstacleBox = collisionBox(dimensions(1), dimensions(2), dimensions(3));
        centerPosition = (minPt + maxPt) / 2;
        obstacleBox.Pose = trvec2tform(centerPosition);

        tempStruct.collisionObject = obstacleBox;
        tempStruct.centroid = centerPosition(:)'; 
        obstacleStructs{validObstacleCount} = tempStruct;
    end
end

obstacleStructs = obstacleStructs(1:validObstacleCount);
disp(['MATLAB (process_point_cloud): 已创建 ', num2str(validObstacleCount), ' 个有效障碍物碰撞体。']);

processedPointCloudForVis = nonGroundCloud;
if nonGroundCloud.Count == 0
     processedPointCloudForVis = pointCloud(zeros(0,3,'single'));
end

if enableGraphics
    disp('MATLAB (process_point_cloud): 正在生成处理后点云的图形...');
    figureHandle = findobj('Type', 'Figure', 'Name', 'Point Cloud Processing Result');
    if isempty(figureHandle)
        figureHandle = figure('Name', 'Point Cloud Processing Result');
    else
        figure(figureHandle); % Bring to front
        clf(figureHandle);   % Clear figure
    end

    pcshow(processedPointCloudForVis); 
    hold on;
    for k_obj = 1:length(obstacleStructs)
        obs_s = obstacleStructs{k_obj}; % obs_s is a struct
        if isfield(obs_s, 'collisionObject') && isa(obs_s.collisionObject, 'collisionBox')
            show(obs_s.collisionObject, 'Parent', gca, 'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k');
        end
    end
    hold off;
    title('处理后的点云数据及识别的障碍物 (AABB)');
    xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
    view(3); 
    axis equal;
    disp('MATLAB (process_point_cloud): 点云处理结果图形已显示/更新。');
else
    disp('MATLAB (process_point_cloud): 图形输出已禁用。');
end
disp('MATLAB (process_point_cloud): 点云处理完成。');
end
