function ros_master_uri = connect_to_ros()
    % connect_to_ros: 尝试连接到ROS Master，并处理现有连接。

    % 默认IP地址 (可以被用户输入覆盖)
    default_ros_ip = '192.168.110.23';
    default_host_ip = '192.168.110.151'; % Windows主机的IP
    default_ros_master_uri = ['http://', default_ros_ip, ':11311'];
    
    % --- 0. 首先尝试使用默认IP地址检查是否已连接到ROS ---
    try
        % 检查是否可以执行ROS命令
        rosnode_list = rosnode('list');
        disp('检测到已有ROS连接，尝试验证连接状态...');
        
        % 尝试获取当前连接的URI
        try
            current_uri = rosmaster.URI;
            disp(['当前已连接到ROS主节点: ', current_uri]);
            ros_master_uri = current_uri;
            return; % 已连接，直接返回
        catch
            % 无法获取URI但可以执行rosnode list，说明连接有效但URI不可用
            disp('已连接到ROS主节点，但无法获取URI。将使用默认URI。');
            ros_master_uri = default_ros_master_uri;
            return; % 已连接，直接返回
        end
    catch
        % 无法执行rosnode list，说明未连接或连接已断开
        disp('未检测到有效的ROS连接，将尝试建立新连接...');
    end

    % --- 1. 从用户获取目标ROS Master的IP地址 ---
    ros_ip_input = input(['请输入Ubuntu VM的IP地址 (默认: ', default_ros_ip, '): '], 's');
    if isempty(ros_ip_input)
        ros_ip = default_ros_ip;
        disp(['使用默认Ubuntu VM IP地址: ', ros_ip]);
    else
        ros_ip = ros_ip_input;
    end

    host_ip_input = input(['请输入Windows主机的IP地址 (默认: ', default_host_ip, '): '], 's');
    if isempty(host_ip_input)
        host_ip = default_host_ip;
        disp(['使用默认Windows主机 IP地址: ', host_ip]);
    else
        host_ip = host_ip_input;
    end
    ros_master_uri_target = ['http://', ros_ip, ':11311'];

    % --- 2. 检查并处理现有的ROS连接 ---
    try
        % 使用内部API检查全局节点状态
        node_instance = matlabshared.ros.internal.GlobalMATLABROSNode.getInstance;
        is_node_valid_and_connected = ~isempty(node_instance) && ...
                                      ~isempty(node_instance.getNode) && ...
                                      node_instance.Connected;

        if is_node_valid_and_connected
            disp('MATLAB已连接到ROS主节点。');
            disp(['将使用目标URI: ', ros_master_uri_target]);
            ros_master_uri = ros_master_uri_target;
            return; % 成功，直接返回
        else
            disp('未检测到活动的、已连接的MATLAB ROS节点。');
            % 如果存在节点实例但未连接，也尝试关闭以确保干净状态
            if ~isempty(node_instance) && ~isempty(node_instance.getNode)
                disp('检测到未连接的节点实例，尝试关闭...');
                rosshutdown;
                pause(2);
            end
            disp('将尝试建立新连接。');
        end
    catch ME_check
        disp(['检查现有ROS连接时发生错误: ', ME_check.message]);
        disp('假设无有效连接或连接状态异常，将尝试重置并建立新连接。');
        try
            rosshutdown; % 尝试关闭以防部分初始化或损坏状态
            pause(2);
        catch ME_shutdown_early
            disp(['在连接检查失败后尝试关闭ROS时出错: ', ME_shutdown_early.message]);
        end
    end

    % --- 3. 尝试初始化新的ROS连接 (带重试机制) ---
    maxAttempts = 3;
    attempt = 0;
    connected = false;
    ros_master_uri = ''; % 初始化返回的URI为空

    while ~connected && attempt < maxAttempts
        attempt = attempt + 1;
        try
            disp(['尝试连接到 ROS Master (', ros_master_uri_target, ') 使用NodeHost (', host_ip, ') (第 ', num2str(attempt), '/', num2str(maxAttempts), ' 次)...']);
            rosinit(ros_master_uri_target, 'NodeHost', host_ip);
            pause(1); % 短暂暂停，让连接状态稳定

            basic_communication_ok = false;
            try
                disp('尝试执行 rosnode list 来验证基本通信...');
                rosnode('list'); % 尝试列出节点，验证与Master的基本通信
                disp('rosnode list 执行成功，基本通信已建立。');
                basic_communication_ok = true;
            catch ME_rosnode_list
                disp(['rosnode list 执行失败: ', ME_rosnode_list.message]);
                % 标记基本通信失败，后续将依赖此状态
            end

            current_master_uri_check = '';
            if basic_communication_ok
                try
                    current_master_uri_check = rosmaster.URI; % 尝试获取Master URI
                    if isempty(current_master_uri_check)
                        disp('警告: rosmaster.URI 为空，尽管 rosnode list 成功。');
                    end
                catch ME_get_uri
                    disp(['获取 rosmaster.URI 时发生警告: ', ME_get_uri.message]);
                    % current_master_uri_check 保持为空
                end
            end

            if ~isempty(current_master_uri_check)
                disp(['成功连接到ROS主节点: ', current_master_uri_check]);
                ros_master_uri = current_master_uri_check;
                connected = true;
            elseif basic_communication_ok % AND isempty(current_master_uri_check) is implied
                disp('警告: 基本通信 (rosnode list) 成功，但未能获取或验证 rosmaster.URI。');
                disp(['将使用目标URI (', ros_master_uri_target, ') 作为连接参考。某些依赖特定URI的功能可能受限。']);
                ros_master_uri = ros_master_uri_target; % 使用目标URI作为备用
                connected = true; % 认为连接在某种程度上是成功的
            else % basic_communication_ok is false
                % 此处意味着 rosnode list 失败，连接未建立
                error('MATLAB:ROS:ConnectionFailed', 'rosinit 调用后，rosnode list 失败，无法与 ROS Master 建立基本通信。');
            end

        catch ex
            disp(['连接失败 (第 ', num2str(attempt), ' 次): ', ex.message]);
            
            disp('尝试关闭任何可能存在的ROS连接以准备重试...');
            try
                rosshutdown;
                pause(2); % 等待关闭完成
            catch ME_shutdown_retry
                disp(['在重试前关闭ROS时出错: ', ME_shutdown_retry.message]);
            end

            if attempt < maxAttempts
                disp(['等待5秒后重试连接到 (', ros_master_uri_target, ')...']);
                pause(5);
            else
                disp('已达到最大尝试次数，连接失败。');
                ros_master_uri = ''; % 确保在最终失败时返回空
            end
        end
    end
end