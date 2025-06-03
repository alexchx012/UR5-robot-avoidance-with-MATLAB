#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import time
import signal
import sys
import yaml
import os
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import dynamic_reconfigure.client
from threading import Lock

# 贝叶斯优化相关导入
try:
    from skopt import gp_minimize
    from skopt.space import Real
    from skopt.utils import use_named_args
    BAYESIAN_AVAILABLE = True
except ImportError:
    print("警告: scikit-optimize 未安装，请运行: pip install scikit-optimize")
    BAYESIAN_AVAILABLE = False


class PIDAutoTunerBayesian:
    def __init__(self):
        rospy.init_node('pid_auto_tuner_bayesian', anonymous=True)

        # 机械臂关节名称
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # 当前关节状态
        self.current_joint_states = {}
        self.target_joint_states = {}
        self.joint_errors = {}
        self.data_lock = Lock()

        # 性能评估数据
        self.error_history = []
        self.time_history = []
        self.start_time = None

        # 最优参数跟踪
        self.best_params = None
        self.best_performance = float('inf')
        self.optimization_in_progress = False

        # PID参数边界 (p_min, p_max, i_min, i_max, d_min, d_max)
        # 改进的PID参数边界
        self.pid_bounds = {
            'shoulder_pan_joint': (5.0, 80.0, 0.01, 0.8, 0.5, 8.0),
            'shoulder_lift_joint': (20.0, 150.0, 0.05, 1.5, 1.0, 15.0),
            'elbow_joint': (10.0, 100.0, 0.01, 0.5, 0.5, 10.0),
            'wrist_1_joint': (5.0, 80.0, 0.01, 0.8, 0.5, 8.0),
            'wrist_2_joint': (5.0, 80.0, 0.01, 0.8, 0.5, 8.0),
            'wrist_3_joint': (3.0, 60.0, 0.01, 0.6, 0.3, 6.0)
        }

        # Dynamic Reconfigure 客户端字典
        self.dyn_clients = {}
        
        # 配置文件路径
        self.config_path = os.path.expanduser(
            '~/catkin_ws/src/arm_obstacle_avoidance/config/arm_controllers.yaml'
        )
        
        # 贝叶斯优化相关
        self.optimization_space = None
        self.param_names = []
        self.setup_bayesian_space()
        
        # 初始化dynamic_reconfigure客户端
        self.init_dynamic_reconfigure_clients()

        # ROS订阅和发布
        self.joint_state_sub = rospy.Subscriber(
            '/joint_states', JointState, self.joint_state_callback
        )

        self.controller_state_sub = rospy.Subscriber(
            '/custom_arm/arm_controller/state',
            JointTrajectoryControllerState,
            self.controller_state_callback
        )

        self.trajectory_pub = rospy.Publisher(
            '/custom_arm/arm_controller/command',
            JointTrajectory,
            queue_size=1
        )

        # 设置信号处理器
        self.setup_signal_handlers()

        rospy.loginfo("PID调优器初始化完成")

    def setup_bayesian_space(self):
        """设置贝叶斯优化的搜索空间"""
        if not BAYESIAN_AVAILABLE:
            rospy.logerr("贝叶斯优化库不可用")
            return
            
        self.optimization_space = []
        self.param_names = []
        
        for joint_name in self.joint_names:
            bounds = self.pid_bounds[joint_name]
            
            # P增益
            self.optimization_space.append(Real(bounds[0], bounds[1], name=f'{joint_name}_p'))
            self.param_names.append(f'{joint_name}_p')
            
            # I增益
            self.optimization_space.append(Real(bounds[2], bounds[3], name=f'{joint_name}_i'))
            self.param_names.append(f'{joint_name}_i')
            
            # D增益
            self.optimization_space.append(Real(bounds[4], bounds[5], name=f'{joint_name}_d'))
            self.param_names.append(f'{joint_name}_d')
        
        rospy.loginfo(f"贝叶斯优化搜索空间已设置，共{len(self.optimization_space)}个参数")

    def setup_signal_handlers(self):
        """设置信号处理器"""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        rospy.loginfo("信号处理器已设置")

    def signal_handler(self, signum, frame):
        """信号处理函数 - 处理Ctrl+C"""
        rospy.logwarn("\n检测到中断信号，正在保存最优PID参数...")
        
        if self.optimization_in_progress and self.best_params is not None:
            try:
                # 保存最优参数到YAML文件
                self.save_best_params_to_yaml()
                rospy.loginfo("最优PID参数已保存到配置文件")
                
                # 应用最优参数到Dynamic Reconfigure
                self.update_pid_parameters_dynamic(self.best_params)
                rospy.loginfo("最优PID参数已应用到系统")
                
                print(f"\n=== 优化中断，已保存最优参数 ===")
                print(f"最优性能得分: {self.best_performance:.4f}")
                print("最优PID参数:")
                for joint_name, params in self.best_params.items():
                    print(f"{joint_name}: P={params['p']:.3f}, I={params['i']:.4f}, D={params['d']:.3f}")
                print("参数已保存到配置文件并应用到系统。")
                
            except Exception as e:
                rospy.logerr(f"保存参数时出错: {e}")
        else:
            rospy.loginfo("没有找到最优参数或优化未在进行中")
        
        rospy.loginfo("程序正在安全退出...")
        rospy.signal_shutdown("用户中断")
        sys.exit(0)

    def save_best_params_to_yaml(self, method="Manual Tuning"):
        """保存最优PID参数到YAML配置文件"""
        if self.best_params is None:
            rospy.logwarn("没有最优参数可保存")
            return False
            
        try:
            # 读取当前配置
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
            else:
                config = {}

            # 更新PID参数
            if '/custom_arm/gazebo_ros_control/pid_gains' not in config:
                config['/custom_arm/gazebo_ros_control/pid_gains'] = {}

            for joint_name, params in self.best_params.items():
                config['/custom_arm/gazebo_ros_control/pid_gains'][joint_name] = {
                    'p': float(params['p']),
                    'i': float(params['i']),
                    'd': float(params['d'])
                }

            # 添加优化信息注释
            config['# PID_OPTIMIZATION_INFO'] = {
                'optimization_date': time.strftime('%Y-%m-%d %H:%M:%S'),
                'best_performance_score': float(self.best_performance) if hasattr(self, 'best_performance') else 'Unknown',
                'optimization_method': method
            }

            # 写回配置文件
            with open(self.config_path, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

            rospy.loginfo(f"最优PID参数已保存到: {self.config_path}")
            return True

        except Exception as e:
            rospy.logerr(f"保存PID参数到YAML文件失败: {e}")
            return False

    def init_dynamic_reconfigure_clients(self):
        """初始化dynamic_reconfigure客户端"""
        rospy.loginfo("正在初始化Dynamic Reconfigure客户端...")
        
        for joint_name in self.joint_names:
            try:
                # 构建dynamic_reconfigure服务名称
                service_name = f'/custom_arm/gazebo_ros_control/pid_gains/{joint_name}'
                
                # 创建客户端
                client = dynamic_reconfigure.client.Client(service_name, timeout=5.0)
                self.dyn_clients[joint_name] = client
                
                rospy.loginfo(f"成功连接到 {joint_name} 的Dynamic Reconfigure服务")
                
            except Exception as e:
                rospy.logerr(f"连接 {joint_name} 的Dynamic Reconfigure服务失败: {e}")
                # 尝试备用服务名称格式
                try:
                    alt_service_name = f'/custom_arm/gazebo_ros_control/{joint_name}_pid'
                    client = dynamic_reconfigure.client.Client(alt_service_name, timeout=5.0)
                    self.dyn_clients[joint_name] = client
                    rospy.loginfo(f"使用备用服务名称成功连接到 {joint_name}")
                except Exception as e2:
                    rospy.logerr(f"备用连接也失败: {e2}")
        
        rospy.loginfo(f"成功初始化 {len(self.dyn_clients)} 个Dynamic Reconfigure客户端")

    def joint_state_callback(self, msg):
        """关节状态回调函数"""
        with self.data_lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.current_joint_states[name] = msg.position[i]

    def controller_state_callback(self, msg):
        """控制器状态回调函数"""
        with self.data_lock:
            if len(msg.joint_names) == len(msg.actual.positions):
                for i, name in enumerate(msg.joint_names):
                    if name in self.joint_names:
                        actual_pos = msg.actual.positions[i]
                        desired_pos = msg.desired.positions[i]
                        error = abs(desired_pos - actual_pos)
                        self.joint_errors[name] = error

                        # 记录误差历史
                        if self.start_time is not None:
                            current_time = time.time() - self.start_time
                            self.time_history.append(current_time)
                            self.error_history.append(sum(self.joint_errors.values()))

    def update_pid_parameters_dynamic(self, pid_params):
        """使用Dynamic Reconfigure实时更新PID参数"""
        success_count = 0
        
        for joint_name, params in pid_params.items():
            if joint_name in self.dyn_clients:
                try:
                    # 构建参数字典
                    config = {
                        'p': float(params['p']),
                        'i': float(params['i']),
                        'd': float(params['d'])
                    }
                    
                    # 实时更新参数
                    self.dyn_clients[joint_name].update_configuration(config)
                    success_count += 1
                    
                    rospy.logdebug(f"成功更新 {joint_name}: P={params['p']:.3f}, I={params['i']:.4f}, D={params['d']:.3f}")
                    
                except Exception as e:
                    rospy.logerr(f"更新 {joint_name} PID参数失败: {e}")
            else:
                rospy.logwarn(f"未找到 {joint_name} 的Dynamic Reconfigure客户端")
        
        if success_count > 0:
            rospy.loginfo(f"成功更新 {success_count}/{len(pid_params)} 个关节的PID参数")
            return True
        else:
            rospy.logerr("所有PID参数更新失败")
            return False

    def send_test_trajectory(self, target_positions=None):
        """发送测试轨迹"""
        if target_positions is None:
            # 默认测试位置：机械臂抬起姿态
            target_positions = {
                'shoulder_pan_joint': 0.0,
                'shoulder_lift_joint': -1.57,  # 向上抬起
                'elbow_joint': 1.57,
                'wrist_1_joint': -1.57,
                'wrist_2_joint': 0.0,
                'wrist_3_joint': 0.0
            }

        # 创建轨迹消息
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = self.joint_names

        # 添加轨迹点
        point = JointTrajectoryPoint()
        point.positions = [target_positions[name] for name in self.joint_names]
        point.time_from_start = rospy.Duration(3.0)  # 3秒内到达
        trajectory.points.append(point)

        # 发布轨迹
        self.trajectory_pub.publish(trajectory)

        # 保存目标位置
        with self.data_lock:
            self.target_joint_states = target_positions.copy()

        rospy.loginfo("测试轨迹已发送")
        
    def send_initial_pose(self):
        """发送初始姿态，将机械臂抬起10度，shoulder_pan_joint保持在0度"""
        # 初始姿态：所有关节伸展开，以地平面为x轴将机械臂抬起10度
        initial_positions = {
            'shoulder_pan_joint': 0.0,  # 0度，不旋转
            'shoulder_lift_joint': -0.17,  # 约10度，弧度制
            'elbow_joint': 0.0,
            'wrist_1_joint': 0.0,
            'wrist_2_joint': 0.0,
            'wrist_3_joint': 0.0
        }
        
        self.send_test_trajectory(initial_positions)
        rospy.loginfo("机械臂已设置到初始姿态，抬起10度，shoulder_pan_joint在0度位置")
        
    def send_test_pose(self):
        """发送测试姿态，将机械臂抬起170度，同时将shoulder_pan_joint旋转到90度"""
        # 测试姿态：以地平面为x轴整个抬起170度，同时旋转shoulder_pan_joint到90度
        test_positions = {
            'shoulder_pan_joint': 1.57,  # 约90度，弧度制
            'shoulder_lift_joint': -2.97,  # 约170度，弧度制
            'elbow_joint': 0.0,
            'wrist_1_joint': 0.0,
            'wrist_2_joint': 0.0,
            'wrist_3_joint': 0.0
        }
        
        self.send_test_trajectory(test_positions)
        rospy.loginfo("机械臂已设置到测试姿态，抬起170度，shoulder_pan_joint旋转到90度")

    def tune_pid_parameter(self, joint_name, param_type, current_value):
        """调节单个PID参数"""
        param_names = {'p': 'P (比例增益)', 'i': 'I (积分增益)', 'd': 'D (微分增益)'}
        
        while True:
            print(f"\n=== 调节 {joint_name} 的 {param_names[param_type]} ===")
            print(f"当前值: {current_value:.6f}")
            print("1. 请输入要修改的数值")
            print("2. 返回上一级")
            
            choice = input("请选择 (1-2): ")
            
            if choice == '2':
                break
                
            elif choice == '1':
                try:
                    new_value = float(input(f"请输入新的{param_names[param_type]}值: "))
                    
                    # 更新参数
                    update_config = {param_type: new_value}
                    self.dyn_clients[joint_name].update_configuration(update_config)
                    current_value = new_value
                    
                    print(f"{param_names[param_type]}已更新为: {new_value:.6f}")
                    
                    # 更新最佳参数并保存到YAML文件
                    if self.best_params is None:
                        self.best_params = {}
                        for jname in self.joint_names:
                            try:
                                config = self.dyn_clients[jname].get_configuration()
                                self.best_params[jname] = {
                                    'p': config['p'],
                                    'i': config['i'],
                                    'd': config['d']
                                }
                            except Exception as e:
                                rospy.logwarn(f"获取 {jname} 参数失败: {e}")
                    else:
                        # 确保joint_name存在于best_params中
                        if joint_name not in self.best_params:
                            try:
                                config = self.dyn_clients[joint_name].get_configuration()
                                self.best_params[joint_name] = {
                                    'p': config['p'],
                                    'i': config['i'],
                                    'd': config['d']
                                }
                            except Exception as e:
                                rospy.logwarn(f"获取 {joint_name} 参数失败: {e}")
                        
                        # 更新特定参数
                        self.best_params[joint_name][param_type] = new_value
                    
                    # 保存到YAML文件
                    try:
                        self.save_best_params_to_yaml()
                        print("参数已实时保存到配置文件")
                    except Exception as e:
                        print(f"保存参数失败: {e}")
                    
                    # 每次更新参数后都进行测试
                    # 先发送初始姿态（10度，shoulder_pan_joint在0度）
                    print("正在将机械臂移动到初始位置（10度，shoulder_pan_joint在0度）...")
                    self.send_initial_pose()
                    rospy.sleep(3.0)  # 等待机械臂到达初始位置
                    
                    # 然后发送测试姿态（170度，shoulder_pan_joint旋转到90度）
                    print("正在进行测试，将机械臂移动到测试位置（170度，shoulder_pan_joint旋转到90度）...")
                    self.send_test_pose()
                    rospy.sleep(5.0)  # 等待机械臂完成测试动作
                    
                    # 最后再回到初始姿态，为下一次测试做准备
                    print("测试完成，正在恢复到初始位置...")
                    self.send_initial_pose()
                    rospy.sleep(3.0)  # 等待机械臂回到初始位置
                    
                except ValueError:
                    print("输入无效，请输入一个数字")
                except Exception as e:
                    print(f"更新参数失败: {e}")
            else:
                print("无效选择，请重试")

    def test_dynamic_reconfigure_connection(self):
        """测试与Dynamic Reconfigure服务器的连接状态"""
        rospy.loginfo("正在测试Dynamic Reconfigure连接...")
        success_count = 0
        
        for joint_name, client in self.dyn_clients.items():
            try:
                # 尝试获取当前配置来测试连接
                config = client.get_configuration()
                rospy.loginfo(f"成功连接到 {joint_name} 的Dynamic Reconfigure服务")
                rospy.loginfo(f"当前参数: P={config['p']:.3f}, I={config['i']:.4f}, D={config['d']:.3f}")
                success_count += 1
            except Exception as e:
                rospy.logerr(f"连接 {joint_name} 的Dynamic Reconfigure服务失败: {e}")
        
        if success_count == len(self.joint_names):
            rospy.loginfo(f"所有 {success_count} 个Dynamic Reconfigure连接测试成功")
        else:
            rospy.logwarn(f"仅 {success_count}/{len(self.joint_names)} 个Dynamic Reconfigure连接测试成功")
        
        return success_count == len(self.joint_names)

    def manual_tune_pid(self):
        """手动调节PID参数"""
        while True:
            print("\n=== 手动PID调节 ===")
            print("请选择要调节的关节:")
            
            for i, joint_name in enumerate(self.joint_names, 1):
                print(f"{i}. {joint_name}")
            print(f"{len(self.joint_names) + 1}. 返回主菜单")
            
            try:
                choice = int(input(f"请选择 (1-{len(self.joint_names) + 1}): "))
                
                if choice == len(self.joint_names) + 1:
                    break
                    
                if 1 <= choice <= len(self.joint_names):
                    joint_name = self.joint_names[choice - 1]
                    self.tune_joint_pid(joint_name)
                else:
                    print("无效选择，请重试")
                    
            except ValueError:
                print("请输入一个有效的数字")
            except Exception as e:
                print(f"发生错误: {e}")
    
    def tune_joint_pid(self, joint_name):
        """调节单个关节的PID参数"""
        try:
            # 获取当前参数
            current_config = self.dyn_clients[joint_name].get_configuration()
            
            while True:
                print(f"\n=== 调节 {joint_name} 的PID参数 ===")
                print(f"当前参数: P={current_config['p']:.6f}, I={current_config['i']:.6f}, D={current_config['d']:.6f}")
                print("1. 调节P (比例增益)")
                print("2. 调节I (积分增益)")
                print("3. 调节D (微分增益)")
                print("4. 返回上一级")
                
                choice = input("请选择 (1-4): ")
                
                if choice == '4':
                    break
                    
                if choice == '1':
                    self.tune_pid_parameter(joint_name, 'p', current_config['p'])
                    # 更新当前配置
                    current_config = self.dyn_clients[joint_name].get_configuration()
                elif choice == '2':
                    self.tune_pid_parameter(joint_name, 'i', current_config['i'])
                    # 更新当前配置
                    current_config = self.dyn_clients[joint_name].get_configuration()
                elif choice == '3':
                    self.tune_pid_parameter(joint_name, 'd', current_config['d'])
                    # 更新当前配置
                    current_config = self.dyn_clients[joint_name].get_configuration()
                else:
                    print("无效选择，请重试")
                    
        except Exception as e:
            print(f"调节 {joint_name} 参数时出错: {e}")

    def get_current_pid_parameters(self):
        """获取当前PID参数"""
        return self.best_params

    def evaluate_performance(self, test_duration=8.0):
        """性能评估函数 - 测试多个目标位置"""
        # 测试多个目标位置
        test_positions = [
            # 竖直姿态
            {'shoulder_lift_joint': -1.57, 'elbow_joint': 1.57},
            # 水平姿态  
            {'shoulder_lift_joint': 0.0, 'elbow_joint': 0.0},
            # 中间姿态
            {'shoulder_lift_joint': -0.785, 'elbow_joint': 0.785}
        ]
        
        total_performance = 0
        rospy.loginfo(f"开始多位置性能评估，测试{len(test_positions)}个目标位置")
        
        for i, pos in enumerate(test_positions):
            # 为每个位置创建完整的目标位置字典
            target_positions = {
                'shoulder_pan_joint': 0.0,
                'shoulder_lift_joint': pos.get('shoulder_lift_joint', -1.57),
                'elbow_joint': pos.get('elbow_joint', 1.57),
                'wrist_1_joint': -1.57,
                'wrist_2_joint': 0.0,
                'wrist_3_joint': 0.0
            }
            
            rospy.loginfo(f"测试位置 {i+1}/{len(test_positions)}: shoulder_lift={pos['shoulder_lift_joint']:.3f}, elbow={pos['elbow_joint']:.3f}")
            
            # 清空历史数据
            with self.data_lock:
                self.error_history = []
                self.time_history = []
                self.joint_errors = {}
            
            # 发送测试轨迹并评估
            self.send_test_trajectory(target_positions)
            self.start_time = time.time()
            rospy.sleep(test_duration)
            
            if len(self.error_history) > 0:
                avg_error = np.mean(self.error_history)
                steady_error = np.mean(self.error_history[-10:]) if len(self.error_history) >= 10 else avg_error
                max_error = np.max(self.error_history)
                
                # 加权性能指标
                performance = avg_error + 2.5 * steady_error + 0.4 * max_error
                total_performance += performance
                
                rospy.loginfo(f"位置 {i+1} - 平均误差: {avg_error:.4f}, 稳态误差: {steady_error:.4f}, 最大误差: {max_error:.4f}")
                rospy.loginfo(f"位置 {i+1} 性能得分: {performance:.4f}")
            else:
                rospy.logwarn(f"位置 {i+1} 无有效数据")
                total_performance += float('inf')
        
        final_score = total_performance / len(test_positions)
        rospy.loginfo(f"综合性能得分: {final_score:.4f}")
        
        return final_score

    def objective_function(self, params_vector):
        """贝叶斯优化的目标函数"""
        # 将参数向量转换为PID参数字典
        pid_params = {}
        param_idx = 0

        for joint_name in self.joint_names:
            pid_params[joint_name] = {
                'p': params_vector[param_idx],
                'i': params_vector[param_idx + 1],
                'd': params_vector[param_idx + 2]
            }
            param_idx += 3

        # 使用Dynamic Reconfigure实时更新PID参数
        if not self.update_pid_parameters_dynamic(pid_params):
            return float('inf')

        # 等待参数生效
        rospy.sleep(0.5)

        # 评估性能
        performance = self.evaluate_performance(test_duration=6.0)

        # 更新最优参数
        if performance < self.best_performance:
            self.best_performance = performance
            self.best_params = pid_params.copy()
            rospy.loginfo(f"发现更优参数！性能得分: {performance:.4f}")
            
            # 实时保存最优参数（可选）
            try:
                self.save_best_params_to_yaml()
            except Exception as e:
                rospy.logwarn(f"实时保存参数失败: {e}")

        rospy.loginfo(f"当前参数组合性能: {performance:.4f}")
        return performance

    def optimize_pid(self, n_calls=30, n_initial_points=10):
        """使用贝叶斯优化算法优化PID参数"""
        rospy.loginfo("开始基于贝叶斯优化的PID参数调优...")
        
        # 设置优化状态
        self.optimization_in_progress = True
        self.best_params = None
        self.best_performance = float('inf')

        # 构建贝叶斯优化的搜索空间
        search_space = self.setup_bayesian_space()

        # 运行贝叶斯优化
        try:
            print("\n=== 开始贝叶斯PID参数优化 ===")
            print(f"优化次数: {n_calls}, 初始随机点数: {n_initial_points}")
            print("提示: 按 Ctrl+C 可随时中断优化并保存当前最优参数")
            print("="*50)
            
            # 使用scikit-optimize的贝叶斯优化
            result = skopt.gp_minimize(
                self.objective_function,
                search_space,
                n_calls=n_calls,
                n_initial_points=n_initial_points,
                verbose=True,
                callback=lambda res: rospy.loginfo(f"当前最优: {res.fun:.4f}")
            )

            # 优化完成
            self.optimization_in_progress = False

            if result.success:
                rospy.loginfo(f"优化成功！最优性能: {result.fun:.4f}")

                # 解析最优参数
                optimal_params = {}
                param_idx = 0
                for joint_name in self.joint_names:
                    optimal_params[joint_name] = {
                        'p': result.x[param_idx],
                        'i': result.x[param_idx + 1],
                        'd': result.x[param_idx + 2]
                    }
                    param_idx += 3

                # 更新最优参数
                self.best_params = optimal_params
                self.best_performance = result.fun

                # 应用最优参数
                self.update_pid_parameters_dynamic(optimal_params)
                
                # 保存最优参数到YAML文件
                self.save_best_params_to_yaml()

                # 打印最优参数
                rospy.loginfo("最优PID参数:")
                for joint_name, params in optimal_params.items():
                    rospy.loginfo(
                        f"{joint_name}: P={params['p']:.3f}, I={params['i']:.4f}, D={params['d']:.3f}")

                return optimal_params
            else:
                rospy.logwarn("优化未收敛，使用当前最佳参数")
                # 在optimize_pid方法中的保存调用
                self.save_best_params_to_yaml(method="Bayesian Optimization")
                
                # 在objective_function方法中的保存调用
                self.save_best_params_to_yaml(method="Bayesian Optimization")

        except KeyboardInterrupt:
            rospy.loginfo("优化被用户中断")
            self.optimization_in_progress = False
            return self.best_params
        except Exception as e:
            rospy.logerr(f"优化过程出错: {e}")
            self.optimization_in_progress = False
            return self.best_params
        except Exception as e:
            rospy.logerr(f"优化过程出错: {e}")
            self.optimization_in_progress = False
            return self.best_params
            
def main():
    try:
        tuner = PIDAutoTunerBayesian()

        # 等待系统稳定
        rospy.sleep(2.0)
        
        # 测试连接
        tuner.test_dynamic_reconfigure_connection()

        while True:
            print("\n=== PID参数调优程序 ===")
            print("1. 手动调节PID")
            print("2. 自动调节PID")
            print("3. 性能测试")
            print("4. 测试Dynamic Reconfigure连接")
            print("5. 显示当前PID参数")
            print("6. 退出程序")

            choice = input("请选择模式 (1-6): ")

            if choice == '1':
                # 手动调节PID
                tuner.manual_tune_pid()
                
            elif choice == '2':
                # 自动调节PID
                if not BAYESIAN_AVAILABLE:
                    print("错误: 贝叶斯优化库未安装")
                    print("请运行以下命令安装:")
                    print("pip install scikit-optimize")
                    continue
                    
                rospy.loginfo("开始贝叶斯PID优化...")
                
                # 用户可以自定义优化参数
                try:
                    n_calls = int(input("输入总优化次数 (默认30): ") or "30")
                    n_initial = int(input("输入初始随机点数 (默认10): ") or "10")
                except ValueError:
                    n_calls, n_initial = 30, 10
                    
                optimal_params = tuner.optimize_pid(n_calls=n_calls, n_initial_points=n_initial)

                if optimal_params:
                    print("\n贝叶斯优化完成！最优参数已实时应用并保存到配置文件。")
                    print("无需重启仿真环境，参数已生效。")
                    print(f"最优性能得分: {tuner.best_performance:.4f}")
                else:
                    print("优化失败，请检查系统状态。")

            elif choice == '3':
                # 性能测试
                rospy.loginfo("开始性能测试...")
                performance = tuner.evaluate_performance(test_duration=10.0)
                print(f"\n当前PID参数性能得分: {performance:.4f}")
                print("(得分越低表示性能越好)")

            elif choice == '4':
                # 测试连接
                tuner.test_dynamic_reconfigure_connection()
                print("\n连接测试完成，请查看日志输出。")

            elif choice == '5':
                # 显示当前参数
                current_params = tuner.get_current_pid_parameters()
                print("\n当前PID参数:")
                for joint_name, params in current_params.items():
                    print(f"{joint_name}: P={params['p']:.3f}, I={params['i']:.4f}, D={params['d']:.3f}")
                    
            elif choice == '6':
                # 退出程序
                print("正在退出程序...")
                break

            else:
                print("无效选择，请重试")

    except KeyboardInterrupt:
        rospy.loginfo("用户中断程序")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")


if __name__ == '__main__':
    main()

    


    