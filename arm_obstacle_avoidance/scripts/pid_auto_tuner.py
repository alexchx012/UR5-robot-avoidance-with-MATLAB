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
from scipy.optimize import differential_evolution


class PIDAutoTunerDynamic:
    def __init__(self):
        rospy.init_node('pid_auto_tuner_dynamic', anonymous=True)

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
        self.pid_bounds = {
            'shoulder_pan_joint': (1.0, 100.0, 0.001, 1.0, 0.1, 10.0),
            'shoulder_lift_joint': (10.0, 200.0, 0.01, 2.0, 0.5, 20.0),
            'elbow_joint': (1.0, 80.0, 0.001, 0.8, 0.1, 8.0),
            'wrist_1_joint': (1.0, 60.0, 0.001, 0.6, 0.1, 6.0),
            'wrist_2_joint': (1.0, 60.0, 0.001, 0.6, 0.1, 6.0),
            'wrist_3_joint': (1.0, 40.0, 0.001, 0.4, 0.1, 4.0)
        }

        # Dynamic Reconfigure 客户端字典
        self.dyn_clients = {}
        
        # 配置文件路径
        self.config_path = os.path.expanduser(
            '~/catkin_ws/src/arm_obstacle_avoidance/config/arm_controllers.yaml'
        )
        
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

        rospy.loginfo("基于Dynamic Reconfigure的PID自动调优器初始化完成")

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

    def save_best_params_to_yaml(self):
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
                'best_performance_score': float(self.best_performance),
                'optimization_method': 'Dynamic Reconfigure Auto-Tuning'
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
        """优化目标函数"""
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

        # 等待参数生效（Dynamic Reconfigure响应更快）
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

    def optimize_pid(self, max_iterations=25):
        """使用差分进化算法优化PID参数"""
        rospy.loginfo("开始基于Dynamic Reconfigure的PID参数优化...")
        
        # 设置优化状态
        self.optimization_in_progress = True
        self.best_params = None
        self.best_performance = float('inf')

        # 构建参数边界
        bounds = []
        for joint_name in self.joint_names:
            joint_bounds = self.pid_bounds[joint_name]
            bounds.extend([
                (joint_bounds[0], joint_bounds[1]),  # P增益边界
                (joint_bounds[2], joint_bounds[3]),  # I增益边界
                (joint_bounds[4], joint_bounds[5])   # D增益边界
            ])

        # 运行差分进化优化
        try:
            print("\n=== 开始PID参数优化 ===")
            print("提示: 按 Ctrl+C 可随时中断优化并保存当前最优参数")
            print("="*50)
            
            result = differential_evolution(
                self.objective_function,
                bounds,
                maxiter=max_iterations,
                popsize=12,        # 适中的种群大小
                mutation=(0.5, 1.2),  # 调整变异参数
                recombination=0.8,    # 调整重组参数
                seed=None,            # 随机种子
                disp=True,
                atol=0.01,           # 收敛容差
                tol=0.001,
                workers=1            # 单线程避免并发问题
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
                if self.best_params is not None:
                    self.save_best_params_to_yaml()
                return self.best_params

        except KeyboardInterrupt:
            # 这里通常不会执行，因为信号处理器会先处理
            rospy.loginfo("优化被用户中断")
            self.optimization_in_progress = False
            return self.best_params
        except Exception as e:
            rospy.logerr(f"优化过程出错: {e}")
            self.optimization_in_progress = False
            return self.best_params

    def manual_tune_joint(self, joint_name, p_range=(1.0, 50.0), i_range=(0.001, 0.1), d_range=(0.1, 2.0), steps=4):
        """手动调优单个关节"""
        rospy.loginfo(f"开始手动调优关节: {joint_name}")

        if joint_name not in self.dyn_clients:
            rospy.logerr(f"未找到 {joint_name} 的Dynamic Reconfigure客户端")
            return None

        best_performance = float('inf')
        best_params = None

        # 网格搜索
        p_values = np.linspace(p_range[0], p_range[1], steps)
        i_values = np.linspace(i_range[0], i_range[1], steps)
        d_values = np.linspace(d_range[0], d_range[1], steps)

        total_tests = len(p_values) * len(i_values) * len(d_values)
        test_count = 0

        for p in p_values:
            for i in i_values:
                for d in d_values:
                    test_count += 1
                    rospy.loginfo(
                        f"测试 {test_count}/{total_tests}: P={p:.2f}, I={i:.4f}, D={d:.2f}")

                    # 创建测试参数
                    test_params = {joint_name: {'p': p, 'i': i, 'd': d}}

                    # 更新参数并测试
                    if self.update_pid_parameters_dynamic(test_params):
                        rospy.sleep(0.3)  # Dynamic Reconfigure响应更快
                        performance = self.evaluate_performance(test_duration=4.0)

                        if performance < best_performance:
                            best_performance = performance
                            best_params = {'p': p, 'i': i, 'd': d}
                            rospy.loginfo(
                                f"发现更好参数: P={p:.2f}, I={i:.4f}, D={d:.2f}, 性能={performance:.4f}")

        if best_params:
            # 应用最佳参数
            final_params = {joint_name: best_params}
            self.update_pid_parameters_dynamic(final_params)
            
            rospy.loginfo(
                f"{joint_name} 最优参数: P={best_params['p']:.2f}, I={best_params['i']:.4f}, D={best_params['d']:.2f}")
            return best_params
        else:
            rospy.logwarn(f"{joint_name} 调优失败")
            return None

    def get_current_pid_parameters(self):
        """获取当前PID参数"""
        current_params = {}
        
        for joint_name in self.joint_names:
            if joint_name in self.dyn_clients:
                try:
                    config = self.dyn_clients[joint_name].get_configuration()
                    current_params[joint_name] = {
                        'p': config.get('p', 0.0),
                        'i': config.get('i', 0.0),
                        'd': config.get('d', 0.0)
                    }
                except Exception as e:
                    rospy.logwarn(f"获取 {joint_name} 当前参数失败: {e}")
        
        return current_params

    def test_dynamic_reconfigure_connection(self):
        """测试Dynamic Reconfigure连接"""
        rospy.loginfo("测试Dynamic Reconfigure连接...")
        
        for joint_name, client in self.dyn_clients.items():
            try:
                # 获取当前配置
                config = client.get_configuration()
                rospy.loginfo(f"{joint_name} 当前PID: P={config.get('p', 'N/A')}, I={config.get('i', 'N/A')}, D={config.get('d', 'N/A')}")
                
                # 测试更新（使用当前值）
                test_config = {
                    'p': config.get('p', 1.0),
                    'i': config.get('i', 0.01),
                    'd': config.get('d', 0.1)
                }
                client.update_configuration(test_config)
                rospy.loginfo(f"{joint_name} Dynamic Reconfigure连接正常")
                
            except Exception as e:
                rospy.logerr(f"{joint_name} Dynamic Reconfigure连接失败: {e}")


def main():
    try:
        tuner = PIDAutoTunerDynamic()

        # 等待系统稳定
        rospy.sleep(2.0)
        
        # 测试连接
        tuner.test_dynamic_reconfigure_connection()

        print("\n=== 基于Dynamic Reconfigure的PID自动调优程序 ===")
        print("1. 全自动优化（推荐）")
        print("2. 单关节手动调优")
        print("3. 性能测试")
        print("4. 显示当前PID参数")
        print("5. 测试Dynamic Reconfigure连接")

        choice = input("请选择模式 (1-5): ")

        if choice == '1':
            # 全自动优化
            rospy.loginfo("开始全自动PID优化...")
            optimal_params = tuner.optimize_pid(max_iterations=20)

            if optimal_params:
                print("\n优化完成！最优参数已实时应用并保存到配置文件。")
                print("无需重启仿真环境，参数已生效。")
                print(f"最优性能得分: {tuner.best_performance:.4f}")
            else:
                print("优化失败，请检查系统状态。")

        elif choice == '2':
            # 单关节调优
            print("\n可调优的关节:")
            for i, joint in enumerate(tuner.joint_names):
                print(f"{i+1}. {joint}")

            joint_idx = int(input("选择关节编号: ")) - 1
            if 0 <= joint_idx < len(tuner.joint_names):
                joint_name = tuner.joint_names[joint_idx]
                bounds = tuner.pid_bounds[joint_name]

                best_params = tuner.manual_tune_joint(
                    joint_name,
                    p_range=(bounds[0], bounds[1]),
                    i_range=(bounds[2], bounds[3]),
                    d_range=(bounds[4], bounds[5]),
                    steps=3
                )

                if best_params:
                    print(f"\n{joint_name} 最优参数:")
                    print(f"P: {best_params['p']:.3f}")
                    print(f"I: {best_params['i']:.4f}")
                    print(f"D: {best_params['d']:.3f}")
                    print("参数已实时应用。")

        elif choice == '3':
            # 性能测试
            rospy.loginfo("开始性能测试...")
            performance = tuner.evaluate_performance(test_duration=10.0)
            print(f"\n当前PID参数性能得分: {performance:.4f}")
            print("(得分越低表示性能越好)")

        elif choice == '4':
            # 显示当前参数
            current_params = tuner.get_current_pid_parameters()
            print("\n当前PID参数:")
            for joint_name, params in current_params.items():
                print(f"{joint_name}: P={params['p']:.3f}, I={params['i']:.4f}, D={params['d']:.3f}")

        elif choice == '5':
            # 测试连接
            tuner.test_dynamic_reconfigure_connection()
            print("\n连接测试完成，请查看日志输出。")

        else:
            print("无效选择")

    except KeyboardInterrupt:
        rospy.loginfo("用户中断程序")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")


if __name__ == '__main__':
    main()
