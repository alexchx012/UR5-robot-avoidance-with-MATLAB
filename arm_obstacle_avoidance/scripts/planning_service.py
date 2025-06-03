#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
# 导入服务定义, 服务请求的消息类型也一并导入 (例: RequestPlanRequest)
from arm_obstacle_avoidance.srv import RequestPlan, RequestPlanResponse, RequestPlanRequest
# 导入自定义的MATLAB规划结果消息类型
from arm_obstacle_avoidance.msg import MatlabPlanningResult
import threading

class PlanningServiceNode:
    def __init__(self):
        # 初始化ROS节点，名称更具体
        rospy.init_node('planning_service_node')

        # --- 内部状态，用于同步等待MATLAB的响应 ---
        self._matlab_planning_result_cache = None  # 缓存从MATLAB接收到的结果
        self._result_received_event = threading.Event() # 用于线程同步的事件

        # --- ROS Publisher: 用于将规划请求数据发送给MATLAB规划器 ---
        self._matlab_request_publisher = rospy.Publisher(
            '/matlab_planning/request_data', # 主题名称
            RequestPlanRequest,              # 消息类型 (服务请求部分)
            queue_size=5                     # 队列大小，5表示如果MATLAB处理不过来，最多缓存5条
        )
        rospy.loginfo("发布器 /matlab_planning/request_data 已创建。")

        # --- ROS Subscriber: 用于从MATLAB规划器接收规划结果 ---
        self._matlab_result_subscriber = rospy.Subscriber(
            '/matlab_planning/planning_status', # 主题名称
            MatlabPlanningResult,               # 消息类型 (自定义的结果消息)
            self._matlab_result_callback,       # 消息回调函数
            queue_size=5                        # 队列大小
        )
        rospy.loginfo("订阅器 /matlab_planning/planning_status 已创建。")

        # --- ROS Service Server: 提供给外部ROS节点的路径规划服务 ---
        self._planning_ros_service = rospy.Service(
            'request_robot_plan',             # 服务名称 (与srv文件名一致)
            RequestPlan,                      # 服务类型 (导入自srv文件)
            self._handle_ros_planning_request # 服务处理回调函数
        )
        rospy.loginfo("ROS服务 /request_robot_plan 已启动并等待请求。")
        rospy.loginfo("规划服务节点已准备就绪。")

    def _matlab_result_callback(self, msg):
        """
        当从 /matlab_planning/planning_status 主题接收到MATLAB的规划/验证结果时，此回调函数被调用。
        """
        rospy.loginfo("从MATLAB接收到规划/验证结果。成功状态: %s, 环境显著变化: %s",
                      "是" if msg.success else "否",
                      "是" if msg.environment_has_changed_significantly else "否")
        self._matlab_planning_result_cache = msg
        self._result_received_event.set()

    def _handle_ros_planning_request(self, service_req_msg):
        """
        当 /request_robot_plan 服务被调用时，此函数处理传入的请求。
        它将请求数据发布给MATLAB，然后等待MATLAB的响应或超时。
        """
        if service_req_msg.validate_environment_only:
            rospy.loginfo("处理来自ROS客户端的环境验证请求...")
        else:
            rospy.loginfo("处理来自ROS客户端的完整路径规划请求...")

        self._result_received_event.clear()
        self._matlab_planning_result_cache = None

        # 将服务请求消息 (service_req_msg，类型为RequestPlanRequest)
        # 直接发布到给MATLAB的主题。服务请求消息结构已经包含了新字段。
        try:
            # service_req_msg 本身就是 RequestPlanRequest 类型的对象，可以直接发布
            # 它包含了 current_obstacle_map, current_ee_pose, current_joint_values, 
            # goal_ee_pose, 和 validate_environment_only 字段。
            self._matlab_request_publisher.publish(service_req_msg)
            rospy.loginfo("请求数据已发布至 /matlab_planning/request_data 主题，等待MATLAB响应。")
        except Exception as e:
            rospy.logerr("发布规划/验证请求至MATLAB时发生错误: %s", str(e))
            response = RequestPlanResponse()
            response.success = False
            response.message = "ROS_SERVICE错误：未能将请求发送至MATLAB中间件: " + str(e)
            response.environment_has_changed_significantly = False # 默认为false
            return response

        timeout_seconds = 60.0 # 对于规划，可能需要更长时间
        if service_req_msg.validate_environment_only:
            timeout_seconds = 15.0 # 验证应该更快

        rospy.loginfo("等待MATLAB响应，超时时间: %.1f 秒...", timeout_seconds)

        response = RequestPlanResponse() # 初始化响应对象

        if self._result_received_event.wait(timeout=timeout_seconds):
            if self._matlab_planning_result_cache:
                rospy.loginfo("已接收并处理来自MATLAB的响应。")
                # 从缓存的结果 (类型: MatlabPlanningResult) 构建服务响应 (类型: RequestPlanResponse)
                response.planned_trajectory = self._matlab_planning_result_cache.planned_trajectory
                response.success = self._matlab_planning_result_cache.success
                response.message = self._matlab_planning_result_cache.message
                response.environment_has_changed_significantly = self._matlab_planning_result_cache.environment_has_changed_significantly
            else:
                rospy.logerr("内部错误：MATLAB结果事件已设置，但未找到存储的结果数据。")
                response.success = False
                response.message = "ROS_SERVICE错误：处理MATLAB响应时发生内部错误。"
                response.environment_has_changed_significantly = False
        else:
            rospy.logwarn("等待MATLAB响应超时（%.1f秒）。", timeout_seconds)
            response.success = False
            response.message = "ROS_SERVICE错误：等待MATLAB响应超时。"
            response.environment_has_changed_significantly = False # 在超时情况下，假设环境未确认变化

        return response # 将最终的响应返回给服务调用者

if __name__ == '__main__':
    try:
        PlanningServiceNode() # 创建并启动服务节点对象
        rospy.spin()          # 使节点保持运行，监听回调
    except rospy.ROSInterruptException:
        rospy.loginfo("规划服务节点已关闭。")
    except Exception as e_main:
        rospy.logfatal("规划服务节点发生严重错误: %s", str(e_main))
