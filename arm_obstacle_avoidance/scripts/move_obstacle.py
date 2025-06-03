#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import time
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def move_obstacle():
    # 初始化ROS节点
    rospy.init_node('move_obstacle')
    
    # 等待Gazebo服务可用
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # 动态障碍物的参考高度 (在桌面上方，假设障碍物自身高度的中心)
    # 原障碍物尺寸 0.08 0.08 0.16 -> 新尺寸 0.1 0.1 0.2
    # 桌子顶面 z = 0.05
    obstacle_center_z_on_table = 0.05 + (0.2 / 2.0) # 桌高 + 障碍物半高
    
    # 定义调整后的初始位置 (示例值，需要根据您的场景仔细调整)
    initial_x = 0.6
    initial_y = -0.4 # 稍微远离机械臂初始工作区
    initial_z = obstacle_center_z_on_table
    
    # 创建ModelState消息
    model_state = ModelState()
    model_state.model_name = 'dynamic_obstacle'
    model_state.pose.orientation.w = 1.0
    model_state.pose.position.z = initial_z
    model_state.pose.position.x = initial_x # 设置初始X
    model_state.pose.position.y = initial_y # 设置初始Y
    
    # 移动障碍物的主循环
    rate = rospy.Rate(1)  # 1 Hz
    
    # 先将障碍物设置到初始位置 (如果它在world文件中的pose不同)
    try:
        set_model_state(model_state)
        rospy.loginfo("动态障碍物已设置到初始位置。")
    except rospy.ServiceException as e:
        rospy.logerr("设置初始位置失败: %s" % e)

    rospy.loginfo("等待10秒后开始移动障碍物...")
    time.sleep(10) # 或者根据您的测试场景调整等待时间

    try:
        rospy.loginfo("正在移动障碍物到机械臂路径上...")
        # 调整移动到的目标位置 (示例值，应位于机械臂工作路径上)
        model_state.pose.position.x = 0.6 # 假设机械臂工作区中心线
        model_state.pose.position.y = 0.15 # 移动到一个能阻挡路径的位置
        # Z 保持不变
        set_model_state(model_state)
        
        rospy.loginfo("障碍物已移动，将保持30秒...")
        rospy.sleep(30) # 保持在该位置的时间
        
        rospy.loginfo("正在将障碍物移回原位...")
        model_state.pose.position.x = initial_x
        model_state.pose.position.y = initial_y
        set_model_state(model_state)
        
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s" % e)
        
    rospy.loginfo("障碍物移动完成")


if __name__ == '__main__':
    try:
        move_obstacle()
    except rospy.ROSInterruptException:
        pass
