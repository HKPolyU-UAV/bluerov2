#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from nav_msgs.msg import Odometry

class PoseVisualizer:
    def __init__(self):
        rospy.init_node('pose_visualizer', anonymous=True)
        self.time_stamps = []
        self.x_positions = []
        self.y_positions = []
        
        # 订阅融合后的位姿话题
        rospy.Subscriber("/bluerov2/fused_odom", Odometry, self.odom_callback)
        
        # 初始化绘图窗口
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position (m)')
        self.ax.set_title('Fused Pose Trajectory')
        self.ax.set_xlim(0, 100)  # 可根据需要调整时间范围
        self.ax.set_ylim(-20, 20) # 可根据需要调整坐标范围
        self.line_x, = self.ax.plot([], [], label='X Position')
        self.line_y, = self.ax.plot([], [], label='Y Position')
        self.ax.legend()

    def odom_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        
        # 保存时间戳和位置数据
        self.time_stamps.append(current_time)
        self.x_positions.append(x_pos)
        self.y_positions.append(y_pos)
        
        # 限制数据点数量（可选）
        if len(self.time_stamps) > 1000:
            self.time_stamps.pop(0)
            self.x_positions.pop(0)
            self.y_positions.pop(0)

    def animate(self, i):
        self.line_x.set_data(self.time_stamps, self.x_positions)
        self.line_y.set_data(self.time_stamps, self.y_positions)
        self.ax.relim()
        self.ax.autoscale_view()
        return self.line_x, self.line_y

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=50)
        plt.show()

if __name__ == '__main__':
    try:
        visualizer = PoseVisualizer()
        rospy.spin()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass