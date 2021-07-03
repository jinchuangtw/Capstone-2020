#!/usr/bin/env python
import sys
import rospy
import mavros
import math
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate 

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import OpticalFlowRad
from matplotlib.animation import FuncAnimation


mavros.set_namespace()


class MapInfo:

    def __init__(self):
        
        # Subscriber
        fid_pose_sub = rospy.Subscriber('/fiducial_pose', PoseWithCovarianceStamped, self.fid_pose, queue_size=10)
        opt_flow_sub = rospy.Subscriber(mavros.get_topic('px4flow', 'raw', 'optical_flow_rad'), OpticalFlowRad, self.px4flow, queue_size=10)

        # Publisher
        self.vision_pose_pub = rospy.Publisher(mavros.get_topic('vision_pose', 'pose'), PoseStamped, queue_size=10)

        # Initialization
        self.pose = PoseStamped()
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_z = 0.0
        self.flow_x = 0.0
        self.flow_y = 0.0
        self.flow_z = 0.0

        self.x_list = []
        self.y_list = []

        self.figure, self.ax = plt.subplots(figsize=(4,3))
        self.line, = self.ax.plot(self.x_list, self.y_list)
        plt.axis([0, 10, -abs(self.flow_x*10), abs(self.flow_x*10)])



        self.time = []

    #################### Callback ####################
    def fid_pose(self, data):

        self.slam_x = data.pose.pose.position.x
        self.slam_y = data.pose.pose.position.y
        self.slam_z = data.pose.pose.position.z


    def px4flow(self, data):
        
        now = rospy.get_rostime()
        print(now.nsecs)
        self.flow_x = data.integrated_x
        self.flow_y = data.integrated_y
        self.flow_z = data.distance

    def func_animate(self, i):
        self.x_list = np.linspace(0, 4*np.pi, 1000)
        self.y_list = self.flow_x
        
        self.line.set_data(self.x_list, self.y_list)
        
        return self.line,

    #################### Main #################### 
    def send_pose(self):

        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():

            print('Draw')
            ani = FuncAnimation(self.figure,
                    self.func_animate,
                    frames=10,
                    interval=50)
            #ani.save(r'animation.gif', fps=10)

            plt.show()

            #self.vision_pose_pub.publish(self.pose)

            print('count:', count)
            print('raw_slam_x:{}, raw_slam_y:{}, raw_alt:{}'.format(self.flow_x, 
                    self.flow_y, self.flow_z))            

            #print('slam_x:{}, slam_y:{}, alt:{}'.format(self.pose.pose.position.x, 
            #        self.pose.pose.position.y, self.pose.pose.position.z))
            count+=1
            rate.sleep()


def main(args):
    rospy.init_node('MapInfo')
    mi = MapInfo()
    mi.send_pose()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)