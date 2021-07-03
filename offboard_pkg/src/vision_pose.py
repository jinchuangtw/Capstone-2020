#!/usr/bin/env python
import sys
import rospy
import mavros
import math
import numpy as np

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import OpticalFlowRad


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

        self.flow_z = 0.0


    #################### Callback ####################
    def fid_pose(self, data):

        self.slam_x = data.pose.pose.position.x
        self.slam_y = data.pose.pose.position.y
        self.slam_z = data.pose.pose.position.z


    def px4flow(self, data):

        self.flow_z = data.distance


    #################### Main #################### 
    def send_pose(self):

        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():

            # send vision pose
            self.pose.header.frame_id = 'map'
            self.pose.header.stamp = rospy.Time.now()
            self.pose.pose.position.x = round(self.slam_x, 2)
            self.pose.pose.position.y = -round(self.slam_y, 2)
            self.pose.pose.position.z = round(self.slam_z, 2)
            #self.pose.pose.position.z = round(self.flow_z, 2)

            self.vision_pose_pub.publish(self.pose)

            print('slam_x:{}, slam_y:{}, alt:{}'.format(self.pose.pose.position.x, 
                    self.pose.pose.position.y, self.pose.pose.position.z))
   
            rate.sleep()


def main(args):
    rospy.init_node('mapInfo')
    mi = MapInfo()
    mi.send_pose()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)