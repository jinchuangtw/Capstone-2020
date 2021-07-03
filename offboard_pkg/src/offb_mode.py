#!/usr/bin/env python
import sys
import rospy
import mavros
import numpy as np

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode

mavros.set_namespace()

class takeoff:

    def __init__(self):

        self.current_state = State()
        self.pose = PoseStamped()
        self.offb_set_mode = SetMode
        self.update_pose = 0

        self.local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)

        self.arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
        self.set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 
        
        state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self.state_cb, queue_size=10)
        nav_goal = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.pose_update, queue_size=10)

    #################### Callback #################### 
    def state_cb(self, state):

        rospy.loginfo_once('Calling for State...')
        self.current_state = state
 
    def pose_update(self, data):

        rospy.loginfo('Calling for Pose_update...')
        self.target = data
        self.update_pose = 1

    #################### Main ####################
    def offboard(self):

        rospy.loginfo('Commander Takeoff')
        rate = rospy.Rate(20.0) # MUST be more then 2Hz

        # send a few setpoints before starting
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0.7
        for i in range(100):
            self.local_pos_pub.publish(self.pose)
            rate.sleep()

        # wait for FCU connection
        while not self.current_state.connected:
            rate.sleep()

        # Disarmed
        while not self.current_state.armed:
            self.arming_client(True)
            rospy.loginfo("Vehicle armed : %r" % self.current_state.armed)


        #offb_set_mode.custom_mode = "OFFBOARD"
        while self.current_state.mode != "OFFBOARD":
            #offb_set_mode.custom_mode = "OFFBOARD"
            self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            rospy.loginfo("Current mode: %s" % self.current_state.mode)
            rate.sleep()

        # Fly Control

        while not rospy.is_shutdown():
            self.pose.header.frame_id = 'map'
            self.pose.header.stamp = rospy.Time.now()
            
            if self.update_pose == 0:
                self.pose.pose.position.x = 0.0
                self.pose.pose.position.y = 0.0
                rospy.loginfo('Initial State...')
            else:
                self.pose.pose.position.x = self.target.pose.position.x
                self.pose.pose.position.y = self.target.pose.position.y
            
            self.pose.pose.position.z = 0.7

            self.local_pos_pub.publish(self.pose)
            rospy.loginfo('x: %s, y: %s, z: %s', self.pose.pose.position.x, 
                    self.pose.pose.position.y, self.pose.pose.position.z)
            rate.sleep()

def main(args):
    rospy.init_node('Takeoff')
    fly = takeoff()
    fly.offboard()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)