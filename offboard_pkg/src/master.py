#!/usr/bin/env python
import sys
import rospy
import mavros
import numpy as np

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, OpticalFlowRad
from mavros_msgs.srv import CommandBool, SetMode


mavros.set_namespace()

class takeoff:

    def __init__(self):

        # Initialization
        self.current_state = State()
        self.prev_state = self.current_state
        self.offb_set_mode = SetMode

        self.pose = PoseStamped()
        
        self.update_pose = 0
        self.flow_z = 0.0

        # Publisher
        self.local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)

        # Server
        self.arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
        self.set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 
        
        # Subscriber
        state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self.state_cb, queue_size=10)
        nav_goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.pose_update, queue_size=10)
        opt_flow_sub = rospy.Subscriber(mavros.get_topic('px4flow', 'raw', 'optical_flow_rad'), OpticalFlowRad, self.px4flow, queue_size=10)


    #################### Callback #################### 
    def state_cb(self, state):

        rospy.loginfo_once('Call for State...')
        self.current_state = state
 
    def pose_update(self, data):

        #rospy.loginfo('Calling for Pose_update...')
        self.target = data
        self.update_pose = 1

    def px4flow(self, data):

        self.flow_z = data.distance

        '''if self.update_pose == 0 and abs(1.0-self.flow_z)<0.1:
            self.goal_x = 7.0
            self.goal_y = 0.0
            self.update_pose = 1'''

    #################### Main ####################
    def offboard(self):

        rospy.loginfo('Commander Takeoff')
        rate = rospy.Rate(20.0) # MUST be more then 2Hz

        # send a few setpoints before starting
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 1.5
        for i in range(100):
            self.local_pos_pub.publish(self.pose)
            rate.sleep()

        # wait for FCU connection
        while not self.current_state.connected:
            rate.sleep()




        # Fly Control
        last_request = rospy.get_rostime()
        while not rospy.is_shutdown():

            now = rospy.get_rostime()
            if self.current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now 
            else:
                if not self.current_state.armed and (now - last_request > rospy.Duration(5.)):
                    self.arming_client(True)
                    last_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if self.prev_state.armed != self.current_state.armed:
                rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
            if self.prev_state.mode != self.current_state.mode: 
                rospy.loginfo("Current mode: %s" % self.current_state.mode)
            self.prev_state = self.current_state


            # Update timestamp and publish pose 
            self.pose.header.frame_id = 'map'
            self.pose.header.stamp = rospy.Time.now()
            
            if self.update_pose == 0:
                self.pose.pose.position.x = 0.0
                self.pose.pose.position.y = 0.0
                rospy.loginfo_once('Initial State...')

            else:
                self.pose.pose.position.x = self.target.pose.position.x
                self.pose.pose.position.y = self.target.pose.position.y
                '''self.pose.pose.position.x = self.goal_x
                self.pose.pose.position.y = self.goal_y'''


            self.pose.pose.position.z = 1.5

            self.local_pos_pub.publish(self.pose)

            rospy.loginfo('x: %s, y: %s, z: %s', self.pose.pose.position.x, 
                    self.pose.pose.position.y, self.pose.pose.position.z)
    
            
            rate.sleep()

def main(args):
    rospy.init_node('takeoff')
    fly = takeoff()
    fly.offboard()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)