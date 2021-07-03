#!/usr/bin/env python
import sys
import rospy
import mavros
import math
import numpy as np

from fiducial_msgs.msg import FiducialMapEntryArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray

class MapInfo:

    def __init__(self):
        
        # Subscriber
        self.map_sub = rospy.Subscriber('/fiducial_map', FiducialMapEntryArray, self.mapInfo, queue_size=10) 
        self.robot_pose = rospy.Subscriber('/fiducial_pose', PoseWithCovarianceStamped, self.true_pose, queue_size=10)

        # Publisher
        self.ideal_marker_pub = rospy.Publisher('/ideal_marker', Float64MultiArray, queue_size=10)

        # Initialization
        self.send_marker = Float64MultiArray()
        self.footprint = np.zeros((10, 5))
        self.stable = 0 


    #################### Callback ####################
    def true_pose(self, data):
        
        self.true_x = data.pose.pose.position.x
        self.true_y = data.pose.pose.position.y

        print('true_x:{}, true_y:{}'.format(self.true_x, self.true_y))


    #################### Main #################### 
    def mapInfo(self, data):
        
        frame_id = rospy.get_param('/mavros/vision_pose/tf/frame_id')
        
        counter = len(data.fiducials)
        current_pos = 0
        rate = rospy.Rate(10)

        for i in range(counter):
            
            marker = data.fiducials[i]
        
            detected_id = 'fid' + str(marker.fiducial_id)
            print('detected_id', detected_id)
            
            self.footprint[i,0] = marker.fiducial_id
            self.footprint[i,1] = marker.x 
            self.footprint[i,2] = marker.y
            
            if detected_id == frame_id:
                self.footprint[i, 3] = 1
                current_pos = i

        current_x = self.footprint[current_pos, 0]
        current_y = self.footprint[current_pos, 1]

        dist_data = []
        for j in range(counter):
            
            if j == current_pos or self.footprint[j,3] == 1:
                continue
                       
            dist_x = self.footprint[j,0] - current_x
            dist_y = self.footprint[j,1] - current_y 
            self.footprint[j,4] = math.sqrt(dist_x**2 + dist_y**2)
            dist_data.append(self.footprint[j, :])
        
        dist_arr = np.array(dist_data)
        minset = np.argmin(dist_arr, axis=0) 
    
        ideal_marker_set = dist_arr[minset[4], :]
        print(ideal_marker_set)

        # Data form: dim[1,5]; id,x,y,bool,dist
        self.send_marker.data = ideal_marker_set
        self.ideal_marker_pub.publish(self.send_marker)

        rate.sleep()


def main(args):
    rospy.init_node('MapInfo')
    mi = MapInfo()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)