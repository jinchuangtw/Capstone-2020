#!/usr/bin/env python
import sys
import rospy
import numpy as np
import tf
import geometry_msgs.msg

from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform

class Fiducial_tf:

    def __init__(self):

        self.fid_tf_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.tf_listener ,queue_size=10)

        self.tf_bool = 0

    #################### Callback #################### 
    def tf_listener(self, data):
        self.tf_bool = 1 
        self.posture = data
        self.posture.FiducialTransform
        print(self.posture.FiducialTransform)



    #################### Main ####################
    #def converter(self):


def main(args):
    rospy.init_node('pose_estimate')
    fid = Fiducial_tf()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)