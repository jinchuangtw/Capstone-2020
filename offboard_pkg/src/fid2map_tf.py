#!/usr/bin/env python
import sys
import rospy
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg

from fiducial_msgs.msg import FiducialTransformArray

class Fid2map_tf:

    def __init__(self):

        self.fid_tf_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.tf_listener ,queue_size=10)

        self.listener = tf.TransformListener()

    #################### Main #################### 
    def tf_listener(self, data):
  
        self.posture = data
        #br = tf2_ros.StaticTransformBroadcaster()
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)

        if len(self.posture.transforms)>1:
            for i in range(0, len(self.posture.transforms)):

                ft = self.posture.transforms[i]

                fid = ft.fiducial_id
                if fid == 1: continue

                current_id = 'fid' + str(fid)
                trans, rot = self.listener.lookupTransform(current_id, 'map', rospy.Time(0))

                x_pos = -round(trans[0], 1)
                y_pos = -round(trans[1], 1)
                z_pos = 0.0
                #marker_list.append(static_tf)
                print(current_id)
                print('trans:',trans)
                print('rot:',rot)
                br.sendTransform((x_pos, y_pos, z_pos), rot, rospy.Time.now(), 'map', current_id)
                rate.sleep()

        else: pass

def main(args):
    rospy.init_node('fid2map_tf')
    fid2map = Fid2map_tf()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)