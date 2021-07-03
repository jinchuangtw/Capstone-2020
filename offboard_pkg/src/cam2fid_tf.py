#!/usr/bin/env python
import sys
import rospy
import numpy as np
import tf

from fiducial_msgs.msg import FiducialTransformArray

class Fiducial_tf:

    def __init__(self):

        self.listener = tf.TransformListener()

        self.fid_tf_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.tf_listener ,queue_size=10)

    #################### Main #################### 
    def tf_listener(self, data):
  
        self.posture = data
        br = tf.TransformBroadcaster()

        for i in range(0, len(self.posture.transforms)):

            ft = self.posture.transforms[i]

            fid = ft.fiducial_id
            x_pos = round(ft.transform.translation.x, 1)
            y_pos = round(ft.transform.translation.y, 1)
            z_pos = round(ft.transform.translation.z, 1)

            q1 = round(ft.transform.rotation.x, 2)
            q2 = round(ft.transform.rotation.y, 2)
            q3 = round(ft.transform.rotation.z, 2)
            q4 = round(ft.transform.rotation.w, 2)
            quat = [q1, q2, q3, q4]
            quat_norm = quat / np.linalg.norm(quat)

            current_id = 'fid'+ str(fid)
            #print('id:{0}\nx:{1}\ny{2}\nz:{3}'.format(fid, x_pos, y_pos, z_pos))

            br.sendTransform((x_pos, y_pos, z_pos), quat_norm, rospy.Time.now(), current_id, 'usb_cam')

def main(args):
    rospy.init_node('camera2fid_tf')
    fid = Fiducial_tf()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)