#!/usr/bin/env python  
import roslib
import rospy

import tf
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose2D,
    Point,
    Twist,
    TransformStamped,
    Quaternion,
)

if __name__ == '__main__':
    rospy.init_node('center_tf_broadcaster')

    pose = rospy.wait_for_message("/vanishing_point", Pose2D)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
	tf_z=698        
	tf_x=pose.x/tf_z
        tf_y=pose.y/tf_z
	
	
        
        br.sendTransform((tf_x, tf_y, 1),
                         (0.0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "image_pose",
                         "ardrone_base_bottomcam")
        rate.sleep()
