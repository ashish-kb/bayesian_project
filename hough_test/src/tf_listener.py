#!/usr/bin/env python  
import roslib
roslib.load_manifest('hough_test')
import rospy
import math
import tf
from geometry_msgs.msg import Transform
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    p = rospy.Publisher("/transformation", Transform)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/ardrone_base_bottomcam', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #print(rot)
        transformation = Transform()
        transformation.translation.x=trans[0]
        transformation.translation.y=trans[1]
        transformation.translation.z=trans[2]

        transformation.rotation.x=rot[0]
        transformation.rotation.y=rot[1]
        transformation.rotation.z=rot[2]
        transformation.rotation.w=rot[3]
        
        

        p.publish(transformation)

        rate.sleep()
