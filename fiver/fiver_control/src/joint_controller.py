#!/usr/bin/env python
import rospy
import rospkg
import tf
import tf.msg

from std_msgs.msg import Float64
import math 
import numpy as np
import geometry_msgs.msg



if __name__ == "__main__":


    epsilon = 1e-5
    rospy.init_node("Fiver_Controller_Node")
    theta5_pub = rospy.Publisher('/fiver/joint5_position_controller/command',Float64,queue_size=5)
    theta2_pub = rospy.Publisher('/fiver/joint1_position_controller/command',Float64,queue_size=5)
    # pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage,queue_size=5)
    # Initialize the Thetas in the Parameter Server
    rospy.set_param('q2',0)
    rospy.set_param('q5',0)

    theta5 = rospy.get_param("q5") 
    theta2 = rospy.get_param("q2")
    print(theta2,theta5)
    rate = rospy.Rate(10) # 10hz
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform("/link1", "/end_effector", rospy.Time(), rospy.Duration(4.0))
    tmp=[0,0,0]
    while not rospy.is_shutdown():
        theta5 = rospy.get_param("q5") 
        theta2 = rospy.get_param("q2")
        print('theta2,theta5',theta2,theta5)
        # rospy.loginfo("theta2: " + str(theta2)+ "," + "theta5:" +str(theta5))
        theta5_pub.publish(theta5)
        # rate.sleep()
        theta2_pub.publish(theta2)
        (trans,rot) = tf_listener.lookupTransform('/link1', '/end_effector', rospy.Time(0))

        rospy.set_param('end_effector',trans)


    # rospy.spin()
