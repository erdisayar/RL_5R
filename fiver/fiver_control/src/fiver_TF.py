#!/usr/bin/env python
import rospy
import tf
import tf.msg
import geometry_msgs.msg


rospy.init_node("Fiver_TF_Node")
pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage,queue_size=5)
rate = rospy.Rate(10) # 10hz
tf_broadaster = tf.TransformBroadcaster()
tf_listener = tf.TransformListener()
tf_listener.waitForTransform("/link1", "/link2", rospy.Time(), rospy.Duration(4.0))
while not rospy.is_shutdown():
    (trans,rot) = tf_listener.lookupTransform('/link1', '/link3', rospy.Time(0))
    euler_angles = tf.transformations.euler_from_quaternion(rot) 
    counter_angles = tf.transformations.quaternion_from_euler(-euler_angles[0], -euler_angles[1], -euler_angles[2], 'ryxz')
    t = geometry_msgs.msg.TransformStamped()
    print(euler_angles)
    t.header.frame_id = "/link3"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "/end_effector"
    t.transform.translation.x = 0
    t.transform.translation.y = 0.25     
    t.transform.translation.z = 0
    t.transform.rotation.x = counter_angles[0]
    t.transform.rotation.y = counter_angles[1]
    t.transform.rotation.z = counter_angles[2]
    t.transform.rotation.w = counter_angles[3]
    tfm = tf.msg.tfMessage([t])
    pub_tf.publish(tfm)