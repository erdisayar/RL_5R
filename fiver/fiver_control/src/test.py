#!/usr/bin/env python
from fiver_gym_env import FiverEnv
import rospy
import math
test = FiverEnv()
# test.step(0.1,math.pi/3)


for _ in range(10):
    a=test.reset()
    a1,a2 = round(a[0],2), round(a[1],2)
    print(a1,a2)
    test.step(a1,a2)
    rospy.sleep(2)
# test.step(0,0)
# rospy.sleep(2)
# print('step2')
# test.step(0.1,math.pi/3)

rospy.sleep(2)
