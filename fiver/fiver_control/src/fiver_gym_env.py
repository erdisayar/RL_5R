#!/usr/bin/env python
import math
import multiprocessing
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
# import tf
# import tf.msg

from std_msgs.msg import Float64
import rospkg,rospy
import xml.etree.ElementTree as ET


# rospy.init_node('fiver_gym',anonymous=True)
# theta5_pub = rospy.Publisher('/fiver/joint5_position_controller/command',Float64,queue_size=5)
# theta2_pub = rospy.Publisher('/fiver/joint1_position_controller/command',Float64,queue_size=5)
# pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage,queue_size=5)
# tf_broadaster = tf.TransformBroadcaster()
# tf_listener = tf.TransformListener()


class FiverEnv(gym.Env):
    
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    
    def __init__(self):
        

        self.link_lengths= {"link1":0,"link2":0,"link3":0,"link4":0,"link5":0,"width":0}
        # reading the link lengths from the xml file in rosfiles
        self.links_length_loader()
        self.link1 = self.link_lengths["link1"]
        self.link2 = self.link_lengths["link2"]
        self.link3 = self.link_lengths["link3"]
        self.link4 = self.link_lengths["link4"]
        self.link5 = self.link_lengths["link5"]
        self.link_width=self.link_lengths["width"]
        self.q2 = 0
        self.q5 = 0
        self.end_effector = None
        self.objective = [0.209, 0.349,0]
        self.epsilon_limit = 0.01 #Tolerance


        low_angle=np.array([-1.6,-1.2]) # first parameter q2 , second parameter q5
        high_angle=np.array([1.2,1.6])  # first parameter q2 , second parameter q5
        self.action_space = spaces.Box(low_angle, high_angle, dtype=np.float32)
        low = np.array([-60,80])
        high = np.array([60,125])
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        self.seed()
        self.viewer = None
        self.state = None


    def links_length_loader(self):
        # rospack = rospkg.RosPack()
        # fiver_path = rospack.get_path('fiver_description')+"/urdf/fiver.xacro"
        fiver_path = "/home/erdi/catkin_ws/src/fiver/fiver_description/urdf/fiver.xacro"
        tree = ET.parse(fiver_path)
        root = tree.getroot()
        k = np.array([list(elem.attrib.values()) for elem in root.iter("{http://www.ros.org/wiki/xacro}property")])
        tmp = [np.argwhere(k == link_name) for link_name in list(self.link_lengths.keys())]
        for i,link_name in enumerate(list(self.link_lengths.keys())):
           self.link_lengths[link_name] = k[tmp[i][0][0],1]

        return None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, *action):

        print("Step Function is Executed")
        if len(action) == 1 and (isinstance(action[0],list) or isinstance(action[0],np.ndarray)):
            self.q2,self.q5 = action[0]
            self.q2,self.q5 = round(self.q2,2), round(self.q5 ,2)
        elif len(action) >=2:
            self.q2, self.q5 = round(action[0],2),round(action[1],2)
        else:
            print("Something go wrong in inputs for thetas")

        rospy.set_param('q2',self.q2)
        rospy.set_param('q5',self.q5)
        rospy.sleep(1) # Wait until Gazebo done!!!
        print("q2,q5:",self.q2,self.q5)
        print(rospy.get_param('end_effector'))
        self.end_effector = rospy.get_param('end_effector')
        self.state = self.end_effector[0:2]

        # self.state[2] parameter is the z axis and it is zero always
        done =   self.state[0] < self.objective[0] + self.epsilon_limit \
                and self.state[0] > self.objective[0] - self.epsilon_limit\
                and self.state[1] < self.objective[1] + self.epsilon_limit\
                and self.state[1] > self.objective[1] - self.epsilon_limit
        done = bool(done)

        # self.state[2] parameter is the z axis and it is zero always. So there is no need in calculation

        reward = self.epsilon_limit - math.sqrt((self.objective[0]-self.end_effector[0])**2+(self.objective[1]-self.end_effector[1])**2)



        return np.array(self.state), reward, done, {}
        



        

    def reset(self):
        self.state = self.action_space.sample()
        return np.array(self.state)

    def render(self, mode='human'):
        pass

    def close(self):
        pass



if __name__ == "__main__":
    test = FiverEnv()
    try:
        test.step(0,0)
        # p1 = multiprocessing.Process(target=run, args=(test.q2,test.q5))
        # p1.start()
        # p1.joint()
        # test.p1.start()
        # test.p1.join()
        
        # test.run()

        rospy.sleep(2)
        test.step(-math.pi/3,-math.pi/7)
        rospy.sleep(2)

        # test.run()
        # print("Erdi")
        # rospy.spin()
    except rospy.ROSInterruptException:
        print ("Shutting down ROS Image feature detector module")