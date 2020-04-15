#!/usr/bin/env python
import math
import multiprocessing
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
import tf
import tf.msg

from std_msgs.msg import Float64
import rospkg,rospy
import xml.etree.ElementTree as ET

class FiverEnv(gym.Env):
    
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    rospy.init_node('fiver_gym',anonymous=True)
    pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage,queue_size=5)
    tf_broadaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()
    
    def __init__(self):
        
        self.theta5_pub = rospy.Publisher('/fiver/joint5_position_controller/command',Float64,queue_size=5)
        self.theta2_pub = rospy.Publisher('/fiver/joint1_position_controller/command',Float64,queue_size=5)

        self.link_lengths= {"link1":0,"link2":0,"link3":0,"link4":0,"link5":0,"width":0}
        # reading the link lengths from the xml file in rosfiles
        self.links_length_loader()
        self.link1 = self.link_lengths["link1"]
        self.link2 = self.link_lengths["link2"]
        self.link3 = self.link_lengths["link3"]
        self.link4 = self.link_lengths["link4"]
        self.link5 = self.link_lengths["link5"]
        self.link_width=self.link_lengths["width"]
        self.p1 = multiprocessing.Process(target=self.run, args=(self.theta2_pub,self.theta5_pub))

        self.offset_distance = 350
        self.q2 = 0
        self.q5 = 0
        self.q3 = None
        self.q4 = None
        self.objective = [504.5, 763.056]
        self.epsilon_limit = 20 #Tolerance
        low = np.array([-60,80])
        high = np.array([60,125])


        self.input_angles = multiprocessing.Array('d',2) #2 values,  self.q2 and self.q5
        self.input_angles[0] = self.q2
        self.input_angles[1] = self.q5


        # # low_angle=np.array([0,0])
        # low_angle=np.array([0,0])
        # # high_angle=np.array([5*math.pi/6,5*math.pi/6])
        # high_angle=np.array([math.pi,math.pi])
        # self.action_space = spaces.Box(low_angle, high_angle, dtype=np.float32)
        # self.observation_space = spaces.Box(low, high, dtype=np.float32)

        # self.seed()
        # self.viewer = None
        # self.state = None


    def links_length_loader(self):
        rospack = rospkg.RosPack()
        fiver_path = rospack.get_path('fiver_description')+"/urdf/fiver.xacro"
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
        print(self.q2,self.q5)
        if len(action) == 1 and (isinstance(action[0],list) or isinstance(action[0],np.ndarray)):
            self.q2,self.q5 = action[0]
        elif len(action) >=2:
            self.q2, self.q5 = action[0],action[1]
        else:
            print("Something go wrong in inputs for thetas")

        self.input_angles[0] = self.q2
        self.input_angles[1] = self.q5

        print("step function", self.input_angles[:])        



    def run(self,pub_theta2,pub_theta5):
        # print("Run Function",pub_theta2)
        while not rospy.is_shutdown():
            self.q2=self.input_angles[0]
            self.q5=self.input_angles[1]
            pub_theta2.publish(self.q5)    
            pub_theta5.publish(self.q2)
            print("Run Function",self.q2,self.q5)





    def reset(self):
        self.state = self.observation_space.sample()
        return np.array(self.state)

    def render(self, mode='human'):
        
        screen_width = 1000
        screen_height = 800
        screen_width = 1000

        if self.viewer is None:
            print("Inside self.viewer")
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)


            l,r,t,b = -self.link_width/2, self.link_width/2, self.link1/2, -self.link1/2

            link1_location=[self.offset_distance,self.offset_distance]
            link1 = rendering.FilledPolygon([(b,l), (b,r), (t,r), (t,l)])
            link1.set_color(1,0,0)
            link1trans = rendering.Transform(translation=(link1_location[0], link1_location[1]),rotation=0)
            link1.add_attr(link1trans)
            self.viewer.add_geom(link1)

            track = rendering.Line((0,self.offset_distance), (screen_width,self.offset_distance))
            track.set_color(1,0,0)
            self.viewer.add_geom(track)

            l,r,t,b = -self.link_width/2, self.link_width/2, self.link2/2, -self.link2/2
            link2_location=[link1_location[0]-self.link1/2+0.5*self.link2*math.cos(self.q2),link1_location[1]+0.5*self.link2*math.sin(self.q2)]
            link2 = rendering.FilledPolygon([(b,l), (b,r), (t,r), (t,l)])
            link2.set_color(1,0,0)
            link2trans = rendering.Transform(translation=(link2_location[0], link2_location[1]),rotation=self.q2)
            link2.add_attr(link2trans)
            self.viewer.add_geom(link2)

            l,r,t,b = -self.link_width/2, self.link_width/2, self.link3/2, -self.link3/2
            link3_location=[link1_location[0]-self.link1/2+self.link2*math.cos(self.q2)+0.5*self.link3*math.cos(self.q3),link1_location[1]+self.link2*math.sin(self.q2)+0.5*self.link3*math.sin(self.q3)]
            link3 = rendering.FilledPolygon([(b,l), (b,r), (t,r), (t,l)])
            link3.set_color(1,0,0)
            link3trans = rendering.Transform(translation=(link3_location[0], link3_location[1]),rotation=self.q3)
            link3.add_attr(link3trans)
            self.viewer.add_geom(link3)


            l,r,t,b = -self.link_width/2, self.link_width/2, self.link5/2, -self.link5/2
            link5_location=[link1_location[0]+self.link1/2+0.5*self.link5*math.cos(self.q5),link1_location[1]+0.5*self.link5*math.sin(self.q5)]
            link5 = rendering.FilledPolygon([(b,l), (b,r), (t,r), (t,l)])
            link5.set_color(1,0,1)
            link5trans = rendering.Transform(translation=(link5_location[0], link5_location[1]),rotation=self.q5)
            link5.add_attr(link5trans)
            self.viewer.add_geom(link5)



            l,r,t,b = -self.link_width/2, self.link_width/2, self.link4/2, -self.link4/2
            link4_location=[link1_location[0]+self.link1/2+self.link5*math.cos(self.q5)+0.5*self.link4*math.cos(self.q4),link1_location[1]+self.link5*math.sin(self.q5)+self.link3*0.5*math.sin(self.q4)]
            link4 = rendering.FilledPolygon([(b,l), (b,r), (t,r), (t,l)])
            link4.set_color(1,0,1)
            link4trans = rendering.Transform(translation=(link4_location[0], link4_location[1]),rotation=self.q4)
            link4.add_attr(link4trans)
            self.viewer.add_geom(link4)


            origin_location = [self.offset_distance,self.offset_distance]
            origin = rendering.make_circle(20/2,filled=True)
            origin.set_color(0,1,1)
            origin_trans = rendering.Transform(translation=(origin_location[0], origin_location[1]))
            origin.add_attr(origin_trans)
            self.viewer.add_geom(origin)


            objective_location = self.objective
            objective = rendering.make_circle(3,filled=True)
            objective_circle = rendering.make_circle(3+self.epsilon_limit,filled=False)
            objective.set_color(1,1,0)
            objective_circle.set_color(1,0,0)
            objective_trans = rendering.Transform(translation=(objective_location[0], objective_location[1]))
            objective_circle_trans = rendering.Transform(translation=(objective_location[0], objective_location[1]))
            objective.add_attr(objective_trans)
            objective_circle.add_attr(objective_circle_trans)
            self.viewer.add_geom(objective)
            self.viewer.add_geom(objective_circle)



            self.end_effector_location = [link1_location[0]+self.link1/2+self.link5*math.cos(self.q5)+self.link4*math.cos(self.q4),link1_location[1]+self.link5*math.sin(self.q5)+self.link3*math.sin(self.q4)]
            end_effector = rendering.make_circle(20/2)
            end_effector_trans = rendering.Transform(translation=(self.end_effector_location[0], self.end_effector_location[1]))
            end_effector.add_attr(end_effector_trans)
            self.viewer.add_geom(end_effector)
            return self.viewer.render(return_rgb_array = mode=='rgb_array')


    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None

if __name__ == "__main__":
    test = FiverEnv()
    try:

        rospy.sleep(2)
        test.p1.start()
        test.p1.join()
        
        # test.run()


        test.step(0,0)
        rospy.sleep(2)

        test.step(-math.pi/10,+math.pi/10)
        # test.run()
        # print("Erdi")
        # rospy.spin()
    except rospy.ROSInterruptException:
        print ("Shutting down ROS Image feature detector module")