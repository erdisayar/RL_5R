"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from http://incompleteideas.net/sutton/book/code/pole.c
permalink: https://perma.cc/C9ZM-652R
"""

import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np

class FiverEnv(gym.Env):
    """
    Description:
        A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track. The pendulum starts upright, and the goal is to prevent it from falling over by increasing and reducing the cart's velocity.
    Source:
        This environment corresponds to the version of the cart-pole problem described by Barto, Sutton, and Anderson
    Observation: 
        Type: Box(4)
        Num	Observation                 Min         Max
        0	Cart Position             -4.8            4.8
        1	Cart Velocity             -Inf            Inf
        2	Pole Angle                 -24 deg        24 deg
        3	Pole Velocity At Tip      -Inf            Inf
        
    Actions:
        Type: Discrete(2)
        Num	Action
        0	Push cart to the left
        1	Push cart to the right
        
        Note: The amount the velocity that is reduced or increased is not fixed; it depends on the angle the pole is pointing. This is because the center of gravity of the pole increases the amount of energy needed to move the cart underneath it
    Reward:
        Reward is 1 for every step taken, including the termination step
    Starting State:
        All observations are assigned a uniform random value in [-0.05..0.05]
    Episode Termination:
        Pole Angle is more than 12 degrees
        Cart Position is more than 2.4 (center of the cart reaches the edge of the display)
        Episode length is greater than 200
        Solved Requirements
        Considered solved when the average reward is greater than or equal to 195.0 over 100 consecutive trials.
    """
    
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    def __init__(self):
        self.link1 = 250
        self.link2 = 250
        self.link3 = 250
        self.link4 = 250
        self.link5 = 200
        self.link_width=10def
        self.offset_distance = 350
        self.q2 = 2*math.pi/3
        self.q5 = math.pi/3
        self.q3 = None
        self.q4 = None
        self.objective = [504.5, 763.056]
        self.epsilon_limit = 20 #Tolerance
        low = np.array([-60,80])
        high = np.array([60,125])

        # low_angle=np.array([0,0])
        low_angle=np.array([0,0])
        # high_angle=np.array([5*math.pi/6,5*math.pi/6])
        high_angle=np.array([math.pi,math.pi])
        self.action_space = spaces.Box(low_angle, high_angle, dtype=np.float32)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        self.seed()
        self.viewer = None
        self.state = None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self.q2 = action[0]
        self.q5 = action[1]
        A=2*self.link2*self.link3*math.cos(self.q2)-2*self.link3*(self.link5*math.cos(self.q5)+self.link1)
        B=2*self.link2*self.link3*math.sin(self.q2)-2*self.link3*self.link5*math.sin(self.q5)
        C=-1*(-2*self.link2*(self.link5*math.cos(self.q5)+self.link1)*math.cos(self.q2)+2*self.link1*self.link5*math.cos(self.q5)-2*self.link2*math.sin(self.q2)*self.link5*math.sin(self.q5)+self.link1*self.link1+self.link2*self.link2+self.link3*self.link3+self.link5*self.link5-self.link4*self.link4)
        try:
            t2= (B-math.sqrt(A*A+B*B-C*C))/(A+C)
            self.q3 = 2*math.atan(t2)
            self.q4 = math.atan2((self.link2*math.sin(self.q2)+self.link3*math.sin(self.q3)-self.link5*math.sin(self.q5)) ,(self.link2*math.cos(self.q2)+self.link3*math.cos(self.q3)-self.link1-self.link5*math.cos(self.q5)))
        except (ValueError):
            try:
                print("First Square is not calculated...")
                t1 = (B+math.sqrt(A*A+B*B-C*C))/(A+C)
                self.q3 = 2*math.atan(t1)
                self.q4 = math.atan2((self.link2*math.sin(self.q2)+self.link3*math.sin(self.q3)-self.link5*math.sin(self.q5)) ,(self.link2*math.cos(self.q2)+self.link3*math.cos(self.q3)-self.link1-self.link5*math.cos(self.q5)))

            except (ValueError):
                print("Second Square is not calculated...")
                self.q3 = None
                self.q4 = None
                done = True
                reward = -100.0


        self.end_effector_location = [self.offset_distance+self.link1/2+self.link5*math.cos(self.q5)+self.link4*math.cos(self.q4),self.offset_distance+self.link5*math.sin(self.q5)+self.link3*math.sin(self.q4)]
        
        self.state = (self.end_effector_location[0],self.end_effector_location[1])
        done =   self.state[0] < self.objective[0] + self.epsilon_limit \
                and self.state[0] > self.objective[0] - self.epsilon_limit\
                and self.state[1] < self.objective[1] + self.epsilon_limit\
                and self.state[1] > self.objective[1] - self.epsilon_limit
        done = bool(done)

        if not done:
            reward = -1.0
        else:
            reward = 1.0

        return np.array(self.state), reward, done, {}

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