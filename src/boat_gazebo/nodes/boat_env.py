#!/usr/bin/env python3
import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gym.utils import seeding
import copy
import math
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

class BoatEnv(gazebo_env.GazeboEnv):
	def __init__(self):
		
		# init environment
		LAUNCH_PATH = '/home/fizzer/gym_gazebo_ws/src/boat_gazebo/launch/boat.launch'
		gazebo_env.GazeboEnv.__init__(self, LAUNCH_PATH)

		# init gazebo services
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

		# init pubs & subs
		self.left_pub = rospy.Publisher('/boat/thrusters/left_thrust_cmd', Float32, queue_size=10)
		self.right_pub = rospy.Publisher('/boat/thrusters/right_thrust_cmd', Float32, queue_size=10)
		rospy.Subscriber('/camera1/image_raw', Image, queue_size=10)
		
		# idk
		self._seed()
		self.bridge = CvBridge()

		# define action and observation spaces
		act_high = np.array([1, 1])
		obs_high = np.array([np.inf, np.inf, np.inf, np.inf, np.pi, np.inf])
		self.action_space = spaces.Box(-act_high, act_high)
		self.observation_space = spaces.Box(-obs_high, obs_high)

		# position target (x y yaw x' y' yaw')
		self.target = np.array([5, 5, 0, 0, 0, 0])
		self.tol = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

	def get_state(self, img, prev_state):
		## comp position to get x y yaw
		## use (new state - old state) / time to get x' y' yaw'
		return np.zeros(6)

	def done_check(self, state): #task
		# if (abs(state - self.target) < self.tol):
		# 	return True
		# else:
		return False
		
	def compute_reward(self, state, prev_state): #task
		reward = 0
		# for i in range(5):
		# 	if (abs(self.target[i]-state[i]) < self.tol[i]):
		# 		reward += 2
		# 	elif (abs(self.target[i]-state[i]) < abs(self.target[i]-prev_state[i])):
		# 		reward += 1
		# 	else: reward -= 1
		return reward

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def step(self, speedL, speedR):

		# unpause physics
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		# execute action
		self.left_pub.publish(speedL)
		self.right_pub.publish(speedR)

		# get data from topic
		img_data = None
		while img_data is None:
			try:
				img_data = rospy.wait_for_message('/camera1/image_raw', Image, timeout=5)
			except:
				pass

		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
			cv2.imshow("Camera Feed", cv_image)
			cv2.waitKey(1)
		except CvBridgeError as e:
			print(e)

        # pause physics
		# rospy.wait_for_service('/gazebo/pause_physics')
		# try:
		# 	self.pause()
		# except (rospy.ServiceException) as e:
		# 	print ("/gazebo/pause_physics service call failed")

		# process image -> returns state (x y yaw, x' y' yaw')
		self.state = self.get_state(img_data,self.prev_state)

		# compute reward
		step_reward = self.compute_reward(self.state, self.prev_state)

		# check if done
		done = self.done_check(self.state)

		self.prev_state = self.state

      	# return state reward and done flag
		return self.state, step_reward, done

	def reset(self):

		# reset environment
		rospy.wait_for_service('/gazebo/reset_simulation')
		try:
			self.reset_proxy()
		except (rospy.ServiceException) as e:
			print ("/gazebo/reset_simulation service call failed")

		# unpause simulation to make observation
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			self.unpause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/unpause_physics service call failed")

		# get data from topic
		img_data = None
		while img_data is None:
			try:
				img_data = rospy.wait_for_message('/camera1/image_raw', Image, timeout=5)
			except:
				pass

		# process image -> returns state (x y yaw, x' y' yaw')
		self.prev_state = None
		self.state = self.get_state(img_data, self.prev_state)

		# pause physics
		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		return self.state

        