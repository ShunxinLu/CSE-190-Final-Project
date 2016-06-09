#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import json
from math import *
from cse_190_assi_3.msg import AStarPath
from read_config import read_config
from mdp import *
from astar import *
import random as r

class Map():

	def __init__(self):
		rospy.init_node("map")
		self.config = read_config()
		self.seed = self.config["seed"]

		self.map_publisher = rospy.Publisher(
                "/results/policy_list",
                PolicyList,
                queue_size = 10
        )

		self.complete_publisher = rospy.Publisher(
			"/map_node/sim_complete",
			Bool,
			queue_size = 10
		)

		self.robot1_activator = rospy.Publisher(
				"robot1_turn",
				Bool,
				queue_size = 10
		)

		self.robot2_activator = rospy.Publisher(
				"robot2_turn",
				Bool,
				queue_size = 10
		)

		self.reached_goal_sub = rospy.Subscriber(
                "reached_goal",
                Bool,
                self.handle_reached_goal
        )
 
		self.reached_goal = False
		self.board = self.init_board()
		rospy.sleep(1)
		self.map_publisher.publish(PolicyList(self.to_policy()))
		rospy.sleep(3)
		
		
		#print "one turn \n"
		self.robot1_activator.publish(True)
		"""rospy.sleep(60)
		self.robot2_activator.publish(True)
		rospy.sleep(60)"""
		rospy.spin()


	def handle_reached_goal(self, message):
		self.reached_goal = message.data
		self.complete_publisher.publish(True)
		rospy.sleep(1)
		rospy.signal_shutdown("map")

	def init_board(self):

		board = []
		obstacles = []
		while len(obstacles) < self.config["num_of_obstacles"]:
			pos_ob = [r.randint(0, self.config["map_size"][0]), r.randint(0, self.config["map_size"][1])]
			if pos_ob in obstacles or pos_ob == self.config["robot1"] or pos_ob == self.config["robot2"] or pos_ob == self.config["goal"]:
				continue
			obstacles.append(pos_ob)

		for i in range(self.config["map_size"][0]):
			board.append([])
			for j in range(self.config["map_size"][1]):
				cur = " "
				if [i, j] == self.config["robot1"]:			
					cur = "1"
				elif [i, j]  == self.config["robot2"]:
					cur = "2"
				elif [i, j] == self.config["goal"]:
					cur = "GOAL"
				elif [i, j] in obstacles:
					cur = "WALL"
				board[i].append(cur)
	
		return board


	def to_policy(self):
		result = []
		for i in range(self.config["map_size"][0]):
			for j in range(self.config["map_size"][1]):
				result.append(self.board[i][j])

		return result
if __name__ == '__main__':
	mp = Map()
		
