#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import json
from math import *
from cse_190_assi_3.msg import AStarPath
from read_config import read_config
from mdp import *
from astar import *


class Robot():
	
	def __init__(self):
		rospy.init_node("robot") 
		self.config = read_config()

		self.map_publisher = rospy.Publisher(
                "/results/policy_list",
                PolicyList,
                queue_size = 10
        )

		self.map_subscriber = rospy.Subscriber(
                "/results/policy_list",
                PolicyList,
                self.handle_map_message
        )

		self.robot1_sub = rospy.Subscriber(
				"robot1_turn",
				Bool,
				self.handle_turn
		)

		self.reached_goal_pub = rospy.Publisher(
                "reached_goal",
                Bool,
                queue_size = 10
        )

		self.robot2_activator = rospy.Publisher(
				"robot2_turn",
				Bool,
				queue_size = 10
		)

		"""AStarResult = AStarSearch()

		for item in AStarResult:
			print item
			rospy.sleep(1)
			self.path_publisher.publish(AStarPath(item))

		MDPResult = mdp()"""

		rospy.spin()
		#rospy.signal_shutdown("robot")

	
	def handle_map_message(self, message):
		new_board = []
		self.obstacles = []

		for i in range(self.config["map_size"][0]):
			new_board.append([])
			for j in range(self.config["map_size"][1]):
				if message.data[self.config["map_size"][1] * i + j] == "1":
					self.cur_pos = [i, j]
				elif message.data[self.config["map_size"][1] * i + j] == "2":
					self.opp_pos = [i, j]
				elif message.data[self.config["map_size"][1] * i + j] == "GOAL":
					self.goal = [i, j]
				elif message.data[self.config["map_size"][1] * i + j] == "WALL" or message.data[self.config["map_size"][1] * i + j] == "PIT":
					self.obstacles.append([i, j])
				new_board[i].append(message.data[self.config["map_size"][1] * i + j])

		self.board = new_board


	def handle_turn(self, message):
		if self.opp_pos == self.goal:
			pass
		path = AStarSearch(self.cur_pos, self.goal, self.board)
		self.board[self.cur_pos[0]][self.cur_pos[1]] = " "
		self.cur_pos = path[1]
		self.board[self.cur_pos[0]][self.cur_pos[1]] = "1"

		if self.cur_pos == self.goal:
			self.reached_goal_pub.publish(True)

		new_obstacle = [-999999, []]
		for i in range(self.opp_pos[0] - 3, self.opp_pos[0] + 3):
			if i < 0 or i >= self.config["map_size"][0]:
				continue
			for j in range(self.opp_pos[1] - 3, self.opp_pos[1] + 3):
				if [i, j] == self.cur_pos or [i, j] == self.opp_pos or [i, j] == self.goal or j < 0 or j >= self.config["map_size"][1] or [i, j] in self.obstacles:
					continue
				self.board[i][j] = "WALL"
				opp_path = AStarSearch(self.opp_pos, self.goal, self.board)
				if opp_path == False or len(opp_path) < new_obstacle[0] or (len(opp_path) == new_obstacle[0] and manhattan_distance([i, j], self.goal) >= manhattan_distance(new_obstacle[1], self.goal)) or AStarSearch(self.cur_pos, self.goal, self.board) == False:
					self.board[i][j] = " "
					continue
				new_obstacle[0] = len(opp_path)
				new_obstacle[1] = [i, j]
				self.board[i][j] = " "

		print new_obstacle
		if new_obstacle[1] == []:
			self.map_publisher.publish(PolicyList(self.to_policy()))
		else:
			self.board[new_obstacle[1][0]][new_obstacle[1][1]] = "PIT"
			self.map_publisher.publish(PolicyList(self.to_policy()))
		
		if self.cur_pos != self.goal:
			rospy.sleep(2)
			self.robot2_activator.publish(True)
		
				

	def to_policy(self):
		result = []
		for i in range(self.config["map_size"][0]):
			for j in range(self.config["map_size"][1]):
				result.append(self.board[i][j])

		return result

if __name__ == '__main__':
	rt = Robot()
		

