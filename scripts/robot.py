#!/usr/bin/env python
'''
Author: Lucas Tindall

Publisher on temp senser activation topic

Subsciber on temp sensor data topic

Sensor proxy on texture sensor service

Sensor proxy on map move service

Calculate position

Publish position, temp and texture

Check if robot has made all the moves in move list

If so publish to sim_complete 
And shutdown 
'''
from __future__ import division
import rospy
import itertools
import random as r
import math as m
import numpy as np
from copy import deepcopy
from std_msgs.msg import String, Float32, Bool
from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
from cse_190_assi_1.srv import requestMapData, moveService, requestTexture
from read_config import read_config



class robot():
    def __init__(self):
        """Read config file and setup ROS things"""
        self.config = read_config()
        #self.config["prob_move_correct"] = .75
        rospy.init_node("robot")
	self.temperature_sensor_activation = rospy.Publisher(
                "/temp_sensor/activation",
                Bool,
                queue_size = 10
        )
	self.temperature_sensor_data = rospy.Subscriber(
                "/temp_sensor/data",
                temperatureMessage,
                self.handle_temperature_sensor_data
        )
	self.robot_texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture
        )
	self.robot_move_requester = rospy.ServiceProxy(
                "moveService",
                moveService
        )
	self.position_publisher = rospy.Publisher(
                "/results/probabilities",
                RobotProbabilities,
                queue_size = 10
        )
	self.temperature_publisher = rospy.Publisher(
                "/results/temperature_data",
                Float32,
                queue_size = 10
        )
	self.texture_publisher = rospy.Publisher(
                "/results/texture_data",
                String,
                queue_size = 10
        )
	self.simulation_complete_publisher = rospy.Publisher(
                "/map_node/sim_complete",
                Bool,
                queue_size = 10
        )
	self.temperature_requester = rospy.ServiceProxy(
                "requestMapData",
                requestMapData
        )
	self.temp_dict = {
                'H': 40.0,
                'C': 20.0,
                '-': 25.0
        }
	self.temp_map = list(itertools.chain.from_iterable(self.config['pipe_map']))
	self.tex_map = list(itertools.chain.from_iterable(self.config['texture_map']))
	self.std_dev = self.config['temp_noise_std_dev'] 
	self.move_index = 0 
	self.moves = self.config['move_list']
	
        self.num_rows = len(self.config['pipe_map'])
        self.num_cols = len(self.config['pipe_map'][0])
	
	self.positions = []
	for i in range(self.num_rows*self.num_cols): 
	    self.positions.append(1/(self.num_rows*self.num_cols))
	#print "initial positions ",self.positions
	rospy.sleep(10)
	self.temperature_sensor_activation.publish(True)
        
        rospy.spin()
    def handle_temperature_sensor_data(self, message): 
    	"""
	Callback function for the temperature sensor data subscriber 

	Each time the robot receives new data from the temperature sensor
	it should request data from the texture sensor using the ServiceProxy
   	robot_texture_requester and then move the robot using the 
	ServiceProxy robot_move_requester. 
    	"""
	# Get sensor readings
 	temperature = message.temperature
	texture_response = self.robot_texture_requester()
	texture = texture_response.data
	
	# Prepare to update position 
	temporary_temp_pos = [] 
	normalization_constant = 0.0


	# Calculate position based on temperature sensor  
	for i in range(self.num_rows*self.num_cols): 
	    expected_temp = self.temp_dict[self.temp_map[i]]
	    temporary_temp_pos.append(((1/(self.std_dev * m.sqrt(2*m.pi))) *
			 m.pow(m.e,-1*(m.pow((temperature-expected_temp),2))/
	                 (2*m.pow(self.std_dev,2))))*self.positions[i])
	    normalization_constant += temporary_temp_pos[i]
	for i in range(self.num_rows*self.num_cols): 
	    self.positions[i]=(temporary_temp_pos[i]/normalization_constant)

	#print "Temperature ",temperature	
	self.temperature_publisher.publish(temperature) 
	#rospy.sleep(2)

	# Calculate position based on texture sensor
	normalization_constant = 0.0
	temporary_tex_pos = []

	for i in range(self.num_rows*self.num_cols): 
	    expected_tex = self.tex_map[i]
	    if (texture == expected_tex): 
	        prob_tex_given_x = self.config['prob_tex_correct']
            else: 
		prob_tex_given_x = 1-self.config['prob_tex_correct'] 
	    temporary_tex_pos.append(prob_tex_given_x * self.positions[i] )
            normalization_constant += temporary_tex_pos[i]
	for i in range(self.num_rows*self.num_cols): 
	    self.positions[i] = (temporary_tex_pos[i]/normalization_constant)

	#print "Texture ",texture
	self.texture_publisher.publish(texture)
	#rospy.sleep(2)
	
  	i = 0
	formated_positions = []
        print "positions: ",self.positions
	while i < len(self.positions):
	    formated_positions.append(self.positions[i:i+self.num_cols])
	    #print self.positions[i:i+self.num_cols]
	    i += self.num_cols
        formated_positions_copy = deepcopy(formated_positions)		
	print "formatted: ",formated_positions
        if self.move_index == len(self.moves):
	    flattened_positions = list(itertools.chain.from_iterable(formated_positions))
	    #print "Positions ",flattened_positions
	    self.position_publisher.publish(flattened_positions) 
            self.simulation_complete_publisher.publish(True)
	    rospy.sleep(5)
    	    rospy.signal_shutdown("Completed all moves, shutting down")
	
	# Move robot 
        self.robot_move_requester(self.moves[self.move_index]) 
        #print "Move ",self.moves[self.move_index]
	
	
	# Calculate position based on movement  
	expected_move = self.moves[self.move_index]
	for i in range(self.num_rows): 
	    for j in range(self.num_cols): 
	        formated_positions[i][j] = 0
		for move in self.config['possible_moves']:
		    if expected_move == move: 
			prob_move = self.config['prob_move_correct']
		    else: 
			prob_move = (1-self.config['prob_move_correct'])/(
				    len(self.config['possible_moves'])-1)
		    formated_positions[i][j] += formated_positions_copy[(i
					-move[0])%self.num_rows][(j-move[1])%
					self.num_cols] * prob_move
	
	flattened_positions = list(itertools.chain.from_iterable(formated_positions))
        self.positions = flattened_positions	
	#print "Positions ",flattened_positions
	self.position_publisher.publish(flattened_positions) 

	#publish temp.tex from square 0, move, publish move, publish temp,tex check possible move	
	
        self.move_index += 1 
	
	
if __name__ == '__main__':
    robo = robot()
