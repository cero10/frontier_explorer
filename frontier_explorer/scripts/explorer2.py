#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *
import time
from heapq import nlargest
from heapq import nsmallest

start = time.time()-5

class object:
	def __init__(self):
		self.pos = Odometry()
		self.map_data = OccupancyGrid()
		self.goal = PoseStamped()
		self.pos.pose.pose.position.x = -5.0
		self.pos.pose.pose.position.y = -22.0
		self.map_data.info.origin.position.x = -31.4
		self.map_data.info.origin.position.y = -39.4
		self.map_data.info.resolution = 0.0500000007451
		self.map_data.info.width = 1056
		self.map_data.info.height = 800
		self.goal.header.frame_id = 'map'
		self.goal.pose.position.x = -5.0
		self.goal.pose.position.y = -20.0
		self.goal.pose.orientation.z = 0.7
		self.goal.pose.orientation.w = 0.7
		self.yaw = pi/2
		self.block = 'X  -5.0Y -22.0'
		self.coord = {}
		self.counter = -1
		self.counter0 = 1
		self.counter1 = 1
		self.lastx = -5.0
		self.lasty = -22.0
		self.lastyaw = 1.57

	def posxres(self,x):
		xx = (abs(self.map_data.info.origin.position.x - x)/(self.map_data.info.width*self.map_data.info.resolution))*self.map_data.info.width
		return round(xx)
	def posyres(self,y):
		yy = (abs(self.map_data.info.origin.position.y - y)/(self.map_data.info.height*self.map_data.info.resolution))*self.map_data.info.height
		return round(yy)
	
	def namer(self,x,y):
		x = self.quantify(x)
		y = self.quantify(y)
		string = 'X{:6.1f}Y{:6.1f}'.format(x,y)
		return string


	def quantify(self,x): #round up xy position to the minimum block size
		#print('quantify')
		absx = abs(x)
		a = int(absx/100)
		b = int(absx/10-a*100)
		c = int(absx/1-a*100-b*10)
		d = absx - a*100-b*10-c*1
		if d >= 0.5:
			d = 5
		elif d >= 0 and d < 0.5:
			d = 0
		if x >= 0:
			quant = float('{}{}{}.{}'.format(a,b,c,d)[0:7])
		elif x < -0.4999999:
			quant = float('-{}{}{}.{}'.format(a,b,c,d)[0:7])
		else:
			quant = float('{}{}{}.{}'.format(a,b,c,d)[0:7])
		return(quant)

	def get_dtheta(self,x,y): # get the angle delta between current position and another position
		delta_x = x - self.goal.pose.position.x
		delta_y = y - self.goal.pose.position.y
		theta = atan2(delta_y, delta_x)
		if theta < 0:
		    theta = pi + (pi - abs(theta))
		theta = self.yaw - theta
		if theta < -pi:
		  theta = pi + (pi - abs(theta))
		return (abs(theta))


	def get_distribution(self,x,y): # takes xy coordinates as input and returns a weight based on the distribution of the states of the cells
		#print("get distro")
		sum_occupied = sum_empty = sum_unknown = 0
		#print(x,y)
		x = self.posxres(x)
		y = self.posyres(y)
		#print(x,y)
		for j in range(int(round(y)+5), int(round(y)-5),-1):
			for i in range(int(x-5), int(x+5)):
				if (i > 0) and (i < self.map_data.info.width) and (j > 0) and (j < self.map_data.info.height):
					listno = (j*self.map_data.info.width)+i
					#print(listno,j,i,self.map_data.info.width)
					if (self.map_data.data[listno] > 0):
				    		sum_occupied = sum_occupied + 1
					elif (self.map_data.data[listno] == 0):
				   		sum_empty = sum_empty + 1
					else:
				    		sum_unknown = sum_unknown + 1
				else:
					sum_occupied = sum_occupied + 1
		# max = 100 cells (10x10)
		if (sum_occupied+sum_empty) > 85:
			return (-0.01*(sum_occupied+sum_empty))
		elif sum_unknown < 90 and sum_unknown > 20 and sum_occupied < 3:
			return (1-(sum_occupied*0.1))
		else:
			return 0

	def mark_blocks(self):
		#print("mark_blocks")
		self.counter = self.counter + 1
		self.block = self.namer(self.pos.pose.pose.position.x,self.pos.pose.pose.position.y)
		#print(self.block)
		if self.block not in self.coord:
			x = float(self.block[1:7])
			y = float(self.block[8:14])
			self.coord[self.block] = self.get_distribution(x,y)
		if self.yaw > 5.49778714 or self.yaw < 0.78539816:
			for i in range(0,12):
				for j in range((-5-i),(5+i)):
					#if self.pos.pose.pose.position.x > -21 and self.pos.pose.pose.position.x < 40 and self.pos.pose.pose.position.y > -23 and self.pos.pose.pose.position.y < 41:
					x = self.pos.pose.pose.position.x+(i*0.5)
					y = self.pos.pose.pose.position.y+(j*0.5)
					if (x > self.map_data.info.origin.position.x+1) and (x < self.map_data.info.width*self.map_data.info.resolution-1) and (y > self.map_data.info.origin.position.y+1) and (y < self.map_data.info.height*self.map_data.info.resolution-1):
						string = self.namer(x,y)
						#print(string)	
						if string not in self.coord:
							#print(string)
							self.coord[string] = self.get_distribution(x,y)
		elif self.yaw > 0.78539816 and self.yaw < 2.35619449:
			for i in range(0,12):
				for j in range((-5-i),(5+i)):
					#if self.pos.pose.pose.position.x > -21 and self.pos.pose.pose.position.x < 40 and self.pos.pose.pose.position.y > -23 and self.pos.pose.pose.position.y < 41:
					x = self.pos.pose.pose.position.x+(i*0.5)
					y = self.pos.pose.pose.position.y+(j*0.5)
					if (x > self.map_data.info.origin.position.x+1) and (x < self.map_data.info.width*self.map_data.info.resolution-1) and (y > self.map_data.info.origin.position.y+1) and (y < self.map_data.info.height*self.map_data.info.resolution-1):
						string = self.namer(x,y)
						#print(string)
						if string not in self.coord:
							#print(string)
							self.coord[string] = self.get_distribution(x,y)
		elif self.yaw > 2.35619449 and self.yaw < 3.92699082:
			for i in range(0,-12,-1):
				for j in range((-5-abs(i)),(5+abs(i))):
					#if self.pos.pose.pose.position.x > -21 and self.pos.pose.pose.position.x < 40 and self.pos.pose.pose.position.y > -23 and self.pos.pose.pose.position.y < 41:
					x = self.pos.pose.pose.position.x+(i*0.5)
					y = self.pos.pose.pose.position.y+(j*0.5)
					if (x > self.map_data.info.origin.position.x+1) and (x < self.map_data.info.width*self.map_data.info.resolution-1) and (y > self.map_data.info.origin.position.y+1) and (y < self.map_data.info.height*self.map_data.info.resolution-1):
						string = self.namer(x,y)
						#print(string)
						if string not in self.coord:
							#print(string)
							self.coord[string] = self.get_distribution(x,y)
		elif self.yaw > 3.92699082 and self.yaw < 5.49778714:
			for i in range(0,-12,-1):
				for j in range((-5-abs(i)),(5+abs(i))):
					#if self.pos.pose.pose.position.x > -21 and self.pos.pose.pose.position.x < 40 and self.pos.pose.pose.position.y > -23 and self.pos.pose.pose.position.y < 41:
					x = self.pos.pose.pose.position.x+(i*0.5)
					y = self.pos.pose.pose.position.y+(j*0.5)
					if (x > self.map_data.info.origin.position.x+1) and (x < self.map_data.info.width*self.map_data.info.resolution-1) and (y > self.map_data.info.origin.position.y+1) and (y < self.map_data.info.height*self.map_data.info.resolution-1):
						string = self.namer(x,y)
						#print(string)
						if string not in self.coord:
							#print(string)
							self.coord[string] = self.get_distribution(x,y)

	def reassess_blocks(self):
		#print("reassess_blocks")
		self.counter = 0
		self.block = self.namer(self.pos.pose.pose.position.x,self.pos.pose.pose.position.y)
		x = float(self.block[1:7])
		y = float(self.block[8:14])
		for i in range(28,-28,-1):
			for j in range((-28),(28)):
				xx = x+(j*0.5)
				yy = y+(i*0.5)
				string = 'X{:6.1f}Y{:6.1f}'.format(xx,yy)
				if self.coord.get(string) >= 0:
					self.coord[string] = (self.get_distribution(xx,yy))

	def frontier_designator(self):
		#print("designator")
		mdict = {}
		largest = nlargest(20, self.coord, key = self.coord.get)
		for i in range(0,len(largest)):
			if self.coord.get(largest[i]) > 0:
				x = float(largest[i][1:7])
				y = float(largest[i][8:14])
				theta = self.get_dtheta(x,y)
				a = self.coord.get(largest[i])
				weight = a - min(0.4,(theta /(4*pi))) - min(0.5,(abs(self.pos.pose.pose.position.x - x) + abs(self.pos.pose.pose.position.y - y))/50)
				mdict[largest[i]] = weight
				#print(type(theta),type(a),type(weight),mdict.get(largest[i]),type(mdict.get(largest[i])))
		if len(mdict) > 1:
			largest = nlargest(2, mdict, key = mdict.get)
		elif len(mdict) == 1:
			if self.goal.pose.position.x != float(largest[0][1:7]) and self.goal.pose.position.y != float(largest[0][8:14]):
				print('1ST CHOICE ', self.coord.get(largest[0]), largest[0],'  distro rank ', mdict.get(largest[0]))
				return largest[0]
			else:
				return 'N'
		else:
			return 'N'
		if self.goal.pose.position.x == float(largest[0][1:7]) and self.goal.pose.position.y == float(largest[0][8:14]):
			print('2ND CHOICE', self.coord.get(largest[1]), largest[1],' distro rank',mdict.get(largest[1]))
			return largest[1]
		else:
			print('1ST CHOICE ', self.coord.get(largest[0]), largest[0],' distro rank',mdict.get(largest[1]))
			return largest[0]
			
	def search_map(self):
		#print('WHOOOOOOLE MAP BABY')
		a = int(2*(self.quantify(self.map_data.info.origin.position.y)+0.5))
		b = int(2*self.quantify(self.quantify(self.map_data.info.origin.position.y + (self.map_data.info.resolution*self.map_data.info.height)-0.5)))
		c = int(2*(self.quantify(self.map_data.info.origin.position.x)+0.5))
		d = int(2*self.quantify(self.quantify(self.map_data.info.origin.position.x + (self.map_data.info.resolution*self.map_data.info.width)-0.5)))
		for j in range(b,a,-1):
			for i in range(c,d,1):
				string = 'X{:6.1f}Y{:6.1f}'.format(float(i)/2,float(j)/2)
				#print(string)
				#if self.coord.get(string) >= 0:
				self.coord[string] = (self.get_distribution(i/2,j/2))
				#print(string,self.coord.get(string))
		ggg = self.frontier_designator()
		return ggg
			

	def calc_dist(self):
		dist = abs(self.pos.pose.pose.position.x - self.goal.pose.position.x) + abs(self.pos.pose.pose.position.y - self.goal.pose.position.y)
		return dist

	def check_if_moved(self):
		global start
		if (time.time()-start) > 4:
			start = time.time()
			a = abs(self.pos.pose.pose.position.x - self.lastx)
			b = abs(self.pos.pose.pose.position.y - self.lasty)
			c = abs(self.yaw - self.lastyaw)
			self.lastx = self.pos.pose.pose.position.x
			self.lasty = self.pos.pose.pose.position.y
			self.lastyaw = self.yaw
			if a < 0.8 or b < 0.8 or c < 0.5:
				return 1
			else:
				return 0
		else:
			return 0

	def update_pos(self, msg):
		self.pos = msg
		orientation_list = [self.pos.pose.pose.orientation.x, self.pos.pose.pose.orientation.y, self.pos.pose.pose.orientation.z, self.pos.pose.pose.orientation.w]
		(roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
		

	def update_map_data(self, msg):
		self.map_data = msg

	def get_status(self, msg):
		self.status = msg.status_list[0].status if len(msg.status_list) else 0

def front_explorer():
	rospy.init_node('front_explorer')
	start = time.time()
	drone1 = object()
	rospy.Subscriber('quadrotor2/ground_truth/state', Odometry, drone1.update_pos)
	rospy.Subscriber('quadrotor2/map', OccupancyGrid, drone1.update_map_data)
	rospy.Subscriber('quadrotor2/move_base/status', GoalStatusArray, drone1.get_status)
	pub1 = rospy.Publisher('quadrotor2/move_base_simple/goal', PoseStamped, queue_size = 50)
	rate = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		if drone1.map_data.data:
			if drone1.counter0 == 1:
				drone1.search_map()
				drone1.counter0 = drone1.counter0 - 1
				drone1.goal.pose.position.x = float(drone1.block[1:7])
				drone1.goal.pose.position.y = float(drone1.block[8:14]) + 2.0
				pub1.publish(drone1.goal)
			drone1.mark_blocks()
			if drone1.counter > 1:
				drone1.reassess_blocks()
			
			if drone1.counter1 == 4:
				drone1.counter1 = 0
				if drone1.status == 4 or drone1.status == 5 or drone1.status == 9:
					print('drone1 ',drone1.status, "exei kollhsei")
					drone1.coord[drone1.namer(drone1.goal.pose.position.x,drone1.goal.pose.position.y)] = -1
					string1 = drone1.namer(drone1.goal.pose.position.x,drone1.goal.pose.position.y)
					drone1.goal.pose.position.x = float(string1[1:7])
					drone1.goal.pose.position.y = float(string1[8:14])
					pub1.publish(drone1.goal)
			else:
				drone1.counter1 = drone1.counter1 + 1

			#if drone1.calc_dist() < 2.0 or drone1.check_if_moved() == 1:
			if drone1.calc_dist() < 2.0 or (time.time()-start) > 20:
				start = time.time()
				string = drone1.frontier_designator()
				if string != 'N':
					drone1.goal.pose.position.x = float(string[1:7])
					drone1.goal.pose.position.y = float(string[8:14])
					pub1.publish(drone1.goal)
					print(drone1.get_distribution(drone1.goal.pose.position.x,drone1.goal.pose.position.y))
				else:
					string = drone1.search_map()
					if string != 'N':
						drone1.goal.pose.position.x = float(string[1:7])
						drone1.goal.pose.position.y = float(string[8:14])
						print('AFTER ALL: ',string)
						pub1.publish(drone1.goal)
					else:
						print('THE MAPPING IS DONE')
			rate.sleep()

if __name__ == '__main__':
    try:
        front_explorer()
    except rospy.ROSInterruptException:
        pass
