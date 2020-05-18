#!/usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransformArray
from actionlib_msgs.msg import GoalStatusArray
from gazebo_msgs.msg import LinkStates
from move_base_msgs.msg import MoveBaseActionGoal 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import sqrt



class line:
	def __init__(self,pub_line_min_dist,startPoint,endPoint):
		self.marker = Marker()
		self.marker.header.frame_id = "/odom"
		self.marker.type = self.marker.LINE_STRIP
		self.marker.action =self. marker.ADD
		# marker scale
		self.marker.scale.x = 0.03
		self.marker.scale.y = 0.03
		self.marker.scale.z = 0.03

	       # marker color
		self.marker.color.a = 1.0
		self.marker.color.r = 1.0
		self.marker.color.g = 1.0
		self.marker.color.b = 0.0
		
		
	    # marker orientaiton
		self.marker.pose.orientation.x = 0.0
		self.marker.pose.orientation.y = 0.0
		self.marker.pose.orientation.z = 0.0
		self.marker.pose.orientation.w = 1.0
	   # marker position
		self.marker.pose.position.x = 0.0
		self.marker.pose.position.y = 0.0
		self.marker.pose.position.z = 0.0

		    # marker line points
		self.marker.points = []
    		# first point
		first_line_point = Point()
		first_line_point.x = 0.0
		first_line_point.y = 0.0
		first_line_point.z = 0.0
		self.marker.points.append(first_line_point)
   		 # second point
		second_line_point = Point()
		second_line_point.x = 1.0
		second_line_point.y = 1.0
		second_line_point.z = 0.0
		self.marker.points.append(second_line_point)

		a_line_point = Point()
		a_line_point.x = 1.0
		a_line_point.y = 2.0
		a_line_point.z = 0.0
		self.marker.points.append(a_line_point)

    		# Publish the Marker
		pub_line_min_dist.publish(self.marker)




class logTool:
	
	def __init__(self):
		self.stan= 0
		self.dictDataModel = {}
		self.lastGoalID = " "		
		self.target  = 0 #goal		
		self.wantedEndPose = 0 #estymation of perfect end position 
		rospy.init_node('loging', anonymous=False)
		rospy.Subscriber("/move_base/status",GoalStatusArray, self.state)
		rospy.Subscriber("/fiducial_transforms",FiducialTransformArray, self.posMarker)
		rospy.Subscriber("/exact_pose",Odometry, self.posAqq)
		rospy.Subscriber("/gazebo/link_states",LinkStates, self.models)
		rospy.Subscriber("/move_base/goal",MoveBaseActionGoal, self.goal)

	def state(self,data):
		#method checking status of move basic(from topic)

		if  len(data.status_list)>=2:
			self.status = data.status_list[1].status	
			if self.stan == 0 and self.status == 1:
				self.stan=1
			if self.stan == 0 and self.status == 8:
				self.stan=2				
			#print("status ",self.status)

	def posAqq(self,data):
		#getting accual position of the robot(from topic)
		self.pos = data.pose.pose
		if self.stan == 1:
			if self.status == 3:
				self.stan = 0
				print("goal aqq")
		
		elif self.stan == 2:
			if self.status == 3:
				self.stan = 0
			        print("goal changed")
	def posMarker(self,data):
		#method chcecking if marker has been detected(form topic)
		if data.transforms:		
			#print(data.transforms) 
			self.startPose = data.transforms 
			pass
	
	def models(self,data): 
		#(form topic)
		names = data.name 
		modelsPos = data.pose
		self.dickDataModel = dict(zip(names,modelsPos)) 
		#print(self.dictData['tag_0::link'])		

	def goal(self,data):
		if self.lastGoalID != data.goal_id.id:
			self.lastGoalPose = data.goal.target_pose.pose
			self.lastGoalID = data.goal_id.id
			#print(self.lastGoalPose)	
	
	def teoriticalGoal(self):	
		self.pos	#pozycja teraz 
		# filter only tags 
		tags = 	[[],[]]	
		distance=[]
		for x in self.dickDataModel:
			if 'tag' in x:
				tags[0].append(x)
				tags[1].append(self.dickDataModel[x])	 
		
		#closest one
		for i in  range(0,len(tags[0])):
			distance.append(sqrt((tags[1][i].position.x + self.pos.position.x)**2 + (tags[1][i].position.y - self.pos.position.y)**2))
			
		return distance






def main():
	lg = logTool()	
	pub_line_min_dist = rospy.Publisher('~line_min_dist', Marker, queue_size=1)
	while 1:
		#print("stan.. ",lg.status)
		#line1 = line(pub_line_min_dist,0,0)
		#time.sleep(2)
		#print("tessssst ================")
		#tags = lg.teoriticalGoal()
		#print tags
		#print(tags[1][1].position)
		pass
		
if __name__ == '__main__':
	main()





