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
from math import sin
from math import cos
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Tag:
	def __init__(self,name = None, pose = Point()):
		self.name = name
		self.pose = pose
		self.distance = 0


	def dist(self,point):
		self.distance = (sqrt((point[0] - self.pose.position.x)**2 + (point[1] - self.pose.position.y)**2))
		 
class mark:
	def __init__(self,pub_cube):	
		self.pc = pub_cube		
		self.marker = Marker()
		self.marker.header.frame_id = "/odom"
		self.marker.type = self.marker.CUBE_LIST
		self.marker.action =self. marker.ADD
		# marker scale
		self.marker.scale.x = 0.14
		self.marker.scale.y = 0.14
		self.marker.scale.z = 0.01

	       # marker color
		self.marker.color.a = 1.0
		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
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
		self.marker.points = []	
    		# Publish the Marker
	
	def publish(self):
		self.pc.publish(self.marker)
	
class line:
	def __init__(self,pub_line_min_dist):
		

		self.pub_line_min_dist = pub_line_min_dist
		self.lenght = 7		
		self.marker = Marker()
		self.marker.header.frame_id = "/odom"
		self.marker.type = self.marker.LINE_LIST
		self.marker.action =self. marker.ADD
		# marker scale
		self.marker.scale.x = 0.03
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
		#first_line_point = Point()
		#first_line_point.x = startPoint[0]
		#first_line_point.y = startPoint[1]
		#first_line_point.z = 0.0
		#self.marker.points.append(first_line_point)


	
   		 # second point
		#second_line_point = Point()
		#second_line_point.x = first_line_point.x+self.lenght*cos(angle)
		#second_line_point.y = first_line_point.y+self.lenght*sin(angle)
		#second_line_point.z = 0.0
		#self.marker.points.append(second_line_point)

	 
	def publish(self):
    		# Publish the Marker
		self.pub_line_min_dist.publish(self.marker)

	


class logTool:
	
	def __init__(self):		
		self.lenght = 5
		self.stan= 0
		self.dictDataModel = {}
		self.lastGoalID = " "		
		self.target  = 0 #goal
		self.status =0
		self.wantedEndPose = 0 #estymation of perfect end position 
		rospy.init_node('loging', anonymous=False)
		#rospy.Subscriber("/move_base/status",GoalStatusArray, self.state)		
		rospy.Subscriber("/exact_pose",Odometry, self.posAqq)
		rospy.Subscriber("/gazebo/link_states",LinkStates, self.models)		
	

		pub_cube = rospy.Publisher('tag', Marker, queue_size=1)
		self.tag = mark(pub_cube) 
			
		pub_line = rospy.Publisher('~line_min_dist', Marker, queue_size=1)
		self.path = line(pub_line)

	def state(self,data):
		#method checking status of move basic(from topic)
		self.status = data.status	
		print("status ",self.status)

	def posAqq(self,data):
		#getting accual position of the robot(from topic)
		self.pos = data.pose.pose
		if self.stan == 1 and self.status == 0:
			self.stan = 0
			print("goal aqq")

	
	def dist(sPoint,endPoint):
		distance = (sqrt((sPoint[0] - endPoint[0])**2 + (sPoint[1] - endPoint[1])**2))
		return distance
	

	def models(self,data): 		
		#(form topic)
		names = data.name 
		tags = []
		self.tag.marker.points = []
		self.path.marker.points = []
		self.distances = []
		modelsPos = data.pose
		self.distClos = 99999
		self.closetsMarker = Tag()
	        self.dickDataModel = dict(zip(names,modelsPos)) 
		for x in self.dickDataModel:
			if 'tag' in x:
				tags.append(Tag(x,self.dickDataModel[x]))	
		for t in tags:		
			#print(t.pose)
			
			
			#markers
			cube = Point()
			cube.x = t.pose.position.x
			cube.y = t.pose.position.y
			cube.z = 0.0
			self.tag.marker.points.append(cube)		
			
			#closet one to the tobot 
			try:
				t.dist([self.pos.position.x,self.pos.position.y])
				if t.distance <= self.distClos:
					self.closetsMarker = t
					self.distClos = t.distance
			except:
				pass	
						
			#first point of line
			first_line_point = Point()
			first_line_point.x = t.pose.position.x
			first_line_point.y = t.pose.position.y
			first_line_point.z = 0.0
			self.path.marker.points.append(first_line_point)
			
			orientation_q = t.pose.orientation
			orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
			
   		 	# second point
			second_line_point = Point()
			second_line_point.x = first_line_point.x+self.lenght*cos(yaw)
			second_line_point.y = first_line_point.y+self.lenght*sin(yaw)
			second_line_point.z = 0.0
			self.path.marker.points.append(second_line_point)
			
		try:		
			print(self.closetsMarker.name)
		except:
			pass
		self.tag.publish()
		self.path.publish()

	


def main():
	lg = logTool()	 
		
		
	while 1:
		#print("stan.. ",lg.status)
			
		time.sleep(2)
		#print("tessssst ================")
		#tags = lg.teoriticalGoal()
		#print tags
		#print(tags[1][1].position)
		pass
		
if __name__ == '__main__':
	main()





