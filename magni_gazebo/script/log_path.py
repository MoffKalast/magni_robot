#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransformArray
from actionlib_msgs.msg import GoalStatusArray

class logTool:
	
	def __init__(self):
		self.stan= 1
		rospy.init_node('loging', anonymous=True)
		rospy.Subscriber("/move_base/status",GoalStatusArray, self.state)
		rospy.Subscriber("/fiducial_transforms",FiducialTransformArray, self.posMarker)
		rospy.Subscriber("/exact_pose",Odometry, self.posAqq)

	def state(self,data):
		self.status = data.status_list[0].status
		if self.status == 1:		
			self.stan = 1
		if self.stan == 1 and self.status==1:
			self.stan = 2	

	def posAqq(self,data):
		self.pos = data.pose
		if self.stan == 2 and self.status == 3:
			self.stan = 0
			print("goal aqq")
		
	def posMarker(self,data):
		if data.transforms:		
			pass
			print(data.transforms.position) 

	def teoriticalGoal(self,data):
		self.tPos = Odometry() #perfect position
		self.tPos.transforms 
		
		

def main():
	lg = logTool()
	try: 
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
  	


if __name__ == '__main__':
	main()
