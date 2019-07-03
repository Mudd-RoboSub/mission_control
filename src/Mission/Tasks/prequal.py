import rospy
import smach
import smach_ros
import rospkg
#depthPath = rospack.get_path('misssion_control') + "/src/Mission/Tasks"
#sys.path.append(depthPath)
#import GoToDepth
from vision.msg import gate
from std_msgs.msg import Bool, Float64
from time import time

"""
class Prequal(smach.State):
	
	# assume global data from allocation: angle, and error
	# p is the time before we check for reoritentation
	# r is the time we travel after we cannot see the gate bc of field of vision
	def __init__(self,surge,heave, yaw):
		smach.State.__init(self	,outcomes = ['success','abort'])
		self.surge =  surge
		self.yaw = yaw
		self.heave = heave

	def execute(self, userdata):
		
		rospy.loginfo("Start gate passing motion")
		self.surge.setZero()
		self.yaw.setZero()
		r=self.r
		p=self.p

		self.yaw.setSetpoint(angle)
		
		while( angle < error):
			self.surge.setSetpoint(p)

		self.surge.setSetpoint(r)
		if self.count<1:
			self.count+=1
			return 'pass_gate_1'
		else:
			return 'pass_gate_2'
"""

class Navigate(smach.State):

	def __init__(self,surge,yaw, initialDegree=20, timeToMarker= 1, timeBehindMarker=1, speed = 0.25):
		smach.State.__init__(self	,outcomes = ['success', 'abort'])
		self.surge =  surge
		self.yaw = yaw
		self.initialDegree = initialDegree
		self.timeToMarker = timeToMarker
		self.timeBehindMarker = timeBehindMarker
		self.speed = speed

	def execute(self, userdata):
		rospy.logwarn("1")
		self.yaw.setZero()
		rospy.logwarn("2")
		self.yaw.setSetpoint(-1* self.initialDegree)
		rospy.sleep(1)
		rospy.logwarn("2.5")
		self.surge.setControlEffort(self.speed)
		rospy.logwarn("3")
		rospy.sleep(self.timeToMarker)
		self.surge.setControlEffort(0)
		rospy.logwarn("4")
		self.yaw.setSetpoint(90+self.initialDegree)
		self.surge.setControlEffort(self.speed)
		rospy.sleep(self.timeBehindMarker)
		self.surge.setControlEffort(0)
		
		self.yaw.setSetpoint(90+self.initialDegree)
		'''
		self.yaw.setSetpoint(userdata.post_gate_input)
		
		while(userdata.post_gate_input < self.error):
			self.surge.setSetpoint(p)

		self.surge.setSetpoint(r)
		'''
		# now we transition to state machine pass_gate
		return "success"	

class Localize(smach.State):
	def __init__(self, yaw):
		smach.State.__init__(self	,outcomes = ['success', 'repeat','abort'],\
								output_keys=['done'])	
		self.localization = rospy.Subscriber("gateState", gate, self.gateCB)
		self.left, self.div, self.right = None, None, None
		self.leftConf, self.divConf, self.rightConf = None, None, None
		self.yaw = yaw
		
	def gateCB(self,data):
		self.left = data.left
		self.div = data.div
		self.right = data.right
		
		self.leftConf = data.leftConf
		self.rightConf = data.rightConf
		self.divConf = data.divConf
	
	def execute(self, userdata):
		pub = rospy.Publisher("gateReset", Bool, queue_size=1)
		
		#look for the gate
		start = time()		
		pub.publish(data=True)
		#TODO: at some point we should make this so we search
		while self.leftConf < 2.75 or self.rightConf < 2.75:
			rospy.loginfo("left conf: {} right conf: {}".format(self.leftConf, self.rightConf))
			rospy.sleep(.01)
			if time() - start > 100:
				return 'abort'
	
		goal = (self.left + self.right)/2 + yaw.plantState
		rospy.logwarn("THE GOAL IS TO ROTATE TO:{}".format(goal))
		print(goal)
		self.yaw.setSetpoint( goal )
	
		
		#rotate to the gate
		complete= False
		start = time()
		while not complete and not rospy.is_shutdown() :
			print("PLANT STATE", self.yaw.plantState)
			complete = goal - abs(self.yaw.plantState) < 3
			rospy.sleep(.01)
			if time()-start > 10:
				return 'abort'
		
		
		#check if we did it well
		pub.publish(data=True)
		while self.leftConf < 2.75 or self.rightConf < 2.75:
			rospy.sleep(.01)
			if time() - start > 100:
				return 'abort'

		goal = (self.left+self.right)/2 + yaw.plantState
		
		if abs(self.yaw.plantState-goal)<3:
			if abs(self.right-self.left)>30:
				userdata.done = True
				return 'success'
			else:
				return 'success'
		else:
			return "repeat"
		
		
class MoveToGate(smach.State):
	def __init__(self,surge, speed = 0.25):
		smach.State.__init__(self	,outcomes = ['success', 'abort','repeat'],input_keys=["done"])
		self.surge =  surge
		self.speed = speed

	def execute(self, userdata):
		
		self.surge.setControlEffort(self.speed)
		rospy.sleep(5)
		if userdata.done:
			rospy.sleep(5)
			self.surge.setControlEffort(0)
			return 'success'
		else:	
			self.surge.setControlEffort(0)
			return 'repeat'
		
		
		
		
		
	
	
		
	
