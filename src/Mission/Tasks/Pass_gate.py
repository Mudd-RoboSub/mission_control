import rospy
import smach
import smach_ros

class Pass_gate(smach.State):
	
	# assume global data from allocation: angle, and error
	# p is the time before we check for reoritentation
	# r is the time we travel after we cannot see the gate bc of field of vision
	def __init__(self,surge,yaw,p=1,r=2 ):
		smach.State.__init(self	,outcomes = ['pass_gate_1,pass_gate_2'])
		self.surge =  surge
		self.yaw = yaw
		self.count =0
		self.r=r
		self.p=p

	def execute(self, userdata):
		
		rospy.loginfo("Start gate passing motion")
		self.surge.setZero()
		set.yaw.setZero()
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
		
		

