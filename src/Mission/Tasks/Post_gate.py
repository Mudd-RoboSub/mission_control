import rospy
import smach
import smach_ros

class Post_gate(smach.State):
	
	
	# n is the initial degree we turned
	# m is the time we travel toward the marker
	# k is the distance we travel behind the marker
	# p is the time we travel per post_gate_input
	# r is the time we travel after we are too close  to the gate
	# error is the degree of drifting that we are wiling to tolerate

	# assume post_gate_input gives angle and error


	def __init__(self,surge,yaw, n=20, m= 100, k=20, p=1, r= 5, error = 5 ):
		smach.State.__init(self	,outcomes = ['complete_marker'])
		self.surge =  surge
		self.yaw = yaw
		self.n=n
		self.m=m
		self.k=k
		self.p=p
		self.r=r
		self.error=error

	def execute(self, userdata):
		
		rospy.loginfo("Start post gate motion")
		self.surge.setZero()
		set.yaw.setZero()
		n=self.n
		m=self.m
		k=self.k
		r=self.r
		p=self.p

		self.yaw.setSetpoint(-n)
		self.surge.setSetpoint(m)
		self.yaw.setSetpoint(90+n)
		self.surge.setSetpoint(k)
		self.yaw.setSetpoint(90+n)
		'''
		self.yaw.setSetpoint(userdata.post_gate_input)
		
		while(userdata.post_gate_input < self.error):
			self.surge.setSetpoint(p)

		self.surge.setSetpoint(r)
		'''
		# now we transition to state machine pass_gate
		return "complete_marker"		
		
		

