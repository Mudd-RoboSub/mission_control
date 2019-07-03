# define state Foo
import rospy
import smach
import smach_ros
from time import time


class GoToDepth(smach.State):
    def __init__(self, heave):
        smach.State.__init__(self,
                             outcomes=['success', 'abort'],
							 input_keys=['depth'])
        self.heave = heave

    def execute(self, userdata):
		rospy.logwarn("6")
		self.heave.setSetpoint(userdata.depth)
		rospy.logwarn("7")
		loopRate = rospy.Rate(50)
		tStart = time()
		done = False
		count = 0
		while not done:
			while not rospy.is_shutdown() and time() - tStart < 1:
				loopRate.sleep()
			rospy.logwarn("8")
				
			if(abs(self.heave.plantState - userdata.depth) < .05):
				rospy.logwarn("9")
				done = True
			elif count > 10000000000:
				return 'abort'
			
			count += 1
		rospy.logwarn("10")
		return 'success'

