# define state Foo
import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool


class GoToDepth(smach.State):
    def __init__(self, heave, yaw):
        smach.State.__init__(self,
                             outcomes=['success', 'abort'],
							 input_keys=['depth'])
        self.heave = heave
	self.yaw = yaw

    def execute(self, userdata):
		rospy.sleep(5)
		self.heave.setZero()
		self.yaw.setZero()
		enPub = rospy.Publisher("thrustEnable", Bool, latch=True)
		rospy.sleep(3)
		enPub.publish(data=True)
		rospy.logwarn("6")
		self.heave.setSetpoint(userdata.depth)
		rospy.logwarn("7")
		loopRate = rospy.Rate(50)
		tStart = time()
		done = False
		count = 0
                successCount = 0
		while not done:
			
			while not rospy.is_shutdown() and time() - tStart < 1:
				loopRate.sleep()
			if(abs(self.heave.plantState - self.heave.setpoint) < .04):
				rospy.logwarn("9")
				successCount += 1	
				if(successCount > 50):
					done = True
			elif count > 10000000000:
				return 'abort'
			else:
				 successCount = 0
			loopRate.sleep()
			count += 1
		rospy.logwarn("10")
		return 'success'

