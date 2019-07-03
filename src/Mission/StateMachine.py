#!/usr/bin/env python

import rospy
import smach
import smach_ros
import rospkg
import imp
import sys
from Axis import Axis
from std_msgs.msg import Bool, Float64

#services
from mission_control.srv import *

from actionlib import *
from actionlib_msgs.msg import *

import roslib; roslib.load_manifest('mission_control')



# define state Bar
class Bar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome2'])

	def execute(self, userdata):
		rospy.loginfo('Executing state BAR')
		return 'outcome2'



def setEnabledClient(axis, setpoint):
	rospy.wait_for_service('EnabledService')
	try:
		enabledServiceProxy = rospy.ServiceProxy('EnabledService', EnabledService)
		res = enabledServiceProxy(axis, setpoint)
		return res.success
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e
	
def startCB(ud, data):
	return not data.data
# main
def main():
	rospy.init_node('state_machine')
	rospack = rospkg.RosPack()
	rospy.sleep(2)	
	prequalPath = rospack.get_path('mission_control') + '/src/Mission/Tasks/prequal.py'

	depthPath = rospack.get_path('mission_control') + '/src/Mission/Tasks/GoToDepth.py'

	surge = Axis("surge")
	heave = Axis("heave")
	yaw = Axis ("yaw")

	rospy.logwarn(1)
	heave.setEnabled(True)
	heave.setInput("DEPTH")
	
	yaw.setEnabled(False)
	yaw.setInput("IMU_POS")
	
        #yaw.setSetpoint(0)	
	done = False

	prequal = imp.load_source('prequal', prequalPath)
	goToDepth = imp.load_source('GoToDepth', depthPath)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['success', 'abort'])
	sm.userdata.done = False
	sm.userdata.count = 0
	sm.userdata.numReps = 4
	sm.userdata.depth = 1.3
	rospy.logwarn("BEFORE CONTAINER")

	# Open the container
	with sm:

		# Add states to the containe
		smach.StateMachine.add("WaitForStart", smach_ros.MonitorState("/start", Bool, startCB), transitions={'invalid':"GoToDepth", "valid":"WaitForStart", "preempted":"WaitForStart"})
		smach.StateMachine.add('GoToDepth', goToDepth.GoToDepth(heave, yaw),
							   transitions={'success':'Navigate',
											'abort':'abort'},
							   remapping={'depth':'depth'})
										 
											
		smach.StateMachine.add('Navigate', prequal.Navigate(surge,yaw),
							  transitions={'success':'success',
											'abort':'abort'})
												
		smach.StateMachine.add('Localize', prequal.Localize(yaw),
							  transitions={'success':'MoveToGate',
											'repeat':"Localize",
											'abort':'abort'})
		smach.StateMachine.add("MoveToGate",prequal.MoveToGate(surge),
								transitions={"success":"success",
											 "repeat":"Localize",
											 "abort":"abort"},
								remapping ={'count':"count", 'numReps':'numReps'})

	ready = False
        while not ready:
                ready = rospy.wait_for_message("/start", Bool)
                print("READY", ready)
	rospy.logwarn("AFTER CONTAINER")
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	for i in range(100):
		print("INTITALIZED")
	# Execute SMACH plan
        startPub = rospy.Publisher("thrustEnable", Bool, latch=True)	
	rospy.logwarn("SHOULD HAVE PUBLISHED BY NOW")
	yaw.setEnabled(False)
	yaw.setControlEffort(0)
 	startPub.publish(data=False)	
	outcome = sm.execute()
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()
