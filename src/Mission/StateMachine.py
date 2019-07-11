#!/usr/bin/env python

import rospy
import smach
import smach_ros
import rospkg
import imp
import sys
from Axis import Axis
from std_msgs.msg import Bool, Float64
from BetterConcurrence import *

#services
from mission_control.srv import *

from actionlib import *
from actionlib_msgs.msg import *

import roslib; roslib.load_manifest('mission_control')


class MonitorStart(smach.State):
	
	def __init__(self, target=True):
		smach.State.__init__(self, outcomes=['True', 'False'])
		self.startSub = rospy.Subscriber("start", Bool, self.startCB)	
		self.start = None
		self.target = target

	def startCB(self, data):
		self.start = data.data

	def execute(self, userdata):
		rate = rospy.Rate(20)
		
		while not rospy.is_shutdown():
			if self.start is not None and self.start == self.target:
				return 'True' if self.start else 'False'
			rate.sleep() 

class Reset(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'abort'])
		self.thrusterPub = rospy.Publisher("thrustEnable", Bool, latch=True, queue_size=1)	
	

	def execute(self, userdata):
		self.thrusterPub.publish(data=False)
		return 'success'

class Dead(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'kill'])
		self.a = 0
	def execute(self, userdata):
		self.__init__()
		rospy.logwarn("the value of a %d", self.a)
		self.a = 1
		try:
			while not rospy.is_shutdown():
				rospy.sleep(2)
				return 'success'
		except PreemptException:
			print("HERHE:LKEHR:LIHEN:LINFS:OHI*E:LNKSDF:HIE")
	def request_preempt(self):
		self._preempt_requested=True

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

	heave.setEnabled(True)
	heave.setInput("DEPTH")
	
	yaw.setEnabled(False)
	yaw.setInput("IMU_POS")
	
        #yaw.setSetpoint(0)	
	done = False

	prequal = imp.load_source('prequal', prequalPath)
	goToDepth = imp.load_source('GoToDepth', depthPath)


	rospy.logwarn("complete loading")

         ######################################## Mission code ############################################	

	# Create a SMACH state machine
	Mission = smach.StateMachine(outcomes=['success', 'abort'])
	Mission.userdata.done = False
	Mission.userdata.count = 0
	Mission.userdata.numReps = 4
	Mission.userdata.depth = 1.3
	rospy.logwarn("BEFORE CONTAINER")

	# Open the container
	with Mission:

		# Add states to the containe
		#smach.StateMachine.add("WaitForStart", smach_ros.MonitorState("/start", Bool, startCB), transitions={'valid':"GoToDepth", "invalid":"WaitForStart", "preempted":"WaitForStart"})
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



	###########################################Concurrence Container########################################

	def childTermCB(outcome_map):
		if outcome_map['MonitorKill'] == 'False':
			return True
		return False
			

	def outcomeCB(outcome_map):
		if outcome_map['MonitorKill'] == 'False':
			print("AAAAAAAAAAAAAAHHHHHHHHHHHHHHHH")
			return 'kill'
		return 'success'

	SafeExecute = BetterConcurrence(outcomes = ['kill','success'], default_outcome='kill', 
			 outcome_map={'success':{'Mission':'success'}, 'kill':{'MonitorKill':'False'}})
					
	with SafeExecute:
		BetterConcurrence.add("MonitorKill", MonitorStart(target=False))
		BetterConcurrence.add("Mission", Dead())

	



	######################################    TOP    #########################
	Top = smach.StateMachine(outcomes=['abort', 'success'])
	with Top:
		smach.StateMachine.add('WaitForStart', MonitorStart(), 
					transitions={'True':'Run', 'False':'WaitForStart'})
		smach.StateMachine.add("Run", SafeExecute, transitions={'success':'Reset', 'kill': 'Reset'})
		smach.StateMachine.add("Reset", Reset(), transitions={'success':'WaitForStart', 'abort':'Dead'})
		smach.StateMachine.add("Dead", Dead(), transitions={'success':'Dead', 'kill':'Dead'})


	ready = False
        while not ready:
                ready = rospy.wait_for_message("/start", Bool)
                print("READY", ready)
	rospy.logwarn("AFTER CONTAINER")
	sis = smach_ros.IntrospectionServer('server_name', Top, '/SM_ROOT')
	sis.start()
	for i in range(100):
		print("INTITALIZED")
	# Execute SMACH plan
        startPub = rospy.Publisher("thrustEnable", Bool, latch=True)	
	rospy.logwarn("SHOULD HAVE PUBLISHED BY NOW")
	yaw.setEnabled(False)
	yaw.setControlEffort(0)
 	startPub.publish(data=False)	
	outcome = Top.execute()
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()
