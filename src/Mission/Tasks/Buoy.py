import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool
from RotateTo import *
from GoToDepth import *
from Move import *
from vision.msg import *
from math import ceil

class Localize(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes = ['success','failure'],
					output_keys = ['angle', 'chosenBuoy'])
		self.buoySub = rospy.Subscriber('buoyState', buoy, self.buoyCB)
		
		self.rightYaw, self.rightYawConf = None, 1
		self.leftYaw, self.leftYawConf = None, 1		

	def buoyCB(self, data):
                self.rightYaw, self.rightYawConf = data.rightYaw, data.rightYawConf
                self.leftYaw, self.leftYawConf = data.leftYaw, data.leftYawConf

	def execute(self, userdata):
		numFound = 0
		startTime = time()
		rate = rospy.Rate(20)
		while time() - startTime < 15 and not rospy.is_shutdown():
			if self.leftYawConf > 2.75 and self.rightyawConf > 2.75:
				numFound = 2
				break
			rate.sleep()
		if self.leftYawConf > 2.75 and numFound != 2:
			numFound = 1
			# 1 for right, -1 for left
			userdata.chosenBuoy = 1 if self.leftYaw > 0 else -1
			userdata.angle = self.leftYaw	
	
		elif self.rightYawConf > 2.75 and numFound != 2:
			numFound = 1
			userdata.chosenBuoy = 1  if self.rightYaw > 0 else -1
			userdata.angle = self.rightYaw

		if numFound == 0:
			return 'failure'
		else:
			return 'success'

class FindHeave(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failure', 'abort'],
				     input_keys = ['timeout', 'target', 'inDirection'], output_keys=['direction'])

                self.rightHeave, self.rightHeaveConf = None, 1
                self.leftHeave, self.leftHeaveConf = None, 1

	def buoyCB(self, data):
                self.rightHeave, self.rightHeaveConf = data.rightHeave, data.rightHeaveConf
                self.leftHeave, self.leftHeaveConf = data.leftHeave, data.leftHeaveConf
	

	def execute(self, userdata):
		while time() - startTime < 15 and not rospy.is_shutdown():
			if userdata.target == -1  and self.leftHeaveConf > 2.75:
				userdata.direction = 'up' if self.leftHeave > 0 else 'down'
				success = True
			elif userdata.target == 1 and self.rightHeaveConf > 2.75:
				userdata.direction = 'up' if self.rightHeave > 0 else 'down'
				success = True
		if not success:
			return 'abort'

		if userdata.direction == userdata.inDirection:
			return 'failure'
		else:
			return 'success'	

			
#class (smach.State):
#	def __init__(self):
#		smach.State.__init__(self,outcomes=
def searchRoutine(yaw,initialAngle,finalAngle):
	searchSpan = finalAngle - initialAngle
	direction = searchSpan > 0
	searchSpan = abs(searchSpan)
	numIt = ceil(searchSpan / 20)
	
	search = smach.Iterator(outcomes=['success', 'abort'],input_keys=[], output_keys=[],
				it= lambda: range(0, numIt),
				it_label='index',
				exhausted_outcome='abort')

	search.userdata.initialAngle = initialAngle
	
	with search:
		container_sm = smach.StateMachine(outcomes = ['success', 'failure','abort','continue'],
						output_keys=['outAngle', 'chosenBuoy'])
		container_sm.userdata.timeout = 500
		container_sm.userdata.angle = 20
		with container_sm:
			smach.StateMachine.add('InitialRotate', RotateTo(yaw, increment=True, direction=1),
				transitions={'success':'Rotate','abort':'abort'},
				remapping={'timeout':'timeout','angle':'initialAngle'})
			smach.StateMachine.add('Rotate', RotateTo(yaw,increment=True, direction=direction),
				transitions={'success':'Localize', 'abort':'abort'},
				remapping = {'timeout':'timeout', 'angle':'angle'})
			smach.StateMachine.add('Localize', Localize(), 
				transitions={'success':'success', 'failure':'continue'},
				remapping={'outAngle':'outAngle', 'chosenBuoy':'chosenBuoy'})
		smach.Iterator.set_contained_state('SEARCH', container_sm, loop_outcomes=['continue'])
		
	return search

def findFirstBuoy(yaw, heave, timeout, initialAngle):
	firstBuoy = smach.StateMachine(outcomes = ['success','abort'])
	firstBuoy.userdata.timeout = timeout
	firstBuoy.userdata.initialAngle = initialAngle
	firstBuoy.userdata.direction = None
	firstBuoy.userdata.initialDepth = 2.5
	firstBuoy.userdata.neg90 = -90
	firstBuoy.userdata.chosenBuoy = None
	firstBuoy.userdata.depthInc = 0.5
	firstBuoy.userdata.true = True
	firstBuoy.userdata.false = False
	

	with firstBuoy:
		smach.StateMachine.add('GoToDepth', GoToDepth(heave), 
			transitions = {'success': 'FirstRotate', 'abort':'abort'},
			remapping = {'timeout':'timeout','depth':'initialDepth', 'increment':'false','direction':'true'})		

		smach.StateMachine.add('FirstRotate',RotateTo(yaw,increment=True), 
			transitions = {'success': 'Localize', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'initialAngle'})

		smach.StateMachine.add('Localize', Localize(),
			transitions = {'success':'RotateToBuoy', 'failure':'SearchRotate'},
			remapping = {'angle':'BuoyOneAngle', 'chosenBuoy':'chosenBuoy'})

		smach.StateMachine.add('SearchRotate',RotateTo(yaw, increment=True),
			transitions = {'success':'Search', 'abort':'abort'},
			remapping={'timeout':'timeout','angle':'neg90'})

		smach.StateMachine.add('Search', searchRoutine(yaw, 0, 180),
			transitions = {'success':'RotateToBuoy', 'abort':'abort'},
			remapping = {'outAngle':'BuoyOneAngle', 'chosenBuoy':'chosenBuoy'})

		smach.StateMachine.add('RotateToBuoy', RotateTo(yaw),
			transitions = {'success':'FindHeave', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'BuoyOneAngle'})
		
		smach.StateMachine.add('FindHeave', FindHeave(),
			transitions = {'success':'success', 'failure':'IncrementDepth', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'target':'chosenBuoy', 'inDirection':'direction',\
				     'direction':'direction'})

		smach.StateMachine.add('IncrementDepth', GoToDepth(heave),
			transitions= {'success':'FindHeave', 'abort':'abort'},
			remapping = {'depth':'depthInc', 'timeout':'timeout', 'increment':'true', 'direction':'direction'})
		
	return firstBuoy

def bumpBuoy(surge, sway, yaw, timeout):
	
        bump = smach.StateMachine(outcomes=['success','abort'])
	numIt = 4
	with bump:
		bump.userdata.back = 180
		bump.userdata.speed = 0.25
		bump.userdata.moveTime = 20

	        it = smach.Iterator(outcomes=['success', 'abort'], input_keys=[], output_keys=[],
                                it= lambda: range(0, numIt),
                                it_label='index',
                                exhausted_outcome='abort')
		with it:
			container_sm = smach.StateMachine(outcomes=['success','abort','continue'])
		
        		container_sm.userdata.moveTime = 6
        		container_sm.userdata.zero = 0
	        	container_sm.userdata.speed = 0.3
			container_sm.userdata.angle = None
			container_sm.userdata.timeout = timeout
			with container_sm:
				smach.StateMachine.add('Move', Move(surge,sway),
					transitions={'success':'ReLocalize'},
					remapping = {'angle':'zero','speed':'speed','moveTime':'moveTime'})
				smach.StateMachine.add('ReLocalize', Localize(),
					transitions={'success':'RotateTo', 'failure':'abort'},
					remapping={'angle':'angle'})
				smach.StateMachine.add('RotateTo', RotateTo(yaw),
					transitions={'success':'continue', 'abort':'abort'},
					remapping={'timeout':'timeout','angle':'angle'})
			smach.Iterator.set_contained_state("CONTAINED", container_sm, loop_outcomes=['continue'])

		smach.StateMachine.add("BUMP_IT", it,
			transitions={'success':'success', 'abort':'abort'})
		
		
		smach.StateMachine.add("MoveBack", Move(surge,sway), 
			transitions={'success':'success'},
			remapping={'angle':'back','speed':'speed','moveTime':'moveTime'})
	return bump

def findSecondBuoy(yaw, heave, timeout):
	findBuoySmach = searchRoutine(yaw, 45, 315)

	findSecond = smach.StateMachine(outcomes=['success','abort'])
	findSecond.userdata.timeout = timeout
	findSecond.userdata.target = None
	findSecond.userdata.angle = None
	findSecond.userdata.direction = None
	findSecond.userdata.depthInc = 0.5

	with findSecond:
		smach.StateMachine.add("Search",findBuoySmach,
			transitions={'success':'RotateTo','abort':'abort'},
			remapping={'chosenBuoy':'target','outAngle':'angle'})
		smach.StateMachine.add("RotateTo", RotateTo(yaw),
			transitions={'success':'FindHeave','abort':'abort'},
			remapping={'timeout':'timeout','angle':'angle'})

                smach.StateMachine.add('FindHeave', FindHeave(),
                        transitions = {'success':'success', 'failure':'IncrementDepth', 'abort':'abort'},
                        remapping = {'timeout':'timeout', 'target':'chosenBuoy', 'inDirection':'direction',\
                                     'direction':'direction'})

                smach.StateMachine.add('IncrementDepth', GoToDepth(heave),
                        transitions= {'success':'FindHeave', 'abort':'abort'},
                        remapping = {'depth':'depthInc', 'timeout':'timeout', 'increment':'true', 'direction':'direction'})


	return findSecond
def StateMachine(surge, sway, heave, yaw, timeout,initialAngle):
	findFirst = findFirstBuoy(yaw, heave, timeout, initialAngle)
	bumpIt = bumpBuoy(surge, sway, yaw, timeout)
	findSecond = findSecondBuoy(yaw,heave,timeout)
	
	Buoy = smach.StateMachine(outcomes=['success','abort'])
	
	Buoy.userdata.count = 0

	with Buoy:
		smach.StateMachine.add('FindFirst', findFirst,
			transitions={'success':'BumpIt','abort':'abort'})

		smach.StateMachine.add('BumpIt', bumpIt, 
			transitions={'success':'FindSecond','abort':'abort'})

		smach.StateMachine.add('FindSecond',findSecond,
			transitions={'success':'BumpSecond','abort':'abort'})
		smach.StateMachine.add('BumpSecond', bumpIt,
			transitions={'success':'success','abort':'abort'})


	return Buoy
