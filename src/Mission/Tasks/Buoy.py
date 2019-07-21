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
import copy 
class Localize(smach.State):
	# if twoBuoy is true, then it needs two confidences. else just the confidence specificed
	# left is the left gate
	def __init__(self, twoBuoy,left):
		smach.State.__init__(self,outcomes = ['success','failure'],
					output_keys = ['angle'])
		self.buoySub = rospy.Subscriber('buoyState', buoy, self.buoyCB)
		self.twoBuoy=twoBuoy
		self.left=left
		self.firstYaw, self.firstYawConf = None, 1
		self.secondYaw, self.secondYawConf = None, 1		
		self.resetPub = rospy.Publisher("buoyReset", Bool, queue_size=1, latch=True)
	def buoyCB(self, data):
                self.firstYaw, self.firstYawConf = data.firstYaw, data.firstYawConf
                self.secondYaw, self.secondYawConf = data.secondYaw, data.secondYawConf

	def execute(self, userdata):
		self.resetPub.publish(data=True)
		rospy.sleep(1)
		startTime = time()
		rate = rospy.Rate(20)
		while time() - startTime < 15 and not rospy.is_shutdown():
			rospy.loginfo("CONF LEFT %f, CONF RIGHT %f", self.firstYawConf, self.secondYawConf)
			if self.twoBuoy:
				if self.firstYawConf > 2.0 and self.secondYawConf > 2.0:
					if self.left:
						userdata.angle = min(self.firstYaw,self.secondYaw)
					else:
						userdata.angle = max(self.firstYaw,self.secondYaw)
					return 'success'
			
			elif self.firstYawConf > 2:
					userdata.angle = self.firstYaw
					return 'success'
			rate.sleep()
		return 'failure'

class FindHeave(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failure', 'abort'],
				     input_keys = ['timeout', 'inDirection'], output_keys=['directionOut'])

		self.buoySub = rospy.Subscriber("buoyState", buoy, self.buoyCB)
		self.resetPub = rospy.Publisher("buoyReset", Bool, latch=True, queue_size=1)
                self.heave, self.heaveConf = None, 1
	def buoyCB(self, data):
			
                self.heave, self.heaveConf = data.firstHeave, data.firstYawConf

	def execute(self, userdata):
		
		inDirection = copy.deepcopy(userdata.inDirection)
                if inDirection is None:
			inDirection = 0
		self.resetPub.publish(data=True)
                while self.heaveConf > 2 and not rospy.is_shutdown():
                        rospy.sleep(0.01)

		startTime = time()
		success = False

		while time() - startTime < 15 and not rospy.is_shutdown():
			if self.heaveConf > 2:
				
				directionOut = 1 if self.heave > 0 else -1
				userdata.directionOut = directionOut
				success = True
				break
		if not success:
			return 'abort'
		rospy.logwarn("INDIRECTION %d, DIROUT %d", inDirection, directionOut)
		if directionOut == inDirection or inDirection == 0:
			return 'failure'
		else:
			return 'success'	

			
#class (smach.State):
#	def __init__(self):
#		smach.State.__init__(self,outcomes=
#left is whether it is left gate. Passes to localize
def searchRoutine(yaw,initialAngle,finalAngle,left=True):
	searchSpan = finalAngle - initialAngle
	direction = searchSpan > 0
	searchSpan = abs(searchSpan)
	numIt = ceil(searchSpan / 20)
	
	search = smach.Iterator(outcomes=['success', 'abort'],input_keys=[], output_keys=[],
				it= lambda: range(0, int(numIt)),
				it_label='index',
				exhausted_outcome='abort')

	search.userdata.initialAngle = initialAngle
	
	with search:
		container_sm = smach.StateMachine(outcomes = ['success', 'abort','continue'],
						output_keys=['outAngle'])
		container_sm.userdata.timeout = 500
		container_sm.userdata.angle = 20
		container_sm.userdata.initialAngle = initialAngle
		container_sm.userdata.outAngle = 0
		with container_sm:
			smach.StateMachine.add('InitialRotate', RotateTo(yaw, increment=True, direction=1),
				transitions={'success':'Localize','abort':'abort'},
				remapping={'timeout':'timeout','angle':'initialAngle'})
			smach.StateMachine.add('Rotate', RotateTo(yaw,increment=True, direction=direction),
				transitions={'success':'continue', 'abort':'abort'},
				remapping = {'timeout':'timeout', 'angle':'angle'})
			smach.StateMachine.add('Localize', Localize(True, left), 
				transitions={'success':'success', 'failure':'Rotate'},
				remapping={'angle':'outAngle'})
		smach.Iterator.set_contained_state('SEARCH', container_sm, loop_outcomes=['continue'])
		
	return search

def findFirstBuoy(yaw, heave, timeout):
	firstBuoy = smach.StateMachine(outcomes = ['success','abort'], input_keys=['angle_in'])
	firstBuoy.userdata.timeout = timeout
	firstBuoy.userdata.direction = None
	firstBuoy.userdata.initialDepth = 2.5
	firstBuoy.userdata.neg90 = -90
	firstBuoy.userdata.chosenBuoy = None
	firstBuoy.userdata.depthInc = 0.5
	firstBuoy.userdata.true = True
	firstBuoy.userdata.false = False
	firstBuoy.userdata.leftAngle = 0	

	with firstBuoy:
		smach.StateMachine.add('GoToDepth', GoToDepth(heave), 
			transitions = {'success': 'success', 'abort':'abort'},
			remapping = {'timeout':'timeout','depth':'initialDepth', 'increment':'false','direction':'true'})		

		smach.StateMachine.add('FirstRotate',RotateTo(yaw,increment=True), 
			transitions = {'success': 'Localize', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'angle_in'})

		smach.StateMachine.add('Localize', Localize(True,True),
			transitions = {'success':'RotateToBuoy', 'failure':'SearchRotate'},
			remapping = {'angle':'leftAngle'})

		smach.StateMachine.add('SearchRotate',RotateTo(yaw, increment=True),
			transitions = {'success':'Search', 'abort':'abort'},
			remapping={'timeout':'timeout','angle':'neg90'})

		smach.StateMachine.add('Search', searchRoutine(yaw, 0, 180),
			transitions = {'success':'RotateToBuoy', 'abort':'abort'},
			remapping = {'outAngle':'leftAngle'})

		smach.StateMachine.add('RotateToBuoy', RotateTo(yaw, increment=True),
			transitions = {'success':'FindHeave', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'leftAngle'})
		
		smach.StateMachine.add('FindHeave', FindHeave(),
			transitions = {'success':'success', 'failure':'IncrementDepth', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'inDirection':'direction',\
				     'directionOut':'direction'})

		smach.StateMachine.add('IncrementDepth', GoToDepth(heave),
			transitions= {'success':'FindHeave', 'abort':'abort'},
			remapping = {'depth':'depthInc', 'timeout':'timeout', 'increment':'true', 'direction':'direction'})
		
	return firstBuoy

def bumpBuoy(surge, sway, yaw, timeout):
	
        bump = smach.StateMachine(outcomes=['success','abort'])
	numIt = 3
	with bump:
		bump.userdata.back = 180
		bump.userdata.speed = 0.25
		bump.userdata.moveTime = 20

	        it = smach.Iterator(outcomes=['success', 'abort'], input_keys=[], output_keys=[],
                                it= lambda: range(0, int(numIt)),
                                it_label='index',
                                exhausted_outcome='success')
		with it:
			container_sm = smach.StateMachine(outcomes=['success','abort','continue'])
		
        		container_sm.userdata.moveTime = 6
			container_sm.userdata.backTime = container_sm.userdata.moveTime * numIt
        		container_sm.userdata.zero = 0
	        	container_sm.userdata.speed = 0.3
			container_sm.userdata.angle = None
			container_sm.userdata.timeout = timeout
			
			with container_sm:
				smach.StateMachine.add('Move', Move(surge,sway),
					transitions={'success':'ReLocalize'},
					remapping = {'angle':'zero','speed':'speed','moveTime':'moveTime'})
				smach.StateMachine.add('ReLocalize', Localize(False,True),
					transitions={'success':'RotateTo', 'failure':'abort'},
					remapping={'angle':'angle'})
				smach.StateMachine.add('RotateTo', RotateTo(yaw),
					transitions={'success':'continue', 'abort':'abort'},
					remapping={'timeout':'timeout','angle':'angle'})
			smach.Iterator.set_contained_state("BumpIt", container_sm, loop_outcomes=['continue'])

		smach.StateMachine.add("BUMP_IT", it,
			transitions={'success':'FinalBump', 'abort':'abort'})
		
		smach.StateMachine.add("FinalBump", Move(surge,sway), 
			transitions={'success':'MoveBack', 'abort':'abort'},
			remapping={'angle':'zero','speed':'speed','moveTime':'backTime'})
		smach.StateMachine.add("MoveBack", Move(surge,sway), 
			transitions={'success':'success'},
			remapping={'angle':'back','speed':'speed','moveTime':'moveTime'})
	return bump

def findSecondBuoy(yaw, heave, timeout):
	findBuoySmach = searchRoutine(yaw, 45, 315, False)

	findSecond = smach.StateMachine(outcomes=['success','abort'])
	findSecond.userdata.timeout = timeout
	findSecond.userdata.target = None
	findSecond.userdata.angle = None
	findSecond.userdata.direction = None
	findSecond.userdata.depthInc = 0.5

	with findSecond:
		smach.StateMachine.add("Search",findBuoySmach,
			transitions={'success':'RotateTo','abort':'abort'},
			remapping={'outAngle':'angle'})
		smach.StateMachine.add("RotateTo", RotateTo(yaw),
			transitions={'success':'FindHeave','abort':'abort'},
			remapping={'timeout':'timeout','angle':'angle'})

                smach.StateMachine.add('FindHeave', FindHeave(),
                        transitions = {'success':'success', 'failure':'IncrementDepth', 'abort':'abort'},
                        remapping = {'timeout':'timeout', 'inDirection':'direction',\
                                     'direction':'direction'})

                smach.StateMachine.add('IncrementDepth', GoToDepth(heave),
                        transitions= {'success':'FindHeave', 'abort':'abort'},
                        remapping = {'depth':'depthInc', 'timeout':'timeout', 'increment':'true', 'direction':'direction'})


	return findSecond
def StateMachine(surge, sway, heave, yaw, timeout):
	findFirst = findFirstBuoy(yaw, heave, timeout)
	bumpIt = bumpBuoy(surge, sway, yaw, timeout)
	findSecond = findSecondBuoy(yaw,heave,timeout)
	
	Buoy = smach.StateMachine(outcomes=['success','abort'], input_keys=['angle_in'])
	
	Buoy.userdata.count = 0

	with Buoy:
		smach.StateMachine.add('FindFirst', findFirst,
			transitions={'success':'BumpIt','abort':'abort'},
			remapping={'angle_in':'angle_in'})

		smach.StateMachine.add('BumpIt', bumpIt, 
			transitions={'success':'FindSecond','abort':'abort'})

		smach.StateMachine.add('FindSecond',findSecond,
			transitions={'success':'BumpSecond','abort':'abort'})
		smach.StateMachine.add('BumpSecond', bumpIt,
			transitions={'success':'success','abort':'abort'})


	return Buoy
