import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool
from RotateTo import *
from Move import *
from vision.msg import *

#determine if gate is in sight, get rough approximation
#Kyle: changed existence to success or failure
class Locate(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['success','failure'], input_keys=['weightThreshold', 'timeout'], \
			             output_keys=['angle', 'div'])
		self.leftConf=1
		self.rightConf=1
		self.gateSub = rospy.Subscriber("gateState", gate, self.gateCB)
		self.gatePub = rospy.Publisher("gateReset", Bool, latch=True, queue_size=1)
		self.left = None
		self.right = None
		
	def gateCB(self, data):
		 self.leftConf = data.leftConf
		 self.rightConf = data.rightConf
		 self.divConf = data.divConf
		 self.left, self.right, self.div = data.left, data.right, data.div
	def execute(self, userdata):
		self.gatePub.publish(True)
		rospy.sleep(1)
		rate = rospy.Rate(20)
		startTime = time()
		count = 0
		while time() - startTime < 24 and not rospy.is_shutdown():
			rate.sleep()
			countTime = time()
			if count % 100 == 0:
				rospy.logwarn("leftConf %f, rightConf %f, divConf %f",self.leftConf, self.rightConf, self.divConf) 
			if self.leftConf > 13 and self.rightConf > 13:
				if self.divConf > 10:
					userdata.div = True
					leftDist = abs(self.div - self.left)
					rightDist = abs(self.div - self.right)
					if leftDist < rightDist:
						userdata.angle = (self.div + self.left) /2
					else:
						userdata.angle = (self.div + self.right)/2
				else:
					userdata.div = False
					userdata.angle = (self.left + self.right) / 2
				return 'success'
		userdata.angle=None
		return 'failure'		


"""
#find angle to gate (with much higher confidence)
class Localize(smach.State):
        def __init__(self, yaw):
                smach.State.__init__(self, outcomes = ['success', 'abort'], input_keys=['timeout'], output_keys=['angle'])
                self.localization = rospy.Subscriber("gateState", gate, self.gateCB)
                self.left, self.div, self.right = None, None, None
                self.leftConf, self.divConf, self.rightConf = None, None, None
                self.yaw = yaw

                self.pub = rospy.Publisher("gateReset", Bool, queue_size=1, latch=True)
        def gateCB(self,data):
                self.left = data.left
                self.div = data.div
                self.right = data.right

                self.leftConf = data.leftConf
                self.rightConf = data.rightConf
                self.divConf = data.divConf

        def execute(self, userdata):
                #look for the gate
                start = time()
                self.pub.publish(data=True)
                prevLeftConf , prevRightConf = 0,0
                while self.leftConf < 13 or self.rightConf < 13:
                        if(prevLeftConf != self.leftConf or prevRightConf != self.rightConf):
                                rospy.loginfo("left conf: {} right conf: {}".format(self.leftConf, self.rightConf))
                        rospy.sleep(.01)
                        prevLeftConf = self.leftConf
                        prevRightConf = self.rightConf
                        if time() - start > 30:
                                return 'abort'

                goal = (self.left + self.right)/2 + self.yaw.zeroedPlantState
                rospy.loginfo("THE GOAL IS TO ROTATE TO:{}".format(goal))
        	userdata.angle = goal        
	
                return 'success'

class Check(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys=['firstPos', 'secondPos'])

	def execute(self, data):
		firstPos = data.firstPos
		secondPos = data.secondPos
		if abs(firstPos - secondPos)  < 2 and secondPos < firstPos:
			return 'success'
		return 'failure'


class SectionID(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes = ['success','failure','abort'],input_keys=['timeout'],output_keys=['angle'])
		self.gate=()
		self.pub = rospy.Publisher('gateReset',Bool,queue_size=1, latch=True)
		rospy.Subscriber('gateState',gate,self.gateCB)
	def gateCB(self,data):
		self.gate=(data.left,data.div,data.right)
		self.confidence = (data.leftConf,data.divConf,data.rightConf)
	def execute(self,userdata):
		rate = rospy.Rate(20)
		start = time()
		self.pub.publish(data=True)
		left,div, right = self.gate
		
		#loop counter
		count = 0


		#how many times have we not seen the divider
		divNoneCount = 0


		leftConf, divConf,rightConf = self.confidence
		
		#to prevent spammy debug
		prevLeftConf, prevDivConf, prevRightConf = None, None, None
		
		while time() - start < 30 and not rospy.is_shutdown() and \
			 (leftConf < 13 or rightConf <13 or divConf < 13):
			rate.sleep()
			leftConf, rightConf, divConf = self.confidence
		        if(prevLeftConf != leftConf or prevRightConf != rightConf or prevDivConf != divConf):
				rospy.loginfo("Section ID: Left conf {} div conf {} right conf {}".format(
						leftConf, divConf, rightConf))
			prevLeftConf, prevDivConf, prevRightConf = leftConf, divConf, rightConf
		left, div, right = self.gate
		if divConf < 13:
			if rightConf > 13 and leftConf > 13:
				userdata.angle = (left + right) / 2
			return 'abort'

		if div - left > right - div:
			userdata.angle = (div+right)/2
		else:
			userdata.angle =  (div+left)/2
				
		return 'success'
		
"""
class Latch(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes = ['dedReckon','search','continue'], input_keys = ['div','latchIn','angle'],output_keys = ['latchOut'])
	
	def execute(self,userdata):
		if userdata.div:
			userdata.latchOut = True
		if userdata.angle is None and userdata.latchIn:
			return 'dedReckon'
		if userdata.angle is not None:
			return 'continue'	
		return 'search'

# HAVE NOT IMPORT ROTATE TO
def getLocation(timeout,weightThreshold,yaw,direction):
	GetLocation = smach.StateMachine(outcomes=['success','abort'], output_keys=['angle'])
	GetLocation.userdata.timeout = timeout
	GetLocation.userdata.weightThreshold = weightThreshold
	GetLocation.userdata.incrementAngle = 20
	with GetLocation:
		smach.StateMachine.add('Locate',Locate(), 
			transitions={'success':'success','failure':'RotateToFindGate'},
			remapping={'timeout':'timeout','weightThreshold':'weightThreshold', 'angle':'angle'})
		smach.StateMachine.add('RotateToFindGate',RotateTo(yaw, increment=True, direction=direction),
			transitions={'success':'Locate','abort':'abort'},
			remapping={'angle':'incrementAngle','timeout':'timeout'})
	
	return GetLocation



def approachGate( surge, sway, yaw, timeout):
	ApproachGate = smach.StateMachine(outcomes= ['success','abort','failure'], output_keys=['angle'])
	ApproachGate.userdata.angle=0
	ApproachGate.userdata.speed = 0.2
	ApproachGate.userdata.moveTime = 5
	ApproachGate.userdata.timeout = timeout
	ApproachGate.userdata.div = False
	ApproachGate.userdata.latch = False	
	with ApproachGate:
		smach.StateMachine.add('Move',Move(surge,sway),
			transitions={'success':'Relocate',},
			remapping = {'angle':'angle','speed':'speed','moveTime':'moveTime'})
		smach.StateMachine.add('Relocate', Locate(),
			transitions = {'success': 'RotateToSection','failure':'Latch'},
			remapping={'timeout':'timeout','angle':'angle', 'div':'div'})
		smach.StateMachine.add('Latch',Latch(),
			transitions={'dedReckon':'success','continue':'Move','search':'failure'},
			remapping={'latchIn':'latch','latchOut':'latch','angle':'angle', 'div':'div'})
		smach.StateMachine.add('RotateToSection', RotateTo(yaw,increment=True),
			transitions= {'success':'Latch', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'angle'})

	return ApproachGate







def StateMachine(surge, sway, yaw, timeout,weightThreshold,direction ):
	GetLocation = getLocation(timeout,weightThreshold,yaw,direction)
	#SanityCheck = sanityCheck(timeout,yaw)
	ApproachGate = approachGate(surge,sway,yaw,timeout)
	
	Gate = smach.StateMachine(outcomes = ['success','abort'], output_keys=['angle_out'])
	Gate.userdata.angle = None
	Gate.userdata.timeout = timeout
	Gate.userdata.zero = 0
	Gate.userdata.moveTime = 5
	Gate.userdata.speed = 0.3
	with Gate:
		smach.StateMachine.add('GetLocation',GetLocation,
					transitions={'success':'RotateToGate','abort':'abort'},
					remapping={'angle':'angle'})
		smach.StateMachine.add('RotateToGate', RotateTo(yaw, increment=True),
					transitions={'success':'ApproachGate','abort':'abort'},
					remapping={'timeout':'timeout','angle':'angle'})
		#smach.StateMachine.add('SanityCheck',SanityCheck,
		#			transitions = {'success':'ApproachGate','failure':'GetLocation','abort':'abort'})
		smach.StateMachine.add('ApproachGate',ApproachGate,
					transitions = {'success': 'GoToGate','failure':'GetLocation', 'abort':'abort'},
					remapping = {'angle':'angle_out'})
		smach.StateMachine.add('GoToGate',Move(surge,sway),
					transitions = {'success': 'success'},
					remapping = {'angle':'zero','speed':'speed','moveTime':'moveTime'})

	return Gate
