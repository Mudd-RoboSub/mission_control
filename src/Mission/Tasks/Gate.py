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
			             output_keys=['angle'])
		self.leftConf=1
		self.rightConf=1
		self.gateSub = rospy.Subscriber("gateState", gate, self.gateCB)
		self.gatePub = rospy.Publisher("gateReset", Bool, latch=True, queue_size=1)
		self.left = None
		self.right = None
		
	def gateCB(self, data):
		 self.leftConf = data.leftConf
		 self.rightConf = data.rightConf
		 self.left, self.right = data.left, data.right
	def execute(self, userdata):
		self.gatePub.publish(True)
		rate = rospy.Rate(20)
		startTime = time()
		count = 0
		while not rospy.is_shutdown():
			rate.sleep()
			countTime = time()
			if count % 100 == 0:
				rospy.logwarn("leftConf %f, rightConf %f, divConf %f",self.leftConf, self.rightConf, self.divConf) 
			if countTime - startTime > 15:
				if self.leftConf > 2.75 and self.rightConf > 2.75:
					userdata.angle = (self.left + self.right) / 2
					return 'success'
				else:
					return 'failure'		

		#gateNone = [i for i in self.gate if i == None]
		
		
		#if len(gateNone) >= 15:
		
		#	return 'success'
		#else:
		#	userdata.angle += 20
		#	return 'failure'


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
                while self.leftConf < 2.75 or self.rightConf < 2.75:
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
	def execute(self,data):
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

		while leftConf < 2.75 or rightConf <2.75 or divConf < 2.75:
			count += 1
			rate.sleep()
			if time() - start > userdata.timeout:
				return 'abort'
		        if(prevLeftConf != leftConf or prevRightConf != rightConf or prevDivConf != divConf):
				rospy.loginfo("Section ID: Left conf {} div conf {} right conf {}".format(
						leftConf, divConf, rightConf))
				prevLeftConf, prevDivConf, prevRightConf = leftConf, divConf, rightConf
			if div is None:
				divNoneCount += 1			
			if count > 10 and divNoneCount / count > 0.7:
				return 'failure'

		if div - left > right - div:
			data.angle = (div+right)/2
		else:
			data.angle =  (div+left)/2
				
		return 'success'
		
		
		
# HAVE NOT IMPORT ROTATE TO
def getLocation(timeout,weightThreshold,yaw,direction):
	GetLocation = smach.StateMachine(outcomes=['success','abort'])
	GetLocation.userdata.timeout = timeout
	GetLocation.userdata.weightThreshold = weightThreshold
	GetLocation.userdata.incrementAngle = 20
	GetLocation.userdata.angle = None
	with GetLocation:
		smach.StateMachine.add('Locate',Locate(), 
			transitions={'success':'success','failure':'RotateToFindGate'},
			remapping={'timeout':'timeout','weightThreshold':'weightThreshold', 'angle':'angle'})
		smach.StateMachine.add('RotateToFindGate',RotateTo(yaw, increment=True, direction=direction),
			transitions={'success':'Locate','abort':'abort'},
			remapping={'angle':'incrementAngle','timeout':'timeout'})
	
	return GetLocation

# do we want to add a function that center ourself to the gate before moving??
def sanityCheck(timeout,yaw):
	
	SanityCheck = smach.StateMachine(outcomes = ['success', 'failure','abort'] )
	SanityCheck.userdata.timeout = timeout
	SanityCheck.userdata.rotateToCheckAngle = 10
	with SanityCheck:
		smach.StateMachine.add('Localize',Localize(yaw),
			transitions = {'success' : 'RotateToCheck', 'abort':'abort'},
			remapping = {'timeout':'timeout','angle':'firstPos'})
		smach.StateMachine.add('RotateToCheck',RotateTo(yaw),
			transitions = {'success' : 'ReLocalize' , 'abort' : 'abort'},
			remapping = {'angle':'rotateToCheckAngle', 'timeout':'timeout'})
		smach.StateMachine.add('ReLocalize', Localize(yaw),
			transitions = {'success': 'Check', 'abort': 'abort'},
			remapping = {'timeout': 'timeout', 'angle': 'secondPos'})
		smach.StateMachine.add('Check',Check(),
			transitions = {'success': 'GoToAngle', 'failure': 'failure'})
		smach.StateMachine.add('GoToAngle', RotateTo(yaw),
			transitions={'success':'success', 'abort':'abort'},
			remapping = {'angle':'secondPos', 'timeout':'timeout'})
	return SanityCheck

#maybe an abort option
#localize as 3 localize?? need option 'failure' to redo but not sure where to add
def approachGate( surge, sway, yaw, timeout):
	ApproachGate = smach.StateMachine(outcomes= ['success','abort'])
	ApproachGate.userdata.angle=0
	ApproachGate.userdata.speed = 0.2
	ApproachGate.userdata.moveTime = 5
	ApproachGate.userdata.timeout = timeout
	
	with ApproachGate:
		smach.StateMachine.add('Move',Move(surge,sway),
			transitions={'success':'SectionID',},
			remapping = {'angle':'angle','speed':'speed','moveTime':'moveTime'})
		smach.StateMachine.add('SectionID',SectionID(),
			transitions = {'success': 'RotateToSection','failure':'Move','abort':'abort'},
			remapping={'timeout':'timeout','angle':'angle'})
		smach.StateMachine.add('RotateToSection', RotateTo(yaw),
			transitions= {'success':'success', 'abort':'abort'},
			remapping = {'timeout':'timeout', 'angle':'angle'})

	return ApproachGate

def goToGate( surge, sway, timeout):
	GoToGate = smach.StateMachine ( outcomes=['success','abort'] )
	GoToGate.userdata.angle = 0
	GoToGate.userdata.speed = 0.2
	GoToGate.userdata.moveTime = 5
	GoToGate.userdata.timeout = timeout
	
	with GoToGate:
		smach.StateMachine.add('MoveToGate',Move(surge,sway),
			transitions = {'success':'success'},
			remapping = {'angle':'angle','speed':'speed','moveTime':'moveTime'})
	return GoToGate


def StateMachine(surge, sway, yaw, timeout,weightThreshold,direction ):
	GetLocation = getLocation(timeout,weightThreshold,yaw,direction)
	SanityCheck = sanityCheck(timeout,yaw)
	ApproachGate = approachGate(surge,sway,yaw,timeout)
	GoToGate = goToGate(surge,sway, timeout)
	
	Gate = smach.StateMachine(outcomes = ['success','abort'])
	Gate.userdata.angle = None
	Gate.userdata.timeout = timeout
	with Gate:
		smach.StateMachine.add('GetLocation',GetLocation,
					transitions={'success':'success','abort':'abort'},
					remapping={'angle':'angle'})
		smach.StateMachine.add('RotateToGate', RotateTo(yaw),
					transitions={'success':'success','abort':'abort'},
					remapping={'timeout':'timeout','angle':'angle'})
		#smach.StateMachine.add('SanityCheck',SanityCheck,
		#			transitions = {'success':'ApproachGate','failure':'GetLocation','abort':'abort'})
		smach.StateMachine.add('ApproachGate',ApproachGate,
					transitions = {'success': 'GoToGate'})
		smach.StateMachine.add('GoToGate',GoToGate,
					transitions = {'success': 'success','abort':'abort'})

	return Gate
