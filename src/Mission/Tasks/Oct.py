import rospy
import smach
import smach_ros
from time import time
from std_msgs.msg import Bool
import math
from Move import *
from GoToDepth import *
from RotateTo import *
def oct(surge,sway,angle,time1,time2,):
	Oct=smach.StateMachine(outcomes = ['success','abort'])
	Oct.userdata.angle = 5
	Oct.userdata.time1= 20
	Oct.userdata.time2 = 1
	Oct.userdata.turnaround = 180
	Oct.userdata.timeout = 30000
	Oct.userdata.depth =1
	Oct.userdata.true = True
	OCt.userdata.neg = -1
	with Oct:
		smach.StateMachine.add('Turn',RotateTo(yaw,True,direction = 1),
			transitions = {'success':'Move1','abort':'abort'},
			remapping = {'angle':'turnaround','timeout':'timeout'})
		smach.StateMachine.add('Move1',Move(surge,sway), 
			transitions={'success':'Move1'},
			remapping = {'angle':'zero','speed':'speed','moveTime':'time1'})
		
		smach.StateMachine.add('Move2',Move(surge,sway), 
			transitions={'success':'success'},
			remapping = {'angle':'angle','speed':'speed','moveTime':'time2'})
		smach.StateMachine.add('Heave',GoToDepth(heave),
			transitions = {'success':'success','abort':'abort'},
			remapping ={'depth':'depth','timeout':'timeout','increment':'true','direction':'neg'})
	return Oct

