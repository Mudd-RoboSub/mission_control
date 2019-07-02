#!/usr/bin/env python

import rospy
import smach
import smach_ros
import rospkg
import imp
import sys
from Axis import Axis

#services
from mission_control.srv import *

from actionlib import *
from actionlib_msgs.msg import *

import roslib; roslib.load_manifest('mission_control')


'''
# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'
'''


def setEnabledClient(axis, setpoint):
    rospy.wait_for_service('EnabledService')
    try:
        enabledServiceProxy = rospy.ServiceProxy('EnabledService', EnabledService)
        res = enabledServiceProxy(axis, setpoint)
        return res.success
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e


# main
def main():
    rospy.init_node('smach_example_state_machine') # change name????
    rospack = rospkg.RosPack()
    post_gate_path = rospack.get_path('mission_control') + '/src/Mission/Tasks/Post_gate.py'
    pass_gate_path = rospack.get_path('mission_control') + '/src/Mission/Tasks/Pass_gate.py'
    rospy.loginfo("Path %s", post_gate_path)

    '''
    axis = "surge"
    value = False
    print "Requesting %s+%s"%(axis, value)
    print "%s + %s = %s"%(axis, value, setEnabledClient(axis, value))
    '''
    rospy.logwarn(0)
    surge = Axis("surge")
    sway = Axis("sway")
    heave = Axis("heave")
#    roll = Axis("roll")
#    pitch = Axis("pitch")
    yaw = Axis ("yaw")

    rospy.logwarn(1)
    heave.setEnabled(True)
    heave.setInput("DEPTH")
    heave.setSetpoint(10)
    rospy.logwarn(2)
    heave.setZero()
    rospy.logwarn(3)



    post_gate = imp.load_source('Post_gate', post_gate_path)
	pass_gate = imp.load_source('Pass_gate',pass_gate_path)


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['stop'])
	
	#sm.userdata = ???

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('Pass_gate', pass_gate.Pass_gate(surge,yaw),
                               transitions={'pass_gate_1':'Post_gate', 'pass_gate_2':'stop'}
							   )
        smach.StateMachine.add('Post_gate', post_gate.Post_gate(surge,yaw),
                               transitions={'complete_marker':'Pass_gate'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
