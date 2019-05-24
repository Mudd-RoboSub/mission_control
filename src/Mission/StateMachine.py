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


# main
def main():
    rospy.init_node('smach_example_state_machine')
    rospack = rospkg.RosPack()
    pidPath = rospack.get_path('mission_control') + '/src/Mission/Tasks/Foo.py'
    rospy.loginfo("Path %s", pidPath)

    '''
    axis = "surge"
    value = False
    print "Requesting %s+%s"%(axis, value)
    print "%s + %s = %s"%(axis, value, setEnabledClient(axis, value))
    '''

    surge = Axis("surge")
    sway = Axis("sway")
    heave = Axis("heave")
    roll = Axis("roll")
    pitch = Axis("pitch")
    yaw = Axis ("yaw")

    # surge.setEnabled(True)
    # surge.setSetpoint(10)
    # surge.setControlEffort(10)
    surge.setInput("CAM_FRONT")

    foo = imp.load_source('Foo', pidPath)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('FOO', foo.Foo(),
                               transitions={'outcome1':'BAR',
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'outcome2':'FOO'})



    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
