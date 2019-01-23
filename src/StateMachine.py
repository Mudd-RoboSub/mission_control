#!/usr/bin/env python

import rospy
import smach
import smach_ros
import rospkg
import imp

from mission_control.msg import FibonacciAction, FibonacciGoal
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




# main
def main():
    rospy.init_node('smach_example_state_machine')
    rospack = rospkg.RosPack()
    pidPath = rospack.get_path('mission_control') + '/src/Tasks/Foo.py'
    rospy.loginfo("Path %s", pidPath)

    foo = imp.load_source('Foo', pidPath)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        fibonacci_goal = FibonacciGoal()
        fibonacci_goal.order = 20
        smach.StateMachine.add('Fbonacci',
                                   smach_ros.SimpleActionState('fibonacci', FibonacciAction,
                                                                goal=fibonacci_goal),
                                   {'succeeded':'FOO',
                                    'preempted':'BAR',
                                    'aborted':'BAR'})

        # Add states to the container
        smach.StateMachine.add('FOO', foo.Foo(),
                               transitions={'outcome1':'BAR',
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'outcome2':'FOO'})



    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
