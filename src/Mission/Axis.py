import numpy as np
import rospy
import rospkg
from mission_control.srv import *


class Axis:

    _axis = ""

    _enabled = False

    _inputs = {"IMU_POS" : 0, "IMU_ACCEL" : 1, "DEPTH" : 2, "CAM_FRONT": 3, "CAM_BOTTOM" : 4}

    def __init__(self, name):
        self._axis = name

    def setEnabled(self, val=True):
        self._enabled = val
        rospy.wait_for_service('EnabledService')
        try:
            enabledServiceProxy = rospy.ServiceProxy('EnabledService', EnabledService)
            res = enabledServiceProxy(self._axis, val)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def setControlEffort(self, val=0):
        rospy.loginfo("Disabling %s control loop", self._axis)
        self.setEnabled(False);
        self._enabled = False
        rospy.wait_for_service('ThrustOverrideService')
        try:
            enabledServiceProxy = rospy.ServiceProxy('ThrustOverrideService', ThrustOverrideService)
            res = enabledServiceProxy(self._axis, val)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e


    def setSetpoint(self, val=0):
        if(not self._enabled):
            rospy.logwarn("Make sure the loop is enabled")
        rospy.wait_for_service('SetpointService')
        try:
            enabledServiceProxy = rospy.ServiceProxy('SetpointService', SetpointService)
            res = enabledServiceProxy(self._axis, val)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def setInput(self, val):
        rospy.loginfo(self._inputs[val])

        if(not self._enabled):
            rospy.logwarn("Make sure the loop is enabled")
        rospy.wait_for_service('InputTypeService')
        try:
            enabledServiceProxy = rospy.ServiceProxy('InputTypeService', InputService)
            res = enabledServiceProxy(self._axis, self._inputs[val])
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e
