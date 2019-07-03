#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import Float64

class Yaw:
    def __init__(self):
        rospy.loginfo("HEREHERE")

        self.setpoint = 0
        self.plantState = 0
        self.plantTopic = ""

        self.setpointSub = rospy.Subscriber("/yawSetpoint", Float64, self.subCB)
        self.plantSub = rospy.Subscriber("/none", Float64, self.plantCB)
        while(not self.updatePlantTopic()):
            rospy.sleep(0.01)
        print("almost")

        self.plantPub = rospy.Publisher('/yawPlantState', Float64, queue_size=5)
        self.setpointPub = rospy.Publisher('/yawSetpointNorm', Float64, queue_size=5)
        self.prevPlantNorm = 0
        rospy.loginfo("init")

    def updatePlantTopic(self):
        topic = rospy.get_param("/TOPICS/YAW")
        if(len(topic) == 0):
            return False
        else:
            self.plantSub = rospy.Subscriber(topic, Float64, self.plantCB)
            self.plantTopic = topic
            return True

    def plantCB(self, data):
        self.plantState = data.data

    def subCB(self, data):
        self.setpoint = data.data

    def getNormalizedPlant(self):
        self.updatePlantTopic()
        distCW = abs(self.setpoint - self.plantState) % 360
        distCCW = -1*abs((360 - (self.setpoint - self.plantState))%360)
        if(abs(distCW) < abs(distCCW)):
            return distCW
        else:
            return distCCW

def main():
    rospy.init_node('yaw_normalize')
    yaw = Yaw()
    yaw.setpointPub.publish(data=0)

    while not rospy.is_shutdown():

        yaw.updatePlantTopic()
        plantNorm = yaw.getNormalizedPlant()
        if(plantNorm != yaw.prevPlantNorm):
            yaw.plantPub.publish(data=plantNorm)

        rospy.sleep(.05)
        prevPlantNorm = plantNorm


if __name__ == '__main__':
    main()
