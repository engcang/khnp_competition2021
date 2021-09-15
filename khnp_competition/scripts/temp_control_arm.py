#!/usr/bin/env python

import time

import rospy
from trajectory_msgs.msg import JointTrajectory

import signal
import sys

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class mason_spawner():
    def __init__(self):
        rospy.init_node('mason_controller', anonymous=True)
        self.gt_subscriber = rospy.Subscriber('/joint_group_position_controller/command_no_arm', JointTrajectory, self.champ_cb)
        self.pub = rospy.Publisher('/joint_group_position_controller/command', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(1)
        self.pub_data=[0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0]

    def champ_cb(self, msg):
        msg.joint_names.append("arm_1_joint")
        msg.joint_names.append("arm_2_joint")
        msg.joint_names.append("arm_3_joint")
        msg.joint_names.append("arm_4_joint")
        msg.joint_names.append("arm_5_joint")
        msg.joint_names.append("arm_6_joint")
        for i in range(12):
            self.pub_data[i]=msg.points[0].positions[i]
        self.pub_data[12]=0.0
        self.pub_data[13]=-1.0
        self.pub_data[14]=-0.9
        self.pub_data[15]=0.0
        self.pub_data[16]=0.2
        self.pub_data[17]=0.0
        msg.points[0].positions=self.pub_data
        self.pub.publish(msg)


if __name__=='__main__':
    mas = mason_spawner()
    time.sleep(1)
    while True:
        try:
            mas.rate.sleep()	    
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
            sys.exit(0)
    sys.exit(0)
