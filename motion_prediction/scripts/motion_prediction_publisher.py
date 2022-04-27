#!/usr/bin/env python
from motion_prediction.msg import ped_motion
import rospy
import utils
from geometry_msgs.msg import Pose, PoseArray
from constants import *


def publish_mot_pred():
    ped_mot = ped_motion()
    p = Pose()
    p.position.x = 0.5
    p.position.y = -0.1
    p.position.z = 1.0
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 1.0
    ped_mot.ped_traj.header.frame_id = 'base_frame'
    ped_mot.ped_traj.poses.append(p)
    pub_mot_pred.publish(ped_mot)


if __name__ == '__main__':
   
    # Node info
    utils.print_node_info()
    rospy.init_node(NODE_NAME, anonymous=True)

    # Init logger after init_node  
    logger = utils.init_logger()

    rate = rospy.Rate(NODE_RATE)
    logger.info(msg = "Ros node named " + NODE_NAME + " initialized. Node rate : " + str(NODE_RATE) + "Hz")

    # Init publisher
    pub_mot_pred = rospy.Publisher('/hri/motion_prediction', ped_motion, queue_size=10)

    while not rospy.is_shutdown():
        publish_mot_pred()
        rate.sleep()
