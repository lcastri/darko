#!/usr/bin/env python
from motion_prediction.msg import ped_motion
import rospy
import utils
from geometry_msgs.msg import Pose, PoseArray
from constants import *
import pandas as pd


def publish_mot_pred(i):
    x = df_data.loc[i].x
    y = df_data.loc[i].y
    
    ped_mot = ped_motion()
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = 0.0
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 0.0
    ped_mot.ped_traj.header.frame_id = 'base_frame'
    ped_mot.ped_traj.poses.append(p)
    pub_mot_pred.publish(ped_mot)


if __name__ == '__main__':

    # Loading csv file into Dataframe
    with open(DATA_FILE, 'r') as file:
        df_data = pd.read_csv(file)
    df_i = 0 # row index initialization
    
    # Node info
    utils.print_node_info()
    rospy.init_node(NODE_NAME, anonymous=True)

    # Import log after ROS init_node
    from log import log
    
    rate = rospy.Rate(NODE_RATE)
    log.info(msg = "Ros node named " + NODE_NAME + " initialized. Node rate : " + str(NODE_RATE) + "Hz")

    # Init publisher
    pub_mot_pred = rospy.Publisher('/hri/motion_prediction', ped_motion, queue_size=10)

    while not rospy.is_shutdown():
        publish_mot_pred(df_i)
        df_i = df_i + 1
        if df_i >= df_data.shape[0]:
            break
        rate.sleep()
