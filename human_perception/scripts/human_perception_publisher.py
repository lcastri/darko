#!/usr/bin/env python
from darko_perception_msgs.msg import Humans, Human
import rospy
import utils
from constants import *
import pandas as pd
import time

def publish_humans(i):
    x = df_data.loc[i].x
    y = df_data.loc[i].y
    if i == 0:  
        vx = 0
        vy = 0
    else:
        vx = df_data.loc[i].x - df_data.loc[i-1].x
        vy = df_data.loc[i].y - df_data.loc[i-1].y
    
    h = Human()
    h.id = 22
    
    h.centroid.pose.position.x = x
    h.centroid.pose.position.y = y
    h.velocity.twist.linear.x = vx
    h.velocity.twist.linear.y = vy

    h_list = Humans()
    h_list.humans.append(h)
    h_list.header.stamp = rospy.Time.now()
    pub_mot_pred.publish(h_list)


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
    pub_mot_pred = rospy.Publisher('/perception/humans', Humans, queue_size=10)
    time.sleep(5)

    while not rospy.is_shutdown():
        publish_humans(df_i)
        df_i = df_i + 1
        if df_i >= df_data.shape[0]:
            break
        rate.sleep()
