#!/usr/bin/env python
from darko_perception_msgs.msg import SceneObject, SceneObjects
import rospy
import utils
from constants import *
import pandas as pd
import time

def publish_mot_pred(i):
    x = df_data.loc[i].x
    y = df_data.loc[i].y
    
    obj = SceneObject()
    obj.id = 0
    obj.pose.pose.position.x = x
    obj.pose.pose.position.y = y

    obj_list = SceneObjects()
    obj_list.objects.append(obj)
    obj_list.header.stamp = rospy.Time.now()
    pub_mot_pred.publish(obj_list)


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
    pub_mot_pred = rospy.Publisher('/mapping/scene_objects', SceneObjects, queue_size=10)
    time.sleep(5)

    while not rospy.is_shutdown():
        publish_mot_pred(df_i)
        df_i = df_i + 1
        if df_i >= df_data.shape[0]:
            break
        rate.sleep()
