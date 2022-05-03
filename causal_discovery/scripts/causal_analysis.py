#!/usr/bin/env python3.8
import json
from causal_discovery.msg import CausalModel, Humans, SceneObjects
from causal_model import causal_model
import rospy
import utils
from constants import *
import threading
import time
import pandas as pd
import message_filters


df_data = pd.DataFrame()
df_reset = True
    

def publish_model(model_json):
    """
    Publish model_json

    Args:
        model_json (JSON): JSON string to publish
    """
    
    # Building message
    causal_disc = CausalModel()
    causal_disc.model = model_json
    # causal_disc.human = human
    # causal_disc.obj = obj
    
    # Publish message
    pub_causal_model.publish(causal_disc)
    log.info("Causal model published : " + model_json)
    
    
def t_multicausal(csv_id):
    """
    Causal analysis on file named csv_id

    Args:
        csv_id (str): csv ID to analyse
    """
    log.info("Causal analysis on file " + csv_id + " started")
    
    # Run causal analysis
    cm = causal_model(csv_id, vars_name, ALPHA)
    cm.run_causal_discovery_algorithm()
    log.info("Causal analysis on file " + csv_id + " completed")

    # Publish causal model in JSON format
    publish_model(json.dumps(cm.inference_dict))
    
    # Delete file .csv just analysed
    utils.delete_csv(csv_id)
    del cm
    
    
def t_fifocausal():
    """
    Causal analysis on files contained in folder data_pool with FIFO strategy
    """
    while True:
        list_datacsv = utils.file_in_folder(DATA_DIR)
        if len(list_datacsv) > 0:
            list_datacsv.sort()
            t_multicausal(list_datacsv[0])
        time.sleep(1)


def cb_handle_data(humans_pose, objs_pose):
    """
    Callback to handle new data on human trajectory

    Args:
        humans_pose (Humans): custom msg from T2.5
        objs_pose (SceneObjects): custom msg from MapServer
    """
    global df_data
    global df_reset
    log.debug("New human and object poses")
    
    # If TS_LENGTH reached then reset dataframes else append new data
    if df_reset:
        df_data = pd.DataFrame(columns = vars_name)
        log.info("Dataframes initialised")
        df_reset = False
   
    human, h_x, h_y, h_vx, h_vy = utils.handle_human_pose(humans_pose, selected_id=0)
    goal, goal_x, goal_y = utils.handle_obj_pose(objs_pose, selected_id=0)

    # Compute and append new data for timeseries
    theta_g = utils.bearing(h_x, h_y, goal_x, goal_y)
    d_g = utils.distance(h_x, h_y, goal_x, goal_y)
    v = utils.velocity(h_vx, h_vy)
    df_data.loc[df_data.shape[0]] = [theta_g, d_g, v]

    # Starting causal analysis if TS_LENGTH has been reached
    if (len(df_data) * 1/NODE_RATE) >= TS_LENGTH:
        # Saving dataframe into .csv file
        csv_id = utils.save_csv(df_data)
        log.info("Data saved into file named : " + csv_id)
        
        if CAUSAL_STRATEGY == causal_stategy.MULTI:
            # Starting causal analysis on .csv just created
            t_causality = threading.Thread(target = t_multicausal, args = (csv_id,))
            t_causality.start()
        
        # New dataframe
        df_reset = True


if __name__ == '__main__':       

    # Create data pool directory
    utils.create_data_dir()
    if CAUSAL_STRATEGY == causal_stategy.FIFO:
        # Starting thread checking for files .csv in data_pool folder
        t_causality = threading.Thread(target = t_fifocausal)
        t_causality.start()
    
    # Node info
    utils.print_node_info()
    rospy.init_node(NODE_NAME, anonymous=True)
    
    # Import log after ROS init_node
    from log import log
    
    # Reading variables name
    vars_name, vars_name_printable = utils.read_vars_name(VARS_FILENAME)

    rate = rospy.Rate(NODE_RATE)
    log.info("Ros node named " + str(NODE_NAME) + " initialized. Node rate : " + str(NODE_RATE) + "Hz")

    # Init subscriber & publisher
    sub_humans_pose = message_filters.Subscriber('/perception/humans', Humans)
    sub_objs_pose = message_filters.Subscriber('/mapping/scene_objects', SceneObjects)
    pub_causal_model = rospy.Publisher('/hri/causal_discovery', CausalModel, queue_size=10)

    # Init synchronizer and assigning a callback 
    ats = message_filters.ApproximateTimeSynchronizer([sub_humans_pose, sub_objs_pose], queue_size=NODE_RATE*TS_LENGTH, slop=0.2, allow_headerless=True)
    ats.registerCallback(cb_handle_data)

    while not rospy.is_shutdown():
        rate.sleep()
