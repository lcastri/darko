#!/usr/bin/env python3.8
from email.errors import NonPrintableDefect
import json
from causal_discovery.msg import ped_motion, causal_discovery
from causal_model import causal_model
import rospy
import utils
from constants import *
import threading
import time
import pandas as pd


df_data = pd.DataFrame()
df_traj = pd.DataFrame(columns = ['x','y'])
df_reset = True
    

def publish_model(model_json):
    """
    Publish model_json

    Args:
        model_json (JSON): JSON string to publish
    """
    
    # Building message
    causal_disc = causal_discovery()
    causal_disc.model = model_json
    
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


def cb_handle_human_traj(data):
    """
    Callback to handle new data on human trajectory

    Args:
        data (ped_traj): PoseArray data
    """
    global df_data
    global df_traj
    global df_reset
    
    # If TS_LENGTH reached then reset dataframes else append new data
    if df_reset:
        df_data = pd.DataFrame(columns = vars_name)
        df_traj = pd.DataFrame(columns = ['x','y'])
        log.info("Dataframes initialised")
        df_reset = False
   
    # TODO: how to take always the same pedestrian?
    x = data.ped_traj.poses[0].position.x
    y = data.ped_traj.poses[0].position.y
    if not df_traj.empty:
        x_old = df_traj.loc[df_traj.index[-1], 'x']
        y_old = df_traj.loc[df_traj.index[-1], 'y']

        # Compute and append new data for timeseries
        theta_g = utils.bearing(x, y, GOAL_X, GOAL_Y)
        d_g = utils.distance(x, y, GOAL_X, GOAL_Y)
        v = utils.velocity(x, y, x_old, y_old)
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
            
    # Append new data
    df_traj.loc[df_traj.shape[0]] = [x, y]


if __name__ == '__main__':
    # TODO: are the goal coordinates defined or they come from a message?
       
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
    sub_human_pose = rospy.Subscriber('/hri/motion_prediction', ped_motion, cb_handle_human_traj)
    pub_causal_model = rospy.Publisher('/hri/causal_discovery', causal_discovery, queue_size=10)
    while not rospy.is_shutdown():
        rate.sleep()
