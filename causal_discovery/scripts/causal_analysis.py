#!/usr/bin/env python
from causal_discovery.msg import ped_motion, causal_discovery
from causal_model import causal_model
import rospy
import utils
import logging
from importlib import reload
from constants import *

def publish_model(model_json):
    """
    Publish model_json

    Args:
        model_json (JSON): JSON string to publish
    """
    causal_disc = causal_discovery()
    causal_disc.model = model_json
    pub_causal_model.publish(causal_disc)
    logging.info("Causal model published")
    logging.info(model_json)


def cb_handle_human_traj(data):
    """
    Callback to handle new data on human trajectory

    Args:
        data (ped_traj): PoseArray data
    """
    logging.info("Ped_mot arrived")
    # TODO: how to take always the same pedestrian?
    ped_x = data.ped_traj.poses[0].position.x
    ped_y = data.ped_traj.poses[0].position.y
    if not cm.df_traj.empty:
        ped_x_old = cm.df_traj.tail(1)['x']
        ped_y_old = cm.df_traj.tail(1)['y']

        # Append new data for timeseries
        theta_g = utils.bearing(ped_x, ped_y, GOAL_X, GOAL_Y)
        d_g = utils.distance(ped_x, ped_y, GOAL_X, GOAL_Y)
        v = utils.velocity(ped_x, ped_y, ped_x_old, ped_y_old)
        new_data = {vars_name[0]: theta_g, vars_name[1]: d_g, vars_name[2]: v}
        cm.add_data(new_data)

        # Starting causal analysis if TS_LENGTH has been reached
        if (len(cm.df) * 1/NODE_RATE) >= TS_LENGTH:
            logging.info("Causal analysis started")
            cm.run_causal_discovery_algorithm()
            publish_model(cm.cm_json)
        
    cm.add_point([ped_x, ped_y])

if __name__ == '__main__':
    # TODO: are the goal coordinates defined or they come from a message?
    # TODO: do I need a thread for running PCMCI?
    utils.init_logger()

    # Causal model info
    vars_name, vars_name_printable = utils.read_vars_name(VARS_FILENAME)
    cm = causal_model(vars_name, ALPHA)
    logging.info("Causal model initialized")

    # Node info
    utils.print_node_info()
    rospy.init_node(NODE_NAME, anonymous=True)

    # Reload logger and handler after init_node
    reload(logging)
    utils.init_logger()
                    
    rate = rospy.Rate(NODE_RATE)
    logging.info("Ros node named " + str(NODE_NAME) + " initialized. Node rate : " + str(NODE_RATE) + "Hz")

    # Init subscriber & publisher
    sub_human_pose = rospy.Subscriber('/hri/motion_prediction', ped_motion, cb_handle_human_traj)
    pub_causal_model = rospy.Publisher('/hri/causal_discovery', causal_discovery, queue_size=10)
    while not rospy.is_shutdown():
        rate.sleep()
