from constants import *
import logging
import os

def print_node_info():
    """
    Print node's name, rate and other info
    """
    print("Node name : " + NODE_NAME)
    print("Node rate : " + str(NODE_RATE) + "Hz")
    print("Node log : " + LOG_FOLDER + '/' + LOG_FILENAME)

# Creating and Configuring Logger
def init_logger():
    """
    Create LOG_FOLDER (if needed) and then initialise logger

    Returns:
        Logger: logger instance
    """
    if not os.path.exists(LOG_FOLDER):
        os.makedirs(LOG_FOLDER)
        
    logger = logging.getLogger(NODE_NAME)
    logger.setLevel(logging.DEBUG)

    # create console handler and set level to debug
    ch = logging.FileHandler(filename = LOG_FOLDER + '/' + LOG_FILENAME)
    ch.setLevel(logging.DEBUG)

    # create formatter
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)
    
    return logger

