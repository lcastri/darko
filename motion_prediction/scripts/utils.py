from constants import *
import logging
import os

def print_node_info():
    """
    Print node's name, rate and other info
    """
    print("\n")
    print("Init node : " + NODE_NAME)
    print("Rate : " + str(NODE_RATE) + "Hz")

# Creating and Configuring Logger
def init_logger():
    if not os.path.exists(LOG_FOLDER):
        os.makedirs(LOG_FOLDER)

    logging.basicConfig(filename = LOG_FOLDER + "/" + LOG_FILENAME,
                        filemode = "a",
                        format = LOG_FORMAT, 
                        level = LOG_LEVEL)

