from datetime import datetime
import logging
import os


VARS_FILENAME = 'vars.txt'
NODE_NAME = 'causal_discovery'
NODE_RATE = 10 #Hz
TS_LENGTH = 150 #seconds
ALPHA = 0.05

# LOG CONSTANTS
LOG_LEVEL = logging.INFO
LOG_FORMAT = "%(levelname)s %(asctime)s - %(message)s"
LOG_FOLDER = os.path.abspath(os.getcwd()) + '/logs'
LOG_FILENAME = "log_" + datetime.now().strftime("%d-%m-%Y_%H:%M:%S") + ".log"

# TODO: TEMPORARY
GOAL_X = 0
GOAL_Y = 0