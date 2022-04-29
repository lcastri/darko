from datetime import datetime
import logging
import os
from enum import Enum


class causal_stategy(Enum):
    MULTI = 0
    FIFO = 1
    

VARS_FILENAME = os.path.dirname(os.path.abspath(__file__)) + '/vars.txt'
NODE_NAME = 'causal_discovery'
NODE_RATE = 10 #Hz
TS_LENGTH = 150 #seconds
ALPHA = 0.05
CAUSAL_STRATEGY = causal_stategy.FIFO

# LOG CONSTANTS
LOG_LEVEL = logging.INFO
LOG_FORMAT = "%(levelname)s %(asctime)s - %(message)s"
LOG_FOLDER = os.path.dirname(os.path.abspath(__file__)) + '/logs'
LOG_FILENAME = "log_" + datetime.now().strftime("%d-%m-%Y_%H:%M:%S") + ".log"

# DATA DIR
DATA_DIR = os.path.dirname(os.path.abspath(__file__)) + '/data_pool'

# TODO: TEMPORARY
GOAL_X = 0
GOAL_Y = 0