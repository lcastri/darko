from datetime import datetime
import logging
import os


NODE_NAME = 'motion_prediction'
NODE_RATE = 10 #Hz

# LOG CONSTANTS
LOG_LEVEL = logging.INFO
LOG_FORMAT = "%(levelname)s %(asctime)s - %(message)s"
LOG_FOLDER = os.path.abspath(os.getcwd()) + '/logs'
LOG_FILENAME = "log_" + datetime.now().strftime("%d-%m-%Y_%H:%M:%S") + ".log"