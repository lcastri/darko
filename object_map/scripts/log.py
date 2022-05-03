import logging
import os
from constants import *

class Log:
    def __init__(self, folder, filename, name):
        self.folder = folder
        self.filename = filename
        self.name = name
        
        if not os.path.exists(folder):
            os.makedirs(folder)
        
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)

        # create console handler and set level to debug
        ch = logging.FileHandler(filename = folder + '/' + filename)
        ch.setLevel(logging.DEBUG)

        # create formatter
        self.formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        # add formatter to ch
        ch.setFormatter(self.formatter)

        # add ch to logger
        self.logger.addHandler(ch)
        
    def info(self, msg):
        self.logger.info(msg)
    
log = Log(LOG_FOLDER, LOG_FILENAME, NODE_NAME)