from constants import *
import logging
from math import sqrt, atan2
import pandas as pd
import os


def print_node_info():
    """
    Print node's name, rate and other info
    """
    print("\n")
    print("Init node : " + NODE_NAME)
    print("Rate : " + str(NODE_RATE) + "Hz")
    print("Inputs needed :")
    print("- human pose")
    print("- goal pose")


# Creating and Configuring Logger
def init_logger():
    if not os.path.exists(LOG_FOLDER):
        os.makedirs(LOG_FOLDER)

    logging.basicConfig(filename = LOG_FOLDER + "/" + LOG_FILENAME,
                        filemode = "a",
                        format = LOG_FORMAT, 
                        level = LOG_LEVEL)


def read_vars_name(filename):
    """
    Read variables name from txt

    Args:
        data (ped_motion): PoseArray data

    Returns:
        vars_name (list(str)): list containing variables name
        vars_name_printable (list(str)): list containing variables name printable
    """
    logging.info("Reading vars from : " + filename)

    # init empty vars_name list
    vars_name = list()
    vars_name_printable = list()

    # read .txt file
    with open(filename, 'r') as file:
        line = file.readline()
        while line:
            vars_name.append(line.strip())
            vars_name_printable.append(r'$' + str(line.strip()) + '$')
            line = file.readline()

    logging.info("Causal analysis on variables : " + str(vars_name))
    return vars_name, vars_name_printable
    

def distance(xa, ya, xb, yb):
    """
    Compute Euler distance between agent A and B

    Args:
        xa (float): x-coord A agent
        ya (float): y-coord A agent
        xb (float): x-coord B agent
        yb (float): y-coord B agent

    Returns:
        d (float): Euler distance
    """
    d = sqrt((xa - xb) ** 2 + (ya - yb) ** 2)
    return d


def bearing(xa, ya, xb, yb):
    """
    Compute angle between agent A and B

    Args:
        xa (float): x-coord A agent
        ya (float): y-coord A agent
        xb (float): x-coord B agent
        yb (float): y-coord B agent

    Returns:
        theta (float): angle between agent A and B
    """
    theta = atan2(yb - ya, xb - xa)
    return theta


def velocity(xa, ya, xa_old, ya_old):
    """
    Compute angle between agent A and B

    Args:
        xa (float): x-coord A agent (t)
        ya (float): y-coord A agent (t)
        xa_old (float): x-coord A agent (t-1)
        ya_old (float): y-coord A agent (t-1)

    Returns:
        v (float): velocity of agent A
    """
    v = sqrt((xa - xa_old) ** 2 + (ya - ya_old) ** 2)
    return v


def init_df(vars_name):
    """
    Init pandas DataFrame for causal discovery

    Args:
        vars_name (list(str)): list containing variables name
    """
    df = pd.DataFrame(columns = vars_name)