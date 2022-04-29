from constants import *
from math import sqrt, atan2
from log import log
from os import listdir
from os.path import isfile, join
import shutil

def file_in_folder(dir_path):
    """
    List files in a specific folder

    Args:
        dir_path (str): folder path

    Returns:
        list[str]: list of files found in the folder
    """
    # count = 0
    # for path in os.scandir(dir_path):
    #     if path.is_file():
    #         count += 1
    files_list = [f for f in listdir(dir_path) if isfile(join(dir_path, f))]

    return files_list


def create_data_dir():
    """
    Create data directory
    """
    if os.path.exists(DATA_DIR):
        shutil.rmtree(DATA_DIR)
    os.makedirs(DATA_DIR)
        
        
def save_csv(df):
    """
    Save dataframe to .cvs

    Args:
        df (DataFrame): data to save to .csv

    Returns:
        str: csv ID
    """

    csv_id = "data_" + datetime.now().strftime("%d-%m-%Y_%H:%M:%S") + ".csv"
    df.to_csv(DATA_DIR + "/" + csv_id, index = False)
    return csv_id


def delete_csv(csv_id):
    """
    Delete a .csv file

    Args:
        csv_id (str): csv file to delete
    """
    csv_path = DATA_DIR + "/" + csv_id
    if(os.path.exists(csv_path) and os.path.isfile(csv_path)):
        os.remove(csv_path)
        log.info("Deleted data file named : " + csv_id)
    else:
        log.error(csv_id + " not found")


def print_node_info():
    """
    Print node's name, rate and other info
    """
    print("Node name : " + NODE_NAME)
    print("Node rate : " + str(NODE_RATE) + "Hz")
    print("Node log : " + LOG_FOLDER + '/' + LOG_FILENAME)


def read_vars_name(filename):
    """
    Read variables name from txt

    Args:
        data (ped_motion): PoseArray data

    Returns:
        vars_name (list(str)): list containing variables name
        vars_name_printable (list(str)): list containing variables name printable
    """
    log.info("Reading vars from : " + filename)

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

    log.info("Causal analysis on variables : " + str(vars_name))
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