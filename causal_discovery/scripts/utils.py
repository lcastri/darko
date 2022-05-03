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


def velocity(vx, vy):
    """
    Compute absolute velocity from vx and vy 

    Args:
        vx (float): human velocity along x-axis
        vy (float): human velocity along y-axis

    Returns:
        v (float): absolute velocity of human
    """
    v = sqrt(vx ** 2 + vy ** 2)
    return v


def handle_obj_pose(data, selected_id):
    """
    new data on object position handler

    Args:
        data (SceneObjects): custom msg from MapServer

    Returns:
        obj: selected human
        x: position along x-axis
        y: position along y-axis
    """
    obj = next(obj for obj in data.objects if obj.id == selected_id)

    x = obj.pose.pose.position.x
    y = obj.pose.pose.position.y
    log.debug("Object pose received : (x " + str(x) + " - y " + str(y) + ")")

    return obj, x, y


def handle_human_pose(data, selected_id):
    """
    new data on human trajectory handler

    Args:
        data (Humans): custom msg from T2.5

    Returns:
        human: selected human
        x: position along x-axis
        y: position along y-axis
        vx: velocity along x-axis
        vy: velocity along y-axis
    """ 
    human = next(human for human in data.humans if human.id == selected_id)

    x = human.centroid.pose.position.x
    y = human.centroid.pose.position.y
    vx = human.velocity.twist.linear.x
    vy = human.velocity.twist.linear.y
    log.debug("Human pose received : (x " + str(x) + " - y " + str(y) + " - vx " + str(vx) + " - vy " + str(vy) + ")")

    return human, x, y, vx, vy