import pandas as pd
import numpy as np
from structures import GRAVITY
import os

def load_car(trial:str) -> pd.DataFrame:
    """ Loads car dataset """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    path = script_dir + '/../four_wheel_datasets_1/' + trial
    df = pd.read_csv(path + '/preprocessed.csv').astype('float32')

    # gyros = [
    #     'Gyr_X_BODY', 'Gyr_Y_BODY', 'Gyr_Z_BODY', 
    #     'Gyr_X_WH1', 'Gyr_Y_WH1', 'Gyr_Z_WH1',
    #     'Gyr_X_WH2', 'Gyr_Y_WH2', 'Gyr_Z_WH2',
    #     'Gyr_X_WH3', 'Gyr_Y_WH3', 'Gyr_Z_WH3',
    #     'Gyr_X_WH4', 'Gyr_Y_WH4', 'Gyr_Z_WH4',
    #     'Gyro_X_GNSS', 'Gyro_Y_GNSS', 'Gyro_Z_GNSS',
    # ]

    # # remove initial gyroscope bias
    # init = df.head(100).mean()
    # df[gyros] -= init[gyros]

    # # scale accelerometers to 1G
    # accels = [
    #     ['Acc_X_BODY', 'Acc_Y_BODY', 'Acc_Z_BODY'], 
    #     ['Acc_X_WH1', 'Acc_Y_WH1', 'Acc_Z_WH1'],
    #     ['Acc_X_WH2', 'Acc_Y_WH2', 'Acc_Z_WH2'],
    #     ['Acc_X_WH3', 'Acc_Y_WH3', 'Acc_Z_WH3'],
    #     ['Acc_X_WH4', 'Acc_Y_WH4', 'Acc_Z_WH4'],
    # ]

    # for acc in accels:
    #     scale = GRAVITY / np.linalg.norm(init[acc])
    #     df[acc] *= scale

    return df


def print_progress_bar(iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    https://stackoverflow.com/questions/3173320/text-progress-bar-in-terminal-with-block-characters
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()

def correct_initial_heading(df : pd.DataFrame, n:int = 10000) -> pd.DataFrame:
    """ optimizes the initial heading for getting the best match on first n samples """
    pts1 = df.head(n)[['PosX', 'PosY']].to_numpy()
    pts2 = df.head(n)[['PosX_GNSS', 'PosY_GNSS']].to_numpy()

    # find rotation
    H = pts1.T @ pts2
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    initHeading = np.atan2(R[1,0], R[0,0])
    pts = (R @ df[['PosX', 'PosY']].to_numpy().T).T

    df['PosX'] = pts[:,0]
    df['PosY'] = pts[:,1]
    df['Yaw'] += initHeading

    return df

