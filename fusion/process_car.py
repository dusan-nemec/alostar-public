from structures import WheelParams, RobotState, SensorData
from pose_estimation_optim import PoseEstimator
import pandas as pd
import torch
import os
import numpy as np
from matplotlib import pyplot as plt
from utils import load_car, print_progress_bar, correct_initial_heading
from trainer import Trainer
from time import time

label = '2wd'
folder = 'results4'
useTraining = False

if label in ['1wd']:
    numWheels = 1
    wheels = [
        WheelParams(radius=0.295, xyz=(-0.29, 0.750, 0.2), steerable=False, side='R'), # rear-right
    ]
    wheelCols = [
        'W2',                                           # wheel speeds in rad/s
        'fi2',                                          # wheel angles in rad
        'S2',                                           # steering rates in rad/s
        'a2',                                           # steering angles in rad
    ]
    
elif label in ['2wd', 'odo2']:
    numWheels = 2
    wheels = [
        WheelParams(radius=0.295, xyz=(-0.29, 0.750, 0.2), steerable=False, side='R'), # rear-right
        WheelParams(radius=0.295, xyz=(-0.29, -0.750, 0.2), steerable=False, side='L') # rear-left
    ]
    wheelCols = [
        'W2', 'W4',                                     # wheel speeds in rad/s
        'fi2', 'fi4',                                   # wheel angles in rad
        'S2',  'S4',                                    # steering rates in rad/s
        'a2',  'a4',                                    # steering angles in rad
    ]

elif label in ['4wd', 'odo4']:
    numWheels = 4
    wheels = [
        WheelParams(radius=0.295, xyz=(2.32, 0.718, 0.2), steerable=True, side='R'), # front-right
        WheelParams(radius=0.295, xyz=(-0.29, 0.750, 0.2), steerable=False, side='R'), # rear-right
        WheelParams(radius=0.295, xyz=(2.32, -0.718, 0.2), steerable=True, side='L'), # front-left
        WheelParams(radius=0.295, xyz=(-0.29, -0.750, 0.2), steerable=False, side='L') # rear-left
    ]
    wheelCols = [
        'W1', 'W2', 'W3', 'W4',                         # wheel speeds in rad/s
        'fi1', 'fi2', 'fi3', 'fi4',                     # wheel angles in rad
        'S1', 'S2', 'S3', 'S4',                         # steering rates in rad/s
        'a1', 'a2', 'a3', 'a4',                         # steering angles in rad
    ]

dst_dir = os.path.dirname(os.path.abspath(__file__)) + f"/../{folder}/{label}"
os.makedirs(dst_dir, exist_ok=True)

performance = []

for trial_no in range(0,14):
    trial = "trial{:02d}".format(trial_no)
    df = load_car(trial)

    # run pose estimation
    est = PoseEstimator(wheelParams=wheels, calibIMU=True)
    if os.name != 'nt':
        est = torch.compile(est)
    else:
        print("Running on Windows, unable to compile the model")

    if useTraining:
        # try:
        #     est.load_state_dict(torch.load(dst_dir + '/model.pth'))
        # except:
        #     pass

        trainer = Trainer(est)

    paramCols = []
    for name, p in est.named_parameters():
        n = p.numel()
        if n > 1:
            for k in range(n):
                paramCols.append(name + '.' + str(k))
        else:
            paramCols.append(name)
    
    s = RobotState(numWheels)
    t = 0.0
    history = []

    for k, rec in df.iterrows():
        try:
            t_new = rec['t']
            sensors = SensorData()

            sensors.body = torch.tensor((rec['Gyr_X_BODY'], rec['Gyr_Y_BODY'], rec['Gyr_Z_BODY'], rec['Acc_X_BODY'], rec['Acc_Y_BODY'], rec['Acc_Z_BODY']))

            if numWheels == 1:
                sensors.wheels = [
                    torch.tensor((rec['Gyr_X_WH2'], rec['Gyr_Y_WH2'],rec['Gyr_Z_WH2'],rec['Acc_X_WH2'],rec['Acc_Y_WH2'],rec['Acc_Z_WH2'])),
                ]

            elif numWheels == 2:
                sensors.wheels = [
                    torch.tensor((rec['Gyr_X_WH2'], rec['Gyr_Y_WH2'],rec['Gyr_Z_WH2'],rec['Acc_X_WH2'],rec['Acc_Y_WH2'],rec['Acc_Z_WH2'])),
                    torch.tensor((rec['Gyr_X_WH4'], rec['Gyr_Y_WH4'],rec['Gyr_Z_WH4'],rec['Acc_X_WH4'],rec['Acc_Y_WH4'],rec['Acc_Z_WH4']))
                ]

            elif numWheels == 4:
                sensors.wheels = [
                    torch.tensor((rec['Gyr_X_WH1'], rec['Gyr_Y_WH1'],rec['Gyr_Z_WH1'],rec['Acc_X_WH1'],rec['Acc_Y_WH1'],rec['Acc_Z_WH1'])),
                    torch.tensor((rec['Gyr_X_WH2'], rec['Gyr_Y_WH2'],rec['Gyr_Z_WH2'],rec['Acc_X_WH2'],rec['Acc_Y_WH2'],rec['Acc_Z_WH2'])),
                    torch.tensor((rec['Gyr_X_WH3'], rec['Gyr_Y_WH3'],rec['Gyr_Z_WH3'],rec['Acc_X_WH3'],rec['Acc_Y_WH3'],rec['Acc_Z_WH3'])),
                    torch.tensor((rec['Gyr_X_WH4'], rec['Gyr_Y_WH4'],rec['Gyr_Z_WH4'],rec['Acc_X_WH4'],rec['Acc_Y_WH4'],rec['Acc_Z_WH4']))
                ]

            dt = t_new - t
            if dt < 0.0001:
                continue

            dt_tensor = torch.scalar_tensor(dt)

            ct0 = time()

            if label in ['odo2', 'odo4']:
                s = est.odometry(s, sensors, dt_tensor)
            else:
                s, corr = est.forward(s, sensors, dt_tensor)
                if useTraining:
                    trainer.process(corr)

            cpu_time = time() - ct0

            t = t_new
            pos_true = rec[['PosX_GNSS', 'PosY_GNSS', 'PosZ_GNSS']]
            vel_true = rec[['VelX_GNSS', 'VelY_GNSS', 'VelZ_GNSS']]

            params = [pv for p in est.parameters() for pv in p.detach().cpu().numpy().ravel().tolist() ]
            history.append((
                t, 
                *pos_true,
                *vel_true, 
                *s.xyz.tolist(),
                *s.linVelocity.tolist(), 
                *s.rpy.tolist(), 
                *s.angVelocity.tolist(),
                *est.linAcceleration.tolist(),
                *s.wheelSpeeds.tolist(),
                *s.wheelAngles.tolist(),
                *s.steerSpeeds.tolist(),
                *s.steerAngles.tolist(),
                *params,
                cpu_time,
                dt
            ))

            k = int(k)
            if k % 100 == 0:
                print_progress_bar(k+1, len(df), decimals=4, prefix="Progress:")
        except Exception as ex:
            print(f"Error: {ex}")
            break

    history = pd.DataFrame(history, columns=[
        't',                                            # time
        'PosX_GNSS', 'PosY_GNSS', 'PosZ_GNSS',          # GNSS position
        'VelX_GNSS', 'VelY_GNSS', 'VelZ_GNSS',          # linear velocity
        'PosX', 'PosY', 'PosZ',                         # estimated position
        'VelX', 'VelY', 'VelZ',                         # linear velocity
        'Roll', 'Pitch', 'Yaw',                         # body Euler angles
        'GyroX', 'GyroY', 'GyroZ',                      # body angular velocity
        'AccX', 'AccY', 'AccZ',                         # body acceleration
        
        *wheelCols,

        # 'Cor_VelX', 'Cor_VelY', 'Cor_VelZ',             # linear velocity corrections
        # 'Cor_AngVelX', 'Cor_AngVelY', 'Cor_AngVelZ',    # body angular velocity corrections
        # 'Cor_PosX', 'Cor_PosY', 'Cor_PosZ',             # position corrections (always zero)
        # 'Cor_Roll', 'Cor_Pitch', 'Cor_Yaw',             # body Euler angles corrections
        # 'Cor_W2', 'Cor_W4',         # wheel speeds in rad/s corrections
        # 'Cor_fi2', 'Cor_fi4',     # wheel angles in rad
        # 'Cor_S2', 'Cor_S4',         # steering rates in rad/s
        # 'Cor_a2', 'Cor_a4',         # steering angles in rad

        # 'Cor_W1', 'Cor_W2', 'Cor_W3', 'Cor_W4',         # wheel speeds in rad/s corrections
        # 'Cor_fi1', 'Cor_fi2', 'Cor_fi3', 'Cor_fi4',     # wheel angles in rad
        # 'Cor_S1', 'Cor_S2', 'Cor_S3', 'Cor_S4',         # steering rates in rad/s
        # 'Cor_a1', 'Cor_a2', 'Cor_a3', 'Cor_a4',         # steering angles in rad

        *paramCols,

        'cpu_time', 'dt'
    ])

    history = correct_initial_heading(history, n=10000)
    history.to_csv(dst_dir + f'/test_car_result_{trial}.csv', index=False)

    if useTraining:
        torch.save(est.state_dict(), dst_dir + '/model.pth')

print("DONE")