import pandas as pd
from utilsFS100 import FS100
import time
import numpy as np
from datetime import datetime
import configparser

config = configparser.ConfigParser()
config_file = r'config.ini'
config.read(config_file)

ipAddress = (config['Robot']['Ip_Address'])

# save_file = './' + str(datetime.now().timestamp() * 1000)


# Sim_env = bool(int(config['Environment']['Simulation_Mode']))
# Sim_env = bool(int(config['Environment']['Simulation_Mode']))
# Sim_env = bool(int(config['Environment']['Simulation_Mode']))

def check_near(reached: tuple, true: tuple):
    its_near = False
    reached = np.array(reached)
    true = np.array(true)
    new_arr = np.absolute(np.divide(np.subtract(reached, true), 100)) < 0.3
    if new_arr.all():
        print('reached to a point')
        its_near = True
    return its_near


def run_fs100():
    print(ipAddress)
    robot = FS100('192.168.255.1')


    pos_info = {}
    if FS100.ERROR_SUCCESS == robot.read_position(pos_info):
        print('Pos Information : ', pos_info)

    load = 0
    '''1st Random PTP test'''
    # stops1 = [(453849, 181765, 222589, -1775731, 154644, 346028, 0),
    #           (422069, -246722, 222583, -1775729, 154653, -175317, 0),
    #           (482251, -264188, 194510, -1667400, -91614, -233674, 0),
    #           (453849, 181765, 222589, -1775731, 154644, 346028, 0),
    #           (422069, -246722, 222583, -1775729, 154653, -175317, 0),
    #           (482251, -264188, 194510, -1667400, -91614, -233674, 0),
    #           (453849, 181765, 222589, -1775731, 154644, 346028, 0)
    #           ]

    '''4 PTP Move test :'''
    stops1 = [(367246, 380031, 117569, -1799981, -26052, 34, 0),
              (367272, -386568, 117557, -1799988, -26070, 12, 0),
              (492303, -255612, -150736, 1799993, -26053, -3, 0),
              (492303, 321936, -150736, -1799993, -26053, 2, 0),
              (367246, 380031, 117569, -1799981, -26052, 34, 0)]

    speed = [200, 200, 200,200,200]
    Acce= 20
    # mode= ('movj')
    # speed = [100, 100, 100,100, 100, 100, 100]
    # speed = [200, 200, 200,200, 200, 200, 200]
    # speed = [300, 300, 300,300, 300, 300, 300]
    # speed = [500, 500, 500,500, 500, 500, 500]
    # speed = [800, 800, 800,800, 800, 800, 800]
    # speed = [1000, 1000, 1000,1000, 1000, 1000, 1000]
    # speed = [1300, 1300, 1300,1300, 1300, 1300, 1300]
    # speed = [1500, 1500, 1500, 1500, 1500, 1500, 1500]
    # speed = [1700, 1700, 1700,1700, 1700, 1700, 1700]
    # speed = [1800, 1800, 1800,1800, 1800, 1800, 1800]

    status = {}
    torques_dataframe = pd.DataFrame()
    for index in range(len(stops1)):
        if FS100.ERROR_SUCCESS == robot.get_status(status):
            if not status['servo_on']:
                robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)
            #
            ret = robot.one_move(FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_BASE,
                                 FS100.MOVE_SPEED_CLASS_PERCENT, speed[index], stops1[index])
            # ret = robot.one_move(FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_TOOL,
            #                      FS100.MOVE_SPEED_CLASS_PERCENT, speed[index], stops1[index])

            # ret1 = robot.one_move(FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
            #                       FS100.MOVE_SPEED_CLASS_PERCENT, speed[index + 1], stops1[index + 1])

            # print(ret1)
            # print(position_info['pos'])
            print(ret)
            indexx = 0
            is_it_near = False
            while not is_it_near:
                torque_data = {}
                position_info = {}
                FS100.read_torque(robot, torque_data, 1)
                robot.read_position(position_info, 1)
                x, y, z, rx, ry, rz, re = position_info['pos']
                print(x, y, z, rx, ry, rz, re)
                is_it_near = check_near(position_info['pos'], stops1[index])
                if position_info['pos'] == stops1[index]:
                    print("reached, going to new")
                    break

                date_time = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")  # %I:%M:%S_%p
                now = datetime.now()
                Current_time = now.strftime("%H:%M:%S")

                torques_dataframe = torques_dataframe.append(
                    {'DateTime': date_time, 'Time': Current_time, 'Position_X(mm)': x * 0.001,
                     'Position_Y(mm)': y * 0.001,
                     'Position_Z(mm)': z * 0.001, 'R_X(deg)': rx * 0.0001,
                     'R_Y(deg)': ry * 0.0001, 'R_Z(deg)': rz * 0.0001}, ignore_index=True)
                time.sleep(0.5)
                indexx += 1
                if indexx == 40:
                    break
                    # index +=1

    #torques_dataframe.to_csv(save_file + '.csv')
    torques_dataframe.to_csv('Movement_1225.csv')
    # torques_dataframe.to_csv('MOVJ_V40_A80_1.csv')
    # torques_dataframe.to_csv('MOVJ_V50_A60.csv')
    # torques_dataframe.to_csv('MOVJ_V50_A80.csv')
    # torques_dataframe.to_csv('MOVJ_V50_A100.csv')

def main():
    run_fs100()

if __name__ == "__main__":
    main()
