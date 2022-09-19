import pandas as pd
from utilsFS100 import FS100
# from tuya_connector import TuyaOpenAPI
import time
import numpy as np
from datetime import datetime
#from KasaSmartPowerStrip import SmartPowerStrip
import configparser

config = configparser.ConfigParser()
config_file = r'../../../0. WENDY/0. KULIAH BOY!!!/4. Semester 4/3. Robot/new/config.ini'
config.read(config_file)

ipAddress = (config['Robot']['Ip_Address'])

save_file = './' + str(datetime.now().timestamp() * 1000)

torques_dataframe = pd.DataFrame()

def run_fs100():
    print(ipAddress)
    robot = FS100('192.168.255.1')
    # Kassa Device
    #power_strip = SmartPowerStrip('192.168.0.2')
    # Tuya Power Meter data
    # ACCESS_ID = "vnupek4tjumdh7ftyxjt"
    # ACCESS_KEY = "a7995392cfea40c3a68ae564c36c855e"
    # API_ENDPOINT = "https://openapi.tuyaus.com"
    # # MQ_ENDPOINT = "wss://mqe.tuyacn.com:8285/"
    # openapi = TuyaOpenAPI(API_ENDPOINT, ACCESS_ID, ACCESS_KEY)
    # openapi.connect()
    # GP7_DEVICE_ID = "eba6e96f1e1c50f29cvgkm"

    pos_info = {}
    if FS100.ERROR_SUCCESS == robot.read_position(pos_info):
        print('Pos Information : ', pos_info)

        status = {}
    torques_dataframe = pd.DataFrame()
    if FS100.ERROR_SUCCESS == robot.get_status(status):
        if not status['servo_on']:
            robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

    load = 0
    # stops1 = [(453849, 181765, 222589, -1775731, 154644, 346028, 0),
    #           (422069, -246722, 222583, -1775729, 154653, -175317, 0),
    #           (482251, -264188, 194510, -1667400, -91614, -233674, 0),
    #           (453849, 181765, 222589, -1775731, 154644, 346028, 0),
    #           (422069, -246722, 222583, -1775729, 154653, -175317, 0),
    #           (482251, -264188, 194510, -1667400, -91614, -233674, 0),
    #           (453849, 181765, 222589, -1775731, 154644, 346028, 0)]

    #speed = 100
    speed = 400
    # speed = 800
    # speed = 1000
    # speed = 1200
    # speed = 1400
    # speed = 1600
    # speed = 1800
    # speed = [100, 100, 100,100, 100, 100, 100]
    # speed = [200, 200, 200,200, 200, 200, 200]
    # speed = [300, 500, 500,300, 500, 300, 600]
    # speed = [500, 500, 500,500, 500, 500, 500]
    # speed = [800, 800, 800,800, 800, 800, 800]
    # speed = [1000, 1000, 1000,1000, 1000, 1000, 1000]
    # speed = [1300, 1300, 1300,1300, 1300, 1300, 1300]
    # speed = [1500, 1500, 1500, 1500, 1500, 1500, 1500]
    # speed = [1700, 1700, 1700,1700, 1700, 1700, 1700]
    # speed = [1800, 1800, 1800,1800, 1800, 1800, 1800]
    # robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,FS100.MOVE_SPEED_CLASS_PERCENT, speed, stops1)

    # for index in range(stops1):
    #     x = stops1[index]
    #     speed_x = speed[index]
    #     robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,FS100.MOVE_SPEED_CLASS_PERCENT, speed, stops1)
        # robot.one_move(FS100.MOVE_TYPE_LINEAR_INCREMENTAL_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT, 1, speed_x, x)

    index = 0
    while True:
        torque_data = {}
        position_info = {}
        FS100.read_torque(robot, torque_data, 1)
        robot.read_position(position_info, 1)
        x, y, z, rx, ry, rz, re = position_info['pos']
        date_time = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")  # %I:%M:%S_%p
        now = datetime.now()
        Current_time = now.strftime("%H:%M:%S")
        torques_dataframe = torques_dataframe.append(
            {'DateTime': date_time, 'Time': Current_time, 'Position_X(mm)': x * 0.001,
             'Position_Y(mm)': y * 0.001,
             'Position_Z(mm)': z * 0.001, 'R_X(deg)': rx * 0.0001,
             'R_Y(deg)': ry * 0.0001, 'R_Z(deg)': rz * 0.0001 }, ignore_index=True)
        time.sleep(0.5)
        index += 1
        print(index)
        print(torques_dataframe)

        if index == 110:
            break

    torques_dataframe.to_csv(save_file + '.csv')



def main():
    run_fs100()



if __name__ == "__main__":
    main()
