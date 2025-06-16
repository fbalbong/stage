import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
# import default_kalman
import matplotlib.pyplot as plt
from tkinter import *
from tkinter import messagebox, simpledialog

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Change the sequence according to your setup
#             x    y    z  YAW
sequence = [
    (0.0, 0.0, 0.3, 0)
]


def take_off_with_speed(cf, position):
    take_off_time = 2
    sleep_time = 0.01
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(f'take off at {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 300)
        time.sleep(sleep_time)

def take_off_with_position(cf, position):
    take_off_time = 1
    sleep_time = 0.01   
    steps = int(take_off_time / sleep_time)
    h = [i*position[2]/steps for i in range(steps)]

    print(f'take off at {position[2]}')

    for i in range(steps):
        cf.commander.send_zdistance_setpoint(0,0,0,h[i])
        time.sleep(sleep_time)


def position_callback(timestamp, data, logconf):
    ax = data['acc.x']
    ay = data['acc.y']
    az = data['acc.z']
    roll = data['gyro.xRaw'] #roll
    pitch = data['gyro.yRaw']
    yaw = data['gyro.zRaw']
    print('Data: (ax : {}, ay : {}, az : {}, roll : {}, pitch : {}, yaw : {})'.format(ax, ay, az, roll, pitch, yaw))
    with open("data2.txt", "a") as f:
        f.write(str([ax,ay,az,roll,pitch,yaw])+"\n")


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=10)
    log_conf.add_variable('acc.x', 'float')
    log_conf.add_variable('acc.y', 'float')
    log_conf.add_variable('acc.z', 'float')

    log_conf.add_variable('gyro.xRaw', 'float')
    log_conf.add_variable('gyro.yRaw', 'float')
    log_conf.add_variable('gyro.zRaw', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()




is_deck_attached = False


pose_est = [0, 0, 0, 0, 0, 0]
L = []

def log_pose_est_callback(timestamp, data, logconf):
    #print(data)
    global pose_est
    global L

    pose_est[0] = data['acc.x']
    pose_est[1] = data['acc.y']
    pose_est[2] = data['acc.z']
    pose_est[3] = data['gyro.xRaw']
    pose_est[4] = data['gyro.yRaw']
    pose_est[5] = data['gyro.zRaw']

    print('stateEstimate')
    print(pose_est)
    with open("data.txt", "a") as f:
        f.write(post_est)


def param_deck_flow(name, value_str):
    value = int(value_str)
    print(value)
    global is_deck_attached
    if value:
        is_deck_attached = True
        print('Deck is attached!')
    else:
        is_deck_attached = False
        print('Deck is NOT attached!')

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

def arm_dron():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print(" Conected. Sending request to ARM...")
        scf.cf.platform.send_arming_request(True)
        print(" Drone armed")

if __name__ == '__main__':
 
    cflib.crtp.init_drivers()
    arm_dron()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',cb=param_deck_flow)
        L=[]
        time.sleep(1)
        logconf = LogConfig(name='stateEstimate', period_in_ms=10)
        


        logconf.add_variable('acc.x', 'float')
        logconf.add_variable('acc.y', 'float')
        logconf.add_variable('acc.z', 'float')

        logconf.add_variable('gyro.xRaw', 'float')
        logconf.add_variable('gyro.yRaw', 'float')
        logconf.add_variable('gyro.zRaw', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pose_est_callback)
        scf.cf.param.set_value('stabilizer.estimator', '2')

        start_position_printing(scf)
        take_off_with_speed(scf.cf,(0,0,0.7,0))


        


        #scf.cf.commander.send_notify_setpoint_stop()
        scf.cf.close_link()


        # plt.plot(ax)
        # plt.plot(ay)
        # plt.plot(az)
        # plt.plot(roll)
        # plt.plot(yaw)
        # plt.plot(pitch)
        # plt.show()