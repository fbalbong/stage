import time
import numpy as np
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

kalman_data = {'time': [], 'x': [], 'y': [], 'z': []}
kalman_data_measured = {'x': [], 'y': [], 'z': []}
start_time = None
phase_times = []

def kalman_callback(timestamp, data, logconf):
    global start_time
    if start_time is None:
        start_time = timestamp
    t = (timestamp - start_time) / 1000.0
    kalman_data['time'].append(t)
    kalman_data['x'].append(data['stateEstimate.x'])
    kalman_data['y'].append(data['stateEstimate.y'])
    kalman_data['z'].append(data['stateEstimate.z'])

def lighthouse_callback(timestamp, data, logconf):
    kalman_data_measured['x'].append(data['lighthouse.x'])
    kalman_data_measured['y'].append(data['lighthouse.y'])
    kalman_data_measured['z'].append(data['lighthouse.z'])

def battery_callback(timestamp, data, logconf):
    voltage = data['pm.vbat']
    print(f"🔋 Batería: {voltage:.2f} V")

def send_velocity(cf, vx, vy, vz, duration):
    dt = 0.01
    steps = int(duration / dt)
    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)
        time.sleep(dt)

def arm_dron():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Conectado. Enviando solicitud de ARM...")
        scf.cf.platform.send_arming_request(True)
        print("Dron armado.")

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    arm_dron()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.set_value('stabilizer.estimator', '2')
        scf.cf.param.set_value('lighthouse.method', '1')  # Activar Lighthouse
        reset_estimator(scf.cf)

        # Batería
        logconf_bat = LogConfig(name='Battery', period_in_ms=500)
        logconf_bat.add_variable('pm.vbat', 'float')
        scf.cf.log.add_config(logconf_bat)
        logconf_bat.data_received_cb.add_callback(battery_callback)
        logconf_bat.start()
        time.sleep(1)
        logconf_bat.stop()

        # Logging Kalman
        logconf = LogConfig(name='Kalman', period_in_ms=100)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(kalman_callback)
        logconf.start()

        # Logging Lighthouse
        logconf_lh = LogConfig(name='Lighthouse', period_in_ms=100)
        logconf_lh.add_variable('lighthouse.x', 'float')
        logconf_lh.add_variable('lighthouse.y', 'float')
        logconf_lh.add_variable('lighthouse.z', 'float')
        scf.cf.log.add_config(logconf_lh)
        logconf_lh.data_received_cb.add_callback(lighthouse_callback)
        logconf_lh.start()

        # Vuelo (10 s)
        send_velocity(scf.cf, 0, 0, 0.25, 2)
        send_velocity(scf.cf, 0, 0, 0.1, 1)
        send_velocity(scf.cf, 0, 0, 0.25, 2)
        send_velocity(scf.cf, 0.25, 0, 0, 2)
        send_velocity(scf.cf, 0, 0.25, 0, 2)
        send_velocity(scf.cf, 0, 0, 0, 1)

        logconf.stop()
        logconf_lh.stop()
        scf.cf.close_link()

    # Trayectoria deseada
    t_ref = np.array(kalman_data['time'])
    x_ref, y_ref, z_ref = [], [], []

    for t in t_ref:
        if t <= 2:
            x, y, z = 0, 0, 0.25 * t
        elif t <= 3:
            x, y, z = 0, 0, 0.5
        elif t <= 5:
            x, y, z = 0, 0, 0.25 * (t - 3) + 0.5
        elif t <= 7:
            x, y, z = 0.25 * (t - 5), 0, 1
        elif t <= 9:
            x, y, z = 0.5, 0.25 * (t - 7), 1
        else:
            x, y, z = 0.5, 0.5, 1
        x_ref.append(x)
        y_ref.append(y)
        z_ref.append(z)

    # Gráficas con 3 curvas
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t_ref, kalman_data['x'], label='x estimado (Kalman)')
    plt.plot(t_ref, kalman_data_measured['x'], label='x medido (Lighthouse)')
    plt.plot(t_ref, x_ref, '--', label='x deseado')
    plt.ylabel('X (m)')
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t_ref, kalman_data['y'], label='y estimado (Kalman)')
    plt.plot(t_ref, kalman_data_measured['y'], label='y medido (Lighthouse)')
    plt.plot(t_ref, y_ref, '--', label='y deseado')
    plt.ylabel('Y (m)')
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t_ref, kalman_data['z'], label='z estimado (Kalman)')
    plt.plot(t_ref, kalman_data_measured['z'], label='z medido (Lighthouse)')
    plt.plot(t_ref, z_ref, '--', label='z deseado')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Z (m)')
    plt.grid()
    plt.legend()

    plt.suptitle('Kalman vs Lighthouse vs Trayectoria Deseada')
    plt.tight_layout()
    plt.show()
