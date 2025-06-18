import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

kalman_data = {'time': [], 'x': [], 'y': [], 'z': []}
lighthouse_data = {'x': [], 'y': [], 'z': []}
start_time = None
flowdeck_active = [False]
multiranger_active = [False]
lighthouse_status = [0]

# === Callbacks ===
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
    lighthouse_data['x'].append(data['lighthouse.x'])
    lighthouse_data['y'].append(data['lighthouse.y'])
    lighthouse_data['z'].append(data['lighthouse.z'])

def lighthouse_status_callback(timestamp, data, logconf):
    lighthouse_status[0] = data['lighthouse.status']

def range_callback(timestamp, data, logconf):
    z = data['range.zrange']
    if 0.05 < z < 3.0:
        flowdeck_active[0] = True

def battery_callback(timestamp, data, logconf):
    voltage = data['pm.vbat']
    print(f"🔋 Batería: {voltage:.2f} V")

def send_position(cf, x, y, z, duration):
    dt = 0.1
    steps = int(duration / dt)
    for _ in range(steps):
        cf.commander.send_position_setpoint(x, y, z, 0)
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

def check_decks_attached(cf):
    decks = {
        'flow2': int(cf.param.get_value('deck.bcFlow2')),
        'multiranger': int(cf.param.get_value('deck.bcMultiranger'))
    }
    return decks

# === MAIN ===
if __name__ == '__main__':
    use_correction = input("¿Quieres usar lighthouse.useCorrection? (1 = sí, 0 = no): ")
    while use_correction not in ['0', '1']:
        use_correction = input("Entrada no válida. Escribe 1 (sí) o 0 (no): ")
    use_correction = int(use_correction)

    lh_method = input("¿Qué método de lighthouse quieres usar? (0 = Crossing Beams, 1 = Sweep Angle): ")
    while lh_method not in ['0', '1']:
        lh_method = input("Entrada no válida. Escribe 0 (Crossing Beams) o 1 (Sweep Angle): ")
    lh_method = int(lh_method)

    cflib.crtp.init_drivers()
    arm_dron()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.set_value('stabilizer.estimator', '2')
        scf.cf.param.set_value('lighthouse.useCorrection', str(use_correction))
        scf.cf.param.set_value('lighthouse.method', str(lh_method))
        reset_estimator(scf.cf)

        decks = check_decks_attached(scf.cf)
        flowdeck_active[0] = decks['flow2'] != 0
        multiranger_active[0] = decks['multiranger'] != 0

        logconf_bat = LogConfig(name='Battery', period_in_ms=500)
        logconf_bat.add_variable('pm.vbat', 'float')
        scf.cf.log.add_config(logconf_bat)
        logconf_bat.data_received_cb.add_callback(battery_callback)
        logconf_bat.start()
        time.sleep(1)
        logconf_bat.stop()

        logconf_kalman = LogConfig(name='Kalman', period_in_ms=100)
        for var in ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z']:
            logconf_kalman.add_variable(var, 'float')
        scf.cf.log.add_config(logconf_kalman)
        logconf_kalman.data_received_cb.add_callback(kalman_callback)
        logconf_kalman.start()

        logconf_lh = LogConfig(name='Lighthouse', period_in_ms=100)
        for var in ['lighthouse.x', 'lighthouse.y', 'lighthouse.z']:
            logconf_lh.add_variable(var, 'float')
        scf.cf.log.add_config(logconf_lh)
        logconf_lh.data_received_cb.add_callback(lighthouse_callback)
        logconf_lh.start()

        logconf_lh_status = LogConfig(name='LighthouseStatus', period_in_ms=500)
        logconf_lh_status.add_variable('lighthouse.status', 'uint8_t')
        scf.cf.log.add_config(logconf_lh_status)
        logconf_lh_status.data_received_cb.add_callback(lighthouse_status_callback)
        logconf_lh_status.start()

        logconf_range = LogConfig(name='Range', period_in_ms=100)
        logconf_range.add_variable('range.zrange', 'float')
        scf.cf.log.add_config(logconf_range)
        logconf_range.data_received_cb.add_callback(range_callback)
        logconf_range.start()

        trajectory = [
            (0, 0, 0.5, 2),
            (0.5, 0, 0.5, 2),
            (0.5, 0.5, 0.5, 2),
            (0, 0.5, 0.5, 2),
            (0, 0, 0.5, 2),
            (0, 0, 0, 1)
        ]

        for x, y, z, duration in trajectory:
            send_position(scf.cf, x, y, z, duration)

        logconf_kalman.stop()
        logconf_lh.stop()
        logconf_lh_status.stop()
        logconf_range.stop()
        scf.cf.close_link()

    # === Animación 3D ===
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([min(kalman_data['x']), max(kalman_data['x'])])
    ax.set_ylim([min(kalman_data['y']), max(kalman_data['y'])])
    ax.set_zlim([min(kalman_data['z']), max(kalman_data['z'])])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Trajectoire 3D Kalman vs Lighthouse')

    line_kalman, = ax.plot([], [], [], label='Kalman', color='blue')
    line_lh, = ax.plot([], [], [], label='Lighthouse', color='orange', linestyle='dashed')
    ax.legend()

    def init():
        line_kalman.set_data([], [])
        line_kalman.set_3d_properties([])
        line_lh.set_data([], [])
        line_lh.set_3d_properties([])
        return line_kalman, line_lh

    def update(frame):
        line_kalman.set_data(kalman_data['x'][:frame], kalman_data['y'][:frame])
        line_kalman.set_3d_properties(kalman_data['z'][:frame])
        line_lh.set_data(lighthouse_data['x'][:frame], lighthouse_data['y'][:frame])
        line_lh.set_3d_properties(lighthouse_data['z'][:frame])
        return line_kalman, line_lh

    frames = min(len(kalman_data['x']), len(lighthouse_data['x']))
    ani = animation.FuncAnimation(fig, update, init_func=init, frames=frames, interval=100, blit=True)

    plt.tight_layout()
    plt.show()
