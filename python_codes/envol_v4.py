import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

kalman_data = {
    'time': [], 'x': [], 'y': [], 'z': [],
    'roll': [], 'pitch': [], 'yaw': []
}
lighthouse_data = {'x': [], 'y': [], 'z': []}
start_time = None
flowdeck_active = [False]
multiranger_active = [False]
lighthouse_status = [0]
extra_data = {
    'motor.m1': [], 'motor.m2': [], 'motor.m3': [], 'motor.m4': [],
    'stabilizer.thrust': [], 'stabilizer.roll': [], 'stabilizer.pitch': [], 'stabilizer.yaw': []
}

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
    kalman_data['roll'].append(data['stateEstimate.roll'])
    kalman_data['pitch'].append(data['stateEstimate.pitch'])
    kalman_data['yaw'].append(data['stateEstimate.yaw'])

def lighthouse_callback(timestamp, data, logconf):
    lighthouse_data['x'].append(data['lighthouse.x'])
    lighthouse_data['y'].append(data['lighthouse.y'])
    lighthouse_data['z'].append(data['lighthouse.z'])

def lighthouse_status_callback(timestamp, data, logconf):
    lighthouse_status[0] = data['lighthouse.status']

def extra_callback(timestamp, data, logconf):
    extra_data['motor.m1'].append(data['motor.m1'])
    extra_data['motor.m2'].append(data['motor.m2'])
    extra_data['motor.m3'].append(data['motor.m3'])
    extra_data['motor.m4'].append(data['motor.m4'])
    extra_data['stabilizer.thrust'].append(data['stabilizer.thrust'])
    extra_data['stabilizer.roll'].append(data['stabilizer.roll'])
    extra_data['stabilizer.pitch'].append(data['stabilizer.pitch'])
    extra_data['stabilizer.yaw'].append(data['stabilizer.yaw'])

# Creo que puedo quitar este def range_callback, probar
def range_callback(timestamp, data, logconf):
    z = data['range.zrange']
    if 0.05 < z < 3.0:
        flowdeck_active[0] = True

def battery_callback(timestamp, data, logconf):
    voltage = data['pm.vbat']
    print(f"🔋 Batería: {voltage:.2f} V")

# === Funciones de vuelo y setup ===
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
    use_correction = input("Vous voulez utiliser lighthouse.useCorrection? (1 = Oui, 0 = Non): ")
    while use_correction not in ['0', '1']:
        use_correction = input("Entrée non validée. Écrivez 1 (Oui) o 0 (Non): ")
    use_correction = int(use_correction)

    lh_method = input("¿Quel method de lighthouse vous voulez utiliser? (0 = Crossing Beams, 1 = Sweep Angle): ")
    while lh_method not in ['0', '1']:
        lh_method = input("Entrée non validée. Écrivez 0 (Crossing Beams) o 1 (Sweep Angle): ")
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

        # Logs
        logconf_bat = LogConfig(name='Battery', period_in_ms=500)
        logconf_bat.add_variable('pm.vbat', 'float')
        scf.cf.log.add_config(logconf_bat)
        logconf_bat.data_received_cb.add_callback(battery_callback)
        logconf_bat.start()
        time.sleep(1)
        logconf_bat.stop()

        # Kalman
        logconf_kalman = LogConfig(name='Kalman', period_in_ms=20)
        for var in [
            'stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
            'stateEstimate.roll', 'stateEstimate.pitch', 'stateEstimate.yaw'
        ]:
            logconf_kalman.add_variable(var, 'float')
        scf.cf.log.add_config(logconf_kalman)
        logconf_kalman.data_received_cb.add_callback(kalman_callback)
        logconf_kalman.start()

        # Lighthouse
        logconf_lh = LogConfig(name='Lighthouse', period_in_ms=20)
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

        # Flowdeck v2
        logconf_range = LogConfig(name='Range', period_in_ms=20)
        logconf_range.add_variable('range.zrange', 'float')
        scf.cf.log.add_config(logconf_range)
        logconf_range.data_received_cb.add_callback(range_callback)
        logconf_range.start()

        # Jules
        lc_ex = LogConfig(name='Extra', period_in_ms=20)
        for var in [
            'motor.m1', 'motor.m2', 'motor.m3', 'motor.m4',
            'stabilizer.thrust', 'stabilizer.roll',
            'stabilizer.pitch', 'stabilizer.yaw'
        ]:
            lc_ex.add_variable(var, 'float')
        scf.cf.log.add_config(lc_ex)
        lc_ex.data_received_cb.add_callback(extra_callback)
        lc_ex.start()

        # Trajectoire souhaitée
        trajectory = [
            (0, 0, 0.5, 2),
            (0.5, 0, 0.5, 2),
            (0.5, 0.5, 0.5, 2),
            (0, 0.5, 0.5, 2),
            (0, 0, 0.5, 2),
            (0, 0, 0, 1)
        ]

        # Ejecutar vuelo
        for x, y, z, duration in trajectory:
            send_position(scf.cf, x, y, z, duration)

        logconf_kalman.stop()
        logconf_lh.stop()
        logconf_lh_status.stop()
        logconf_range.stop()
        scf.cf.close_link()

    # Diagnóstico
    if lighthouse_status[0] == 2:
        print("✅ Lighthouse est utilisé comme une partie d'estimation (status = 2)")
    else:
        print(f"❌ Lighthouse n'est pas utilisé comme une partie d'estimation (status = {lighthouse_status[0]})")

    if flowdeck_active[0]:
        print("✅ Flowdeck v2 détecté")
    else:
        print("❌ Flowdeck v2 pas détecté")

    if multiranger_active[0]:
        print("✅ Multi-ranger deck détecté")
    else:
        print("❌ Multi-ranger deck pas détecté")

    # Exportar a CSV
    df = pd.DataFrame({
        'time': kalman_data['time'],
        'x_kalman': kalman_data['x'],
        'y_kalman': kalman_data['y'],
        'z_kalman': kalman_data['z'],
        'x_lh': lighthouse_data['x'],
        'y_lh': lighthouse_data['y'],
        'z_lh': lighthouse_data['z'],
        'roll': kalman_data['roll'],
        'pitch': kalman_data['pitch'],
        'yaw': kalman_data['yaw']
        'motor.m1': extra_data['motor.m1'],
        'motor.m2': extra_data['motor.m2'],
        'motor.m3': extra_data['motor.m3'],
        'motor.m4': extra_data['motor.m4'],
        'stabilizer.thrust': extra_data['stabilizer.thrust'],
        'stabilizer.roll': extra_data['stabilizer.roll'],
        'stabilizer.pitch': extra_data['stabilizer.pitch'],
        'stabilizer.yaw': extra_data['stabilizer.yaw']
    })
    df.to_csv("vuelo_datos.csv", index=False)
    print("Donnés exportés à vuelo_datos.csv")