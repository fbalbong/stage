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

        # Logs
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

        # Trajectoire souhaitée
        trajectory = [
            (0, 0, 0.5, 2),
            (0.5, 0, 0.5, 2),
            (0.5, 0.5, 0.5, 2),
            (0, 0.5, 0.5, 2),
            (0, 0, 0.5, 2),
            (0, 0, 0, 1)
        ]

        # Crear referencias
        x_ref, y_ref, z_ref, t_ref = [], [], [], []
        t = 0
        for x, y, z, duration in trajectory:
            steps = int(duration / 0.1)
            for _ in range(steps):
                x_ref.append(x)
                y_ref.append(y)
                z_ref.append(z)
                t_ref.append(t)
                t += 0.1

        # Ejecutar vuelo
        # scf.cf.param.set_value('commander.enHighLevel', '1')  # Activar comandos de alto nivel
        # commander = scf.cf.high_level_commander

        # commander.takeoff(0.5, 2.0)
        for x, y, z, duration in trajectory:
            send_position(scf.cf, x, y, z, duration)
        # commander.land(0, 2.0)
    


        logconf_kalman.stop()
        logconf_lh.stop()
        logconf_lh_status.stop()
        logconf_range.stop()
        scf.cf.close_link()

    # Diagnóstico
    if lighthouse_status[0] == 2:
        print("✅ Lighthouse se está usando como fuente de estimación (status = 2)")
    else:
        print(f"❌ Lighthouse no se está usando como fuente de estimación (status = {lighthouse_status[0]})")

    if flowdeck_active[0]:
        print("✅ Flowdeck v2 detectado")
    else:
        print("❌ Flowdeck v2 no detectado")

    if multiranger_active[0]:
        print("✅ Multi-ranger deck detectado")
    else:
        print("❌ Multi-ranger deck no detectado")

    # Alinear longitudes
    n = len(kalman_data['x'])
    t_ref = t_ref[:n]
    x_ref = x_ref[:n]
    y_ref = y_ref[:n]
    z_ref = z_ref[:n]

    # Gráficas
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t_ref, kalman_data['x'], label='x estimé')
    plt.plot(t_ref, x_ref, '--', label='x souhaité')
    if len(lighthouse_data['x']) == len(t_ref):
        plt.plot(t_ref, lighthouse_data['x'], ':', label='x lighthouse')
    plt.ylabel('X (m)')
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t_ref, kalman_data['y'], label='y estimé')
    plt.plot(t_ref, y_ref, '--', label='y souhaité')
    if len(lighthouse_data['y']) == len(t_ref):
        plt.plot(t_ref, lighthouse_data['y'], ':', label='y lighthouse')
    plt.ylabel('Y (m)')
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t_ref, kalman_data['z'], label='z estimé')
    plt.plot(t_ref, z_ref, '--', label='z souhaité')
    if len(lighthouse_data['z']) == len(t_ref):
        plt.plot(t_ref, lighthouse_data['z'], ':', label='z lighthouse')
    plt.xlabel('Temps (s)')
    plt.ylabel('Z (m)')
    plt.grid()
    plt.legend()

    plt.suptitle('Comparaison Kalman vs Lighthouse vs Trajectoire Souhaitée')
    plt.tight_layout()
    plt.show()
