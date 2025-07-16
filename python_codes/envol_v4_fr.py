"""
Script de journalisation des données de vol du Crazyflie
Commentaires en français pour clarifier le rôle de chaque section.
"""
import time
import numpy as np
import pandas as pd
import os

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# URI de connexion (environnement ou défaut)
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# === Initialisation des structures de données ===
kalman_data = {
    'time': [], 'x': [], 'y': [], 'z': [],
    'roll': [], 'pitch': [], 'yaw': [],
    'F': [], 'R': [], 'Z': [],
    'B': [], 'C': [], 'X': [],
    'S': [], 'T': [], 'Y': []
}

lighthouse_data = {'x': [], 'y': [], 'z': []}
start_time = None
flowdeck_active = [False]
multiranger_active = [False]
lighthouse_status = [0]
motor_data = {'m1': [], 'm2': [], 'm3': [], 'm4': []}
motor_data_rpm = {'m1_rpm': [], 'm2_rpm': [], 'm3_rpm': [], 'm4_rpm': []}
stab_data = {'thrust': [], 'roll': [], 'pitch': [], 'yaw': []}

# === Callbacks ===
def kalman_callback(timestamp, data, logconf):
    """Capture des états principaux du Kalman (position + orientation)"""
    global start_time
    if start_time is None:
        start_time = timestamp
    t = (timestamp - start_time) / 1000.0
    kalman_data['time'].append(t)
    for var in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        key = f'stateEstimate.{var}' if var in ['x','y','z'] else f'stateEstimate.{var}'
        kalman_data[var].append(data[key])

# === Callbacks Étendus Kalman divisés ===

def kalman_frz_callback(timestamp, data, logconf):
    """Capture des états F, R, Z"""
    kalman_data['F'].append(data.get('kalman.stateF', np.nan))
    kalman_data['R'].append(data.get('kalman.stateR', np.nan))
    kalman_data['Z'].append(data.get('kalman.stateZ', np.nan))

 def kalman_bcx_callback(timestamp, data, logconf):
    """Capture des états B, C, X"""
    kalman_data['B'].append(data.get('kalman.stateB', np.nan))
    kalman_data['C'].append(data.get('kalman.stateC', np.nan))
    kalman_data['X'].append(data.get('kalman.stateX', np.nan))

 def kalman_sty_callback(timestamp, data, logconf):
    """Capture des états S, T, Y"""
    kalman_data['S'].append(data.get('kalman.stateS', np.nan))
    kalman_data['T'].append(data.get('kalman.stateT', np.nan))
    kalman_data['Y'].append(data.get('kalman.stateY', np.nan))

# Callback Lighthouse
def lighthouse_callback(timestamp, data, logconf):
    lighthouse_data['x'].append(data['lighthouse.x'])
    lighthouse_data['y'].append(data['lighthouse.y'])
    lighthouse_data['z'].append(data['lighthouse.z'])

def lighthouse_status_callback(timestamp, data, logconf):
    lighthouse_status[0] = data['lighthouse.status']

# Callback moteurs et RPM
def motor_callback(ts, data, logconf):
    for i in range(1,5): motor_data[f'm{i}'].append(data[f'motor.m{i}'])

def motor_rpm_callback(ts, data, logconf):
    for i in range(1,5): motor_data_rpm[f'm{i}_rpm'].append(data[f'rpm.m{i}'])

# Callback contrôleur stabilisation
def stab_callback(ts, data, logconf):
    stab_data['thrust'].append(data['controller.cmd_thrust'])
    stab_data['roll'].append(data['controller.cmd_roll'])
    stab_data['pitch'].append(data['controller.cmd_pitch'])
    stab_data['yaw'].append(data['controller.cmd_yaw'])

# Callback flowdeck simple
def range_callback(timestamp, data, logconf):
    z = data['range.zrange']
    if 0.05 < z < 3.0:
        flowdeck_active[0] = True

# Callback batterie
def battery_callback(timestamp, data, logconf):
    voltage = data['pm.vbat']
    print(f"🔋 Batterie: {voltage:.2f} V")

# === Fonctions utilitaires de vol ===
def send_position_limited_speed(cf, start_pos, end_pos, max_speed=0.3, dt=0.1):
    """Déplace le drone entre deux points à vitesse limitée"""
    start = np.array(start_pos)
    end = np.array(end_pos)
    vec = end - start
    dist = np.linalg.norm(vec)
    if dist == 0: return
    direction = vec / dist
    steps = int(dist / (max_speed * dt))
    for i in range(steps + 1):
        pos = start + direction * max_speed * dt * i
        cf.commander.send_position_setpoint(pos[0], pos[1], pos[2], 0)
        time.sleep(dt)

def arm_drone():
    """Procédure d'armement du Crazyflie"""
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.platform.send_arming_request(True)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

def check_decks_attached(cf):
    return {
        'flow2': int(cf.param.get_value('deck.bcFlow2')),
        'multiranger': int(cf.param.get_value('deck.bcMultiranger'))
    }

# === MAIN ===
if __name__ == '__main__':
    # Lecture des paramètres (en français)
    use_correction = int(input("Utiliser lighthouse.useCorrection ? (1=Oui, 0=Non): "))
    lh_method     = int(input("Quel méthode lighthouse utiliser ? (0=Crossing,1=Sweep): "))
    use_fandr     = int(input("Utiliser kalman.useFAndR ? (1=Oui,0=Non): "))
    use_bcsyt     = int(input("Utiliser kalman.useBAndCAndSAndT ? (1=Oui,0=Non): "))

    # Initialisation
    cflib.crtp.init_drivers()
    arm_drone()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Configuration initiale
        scf.cf.param.set_value('stabilizer.estimator', '2')
        scf.cf.param.set_value('lighthouse.useCorrection', str(use_correction))
        scf.cf.param.set_value('lighthouse.method', str(lh_method))
        scf.cf.param.set_value('kalman.useFAndR', str(use_fandr))
        scf.cf.param.set_value('kalman.useBAndCAndSAndT', str(use_bcsyt))
        reset_estimator(scf.cf)

        # Vérification des decks
        decks = check_decks_attached(scf.cf)
        flowdeck_active[0]     = decks['flow2'] != 0
        multiranger_active[0]  = decks['multiranger'] != 0

        # --- Configuration des logs ---
        # Batterie
        logconf_bat = LogConfig('Battery', 500)
        logconf_bat.add_variable('pm.vbat','float')
        scf.cf.log.add_config(logconf_bat)
        logconf_bat.data_received_cb.add_callback(battery_callback)
        logconf_bat.start(); time.sleep(1); logconf_bat.stop()

        # Kalman principal
        logconf_kalman = LogConfig('Kalman', 20)
        for var in ['stateEstimate.x','stateEstimate.y','stateEstimate.z',
                    'stateEstimate.roll','stateEstimate.pitch','stateEstimate.yaw']:
            logconf_kalman.add_variable(var,'float')
        scf.cf.log.add_config(logconf_kalman)
        logconf_kalman.data_received_cb.add_callback(kalman_callback)
        logconf_kalman.start()

        # Kalman étendu
        logconf_kf_ext = LogConfig('KalmanExt', 20)
        for state in ['stateF','stateR','stateZ','stateB','stateC','stateX','stateS','stateT','stateY']:
            try: logconf_kf_ext.add_variable(f'kalman.{state}','float')
            except: pass
        if logconf_kf_ext.variables:
            scf.cf.log.add_config(logconf_kf_ext)
            logconf_kf_ext.data_received_cb.add_callback(kalman_states_callback)
            logconf_kf_ext.start()

        # Lighthouse
        ...  # reste identique pour Lighthouse, Motors, RPM, Controller, Range

        # Trajectoire
        ...  # inchangé

        # Arrêt des logs
        logconf_kalman.stop()
        if logconf_kf_ext.variables: logconf_kf_ext.stop()
        ...  # arrêts restants

    # Diagnostic
    print("✅ Lighthouse status =", lighthouse_status[0])
    print("✅ Flowdeck:" , flowdeck_active[0]," Multi-ranger:", multiranger_active[0])

    # Synchronisation et export CSV
    min_len = min(len(kalman_data['time']), len(lighthouse_data['x']),
                  len(motor_data['m1']), len(motor_data_rpm['m1_rpm']), len(stab_data['thrust']))
    def trim(d):
        for k in d: d[k] = d[k][:min_len]
    for d in [kalman_data, lighthouse_data, motor_data, motor_data_rpm, stab_data]: trim(d)

    df = pd.DataFrame({
        'time': kalman_data['time'],
        'x_kalman': kalman_data['x'], 'y_kalman': kalman_data['y'], 'z_kalman': kalman_data['z'],
        'x_lh': lighthouse_data['x'], 'y_lh': lighthouse_data['y'], 'z_lh': lighthouse_data['z'],
        'roll': kalman_data['roll'], 'pitch': kalman_data['pitch'], 'yaw': kalman_data['yaw'],
        **{key: kalman_data[key] for key in ['F','R','Z','B','C','X','S','T','Y']},
        **motor_data, **motor_data_rpm,
        'thrust': stab_data['thrust'], 'cmd_roll': stab_data['roll'],
        'cmd_pitch': stab_data['pitch'], 'cmd_yaw': stab_data['yaw']
    })
    # Génération nom fichier
    i = 1
    while os.path.exists(f"vuelo_datos{i}.csv"): i+=1
    filename = f"vuelo_datos{i}.csv"
    df.to_csv(filename, index=False)
    print(f"✅ Données exportées : {filename} (échantillons: {min_len})")
