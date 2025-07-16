# -*- coding: utf-8 -*-
"""
Script de journalisation des données de vol du Crazyflie
Commentaires en français pour clarifier le rôle de chaque section.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
from scipy.spatial.transform import Rotation as R

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

# === Callback Kalman principal (6 états) ===
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

# === Callback pour tous les états Kalman étendus (F, R, Z, B, C, X, S, T, Y) ===
def kalman_states_callback(timestamp, data, logconf):
    # On ajoute NaN si une variable manque
    kalman_data['F'].append(data.get('kalman.stateF', np.nan))
    kalman_data['R'].append(data.get('kalman.stateR', np.nan))
    kalman_data['Z'].append(data.get('kalman.stateZ', np.nan))
    kalman_data['B'].append(data.get('kalman.stateB', np.nan))
    kalman_data['C'].append(data.get('kalman.stateC', np.nan))
    kalman_data['X'].append(data.get('kalman.stateX', np.nan))
    kalman_data['S'].append(data.get('kalman.stateS', np.nan))
    kalman_data['T'].append(data.get('kalman.stateT', np.nan))
    kalman_data['Y'].append(data.get('kalman.stateY', np.nan))

# === Callback Lighthouse ===
def lighthouse_callback(timestamp, data, logconf):
    lighthouse_data['x'].append(data['lighthouse.x'])
    lighthouse_data['y'].append(data['lighthouse.y'])
    lighthouse_data['z'].append(data['lighthouse.z'])

def lighthouse_status_callback(timestamp, data, logconf):
    lighthouse_status[0] = data['lighthouse.status']

# === Callback moteurs et RPM ===
def motor_callback(ts, data, logconf):
    motor_data['m1'].append(data['motor.m1'])
    motor_data['m2'].append(data['motor.m2'])
    motor_data['m3'].append(data['motor.m3'])
    motor_data['m4'].append(data['motor.m4'])

def motor_rpm_callback(ts, data, logconf):
    motor_data_rpm['m1_rpm'].append(data['rpm.m1'])
    motor_data_rpm['m2_rpm'].append(data['rpm.m2'])
    motor_data_rpm['m3_rpm'].append(data['rpm.m3'])
    motor_data_rpm['m4_rpm'].append(data['rpm.m4'])

# === Callback contrôleur stabilisation ===
def stab_callback(ts, data, logconf):
    stab_data['thrust'].append(data['controller.cmd_thrust'])
    stab_data['roll'].append(data['controller.cmd_roll'])
    stab_data['pitch'].append(data['controller.cmd_pitch'])
    stab_data['yaw'].append(data['controller.cmd_yaw'])

# === Callback flowdeck simple (activation) ===
def range_callback(timestamp, data, logconf):
    z = data['range.zrange']
    if 0.05 < z < 3.0:
        flowdeck_active[0] = True

# === Callback batterie ===
def battery_callback(timestamp, data, logconf):
    voltage = data['pm.vbat']
    print(f"🔋 Baterie: {voltage:.2f} V")

# === Fonctions utilitaires de vol ===
def send_position_limited_speed(cf, start_pos, end_pos, max_speed=0.3, dt=0.1):
    """Déplace le drône entre deux points à vitesse limitée"""
    start = np.array(start_pos)
    end = np.array(end_pos)
    vec = end - start
    dist = np.linalg.norm(vec)
    if dist == 0:
        return
    direction = vec / dist
    steps = int(dist / (max_speed * dt))
    for i in range(steps + 1):
        pos = start + direction * max_speed * dt * i
        cf.commander.send_position_setpoint(pos[0], pos[1], pos[2], 0)
        time.sleep(dt)

# Fonctions d'armement et de reset
# ... (inchangées) ...

# === MAIN ===
if __name__ == '__main__':
    # Lecture des paramètres en français
    use_correction = input("Utiliser lighthouse.useCorrection ? (1=Oui, 0=Non): ")
    while use_correction not in ['0', '1']:
        use_correction = input("Entrée invalide. 1=Oui ou 0=Non: ")
    # Autres inputs similaires...

    # Initialisation Crazyflie
    cflib.crtp.init_drivers()
    arm_dron()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Configuration des estimateurs et paramètres
        scf.cf.param.set_value('stabilizer.estimator', '2')
        scf.cf.param.set_value('lighthouse.useCorrection', str(use_correction))
        # ...
        reset_estimator(scf.cf)

        # Vérification des decks
        decks = check_decks_attached(scf.cf)
        flowdeck_active[0] = decks['flow2'] != 0
        multiranger_active[0] = decks['multiranger'] != 0

        # --- Configuration des logs ---
        # Batterie
        logconf_bat = LogConfig(name='Battery', period_in_ms=500)
        logconf_bat.add_variable('pm.vbat', 'float')
        scf.cf.log.add_config(logconf_bat)
        logconf_bat.data_received_cb.add_callback(battery_callback)
        logconf_bat.start(); time.sleep(1); logconf_bat.stop()

        # Kalman principal
        logconf_kalman = LogConfig(name='Kalman', period_in_ms=20)
        for var in ['stateEstimate.x','stateEstimate.y','stateEstimate.z',
                    'stateEstimate.roll','stateEstimate.pitch','stateEstimate.yaw']:
            logconf_kalman.add_variable(var, 'float')
        scf.cf.log.add_config(logconf_kalman)
        logconf_kalman.data_received_cb.add_callback(kalman_callback)
        logconf_kalman.start()

        # Kalman étendu (F,R,Z puis B,C,X,S,T,Y)
        logconf_kf_ext = LogConfig(name='KalmanStates', period_in_ms=20)
        for var in ['kalman.stateF','kalman.stateR','kalman.stateZ',
                    'kalman.stateB','kalman.stateC','kalman.stateX',
                    'kalman.stateS','kalman.stateT','kalman.stateY']:
            try:
                logconf_kf_ext.add_variable(var, 'float')
            except Exception:
                pass
        if logconf_kf_ext.variables:
            scf.cf.log.add_config(logconf_kf_ext)
            logconf_kf_ext.data_received_cb.add_callback(kalman_states_callback)
            logconf_kf_ext.start()

        # Autres logs (Lighthouse, Motors, RPM, Controller, Range)
        # ... (inchangés) ...

        # Exécution de la trajectoire
        # ... (inchangés) ...

        # Arrêt des logs
        logconf_kalman.stop()
        if 'logconf_kf_ext' in locals():
            logconf_kf_ext.stop()
        # ... autres stop() ...

    # === Export CSV synchronisé ===
    # Calcul de la longueur minimale pour synchroniser toutes les séries
    min_length = min(
        len(kalman_data['time']),
        len(lighthouse_data['x']),
        len(motor_data['m1']),
        len(motor_data_rpm['m1_rpm']),
        len(stab_data['thrust'])
    )

    def trim_data(data_dict, length):
        """Rogne toutes les listes à la même longueur"""
        for k in data_dict:
            data_dict[k] = data_dict[k][:length]

    for d in [kalman_data, lighthouse_data, motor_data, motor_data_rpm, stab_data]:
        trim_data(d, min_length)

    # Construction du DataFrame et export CSV
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
        'yaw': kalman_data['yaw'],
        'F': kalman_data['F'], 'R': kalman_data['R'], 'Z': kalman_data['Z'],
        'B': kalman_data['B'], 'C': kalman_data['C'], 'X': kalman_data['X'],
        'S': kalman_data['S'], 'T': kalman_data['T'], 'Y': kalman_data['Y'],
        'm1': motor_data['m1'], 'm2': motor_data['m2'], 'm3': motor_data['m3'], 'm4': motor_data['m4'],
        'm1_rpm': motor_data_rpm['m1_rpm'], 'm2_rpm': motor_data_rpm['m2_rpm'],
        'm3_rpm': motor_data_rpm['m3_rpm'], 'm4_rpm': motor_data_rpm['m4_rpm'],
        'thrust': stab_data['thrust'], 'cmd_roll': stab_data['roll'],
        'cmd_pitch': stab_data['pitch'], 'cmd_yaw': stab_data['yaw'],
    })

    # Nom de fichier automatique
    def get_next_filename(base_name="vuelo_datos", ext="csv"):
        i = 1
        while os.path.exists(f"{base_name}{i}.{ext}"):
            i += 1
        return f"{base_name}{i}.{ext}"

    output_filename = get_next_filename()
    df.to_csv(output_filename, index=False)
    print(f"✅ Données exportées dans {output_filename} (échantillons: {min_length})")
