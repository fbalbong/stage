import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

kalman_data = {
    'time': [], 'x': [], 'y': [], 'z': [],
    'roll': [], 'pitch': [], 'yaw': [],
    'F' : [],  'R' : [], 'Z' : [],
}

lighthouse_data = {'x': [], 'y': [], 'z': []}
start_time = None
flowdeck_active = [False]
multiranger_active = [False]
lighthouse_status = [0]
motor_data = {'m1': [], 'm2': [], 'm3': [], 'm4': []}
motor_data_rpm = {'m1_rpm': [], 'm2_rpm': [], 'm3_rpm': [], 'm4_rpm': []}
stab_data  = {'thrust': [], 'roll': [], 'pitch': [], 'yaw': []}

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

def kalman_extra_callback(timestamp, data, logconf):
    # Añade los valores si existen, sino NaN
    kalman_data['F'].append(data.get('kalman.stateF', np.nan))
    kalman_data['R'].append(data.get('kalman.stateR', np.nan))
    kalman_data['Z'].append(data.get('kalman.stateZ', np.nan))

def lighthouse_callback(timestamp, data, logconf):
    lighthouse_data['x'].append(data['lighthouse.x'])
    lighthouse_data['y'].append(data['lighthouse.y'])
    lighthouse_data['z'].append(data['lighthouse.z'])

def lighthouse_status_callback(timestamp, data, logconf):
    lighthouse_status[0] = data['lighthouse.status']

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

def stab_callback(ts, data, logconf):
    stab_data['thrust'].append(data['controller.cmd_thrust'])
    stab_data['roll'].append(data['controller.cmd_roll'])
    stab_data['pitch'].append(data['controller.cmd_pitch'])
    stab_data['yaw'].append(data['controller.cmd_yaw'])

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
    """   use_correction = input("Vous voulez utiliser lighthouse.useCorrection? (1 = Oui, 0 = Non): ")
    while use_correction not in ['0', '1']:
        use_correction = input("Entrée non validée. Écrivez 1 (Oui) o 0 (Non): ")
    use_correction = int(use_correction)
    """
    lh_method = input("¿Quel method de lighthouse vous voulez utiliser? (0 = Crossing Beams, 1 = Sweep Angle): ")
    while lh_method not in ['0', '1']:
        lh_method = input("Entrée non validée. Écrivez 0 (Crossing Beams) o 1 (Sweep Angle): ")
    lh_method = int(lh_method)

    use_correction = input("Vous voulez utiliser kalman.useFAndR? (1 = Oui, 0 = Non): ")
    while use_correction not in ['0', '1']:
        use_correction = input("Entrée non validée. Écrivez 1 (Oui) o 0 (Non): ")
    use_correction = int(use_correction)

    cflib.crtp.init_drivers()
    arm_dron()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.set_value('stabilizer.estimator', '2')
        #scf.cf.param.set_value('lighthouse.useCorrection', str(use_correction))
        scf.cf.param.set_value('kalman.useFAndR', str(use_correction))
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

        # Kalman principal: SOLO estimados principales (6 variables)
        logconf_kalman = LogConfig(name='Kalman', period_in_ms=20)
        for var in [
            'stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
            'stateEstimate.roll', 'stateEstimate.pitch', 'stateEstimate.yaw'
        ]:
            logconf_kalman.add_variable(var, 'float')
        scf.cf.log.add_config(logconf_kalman)
        logconf_kalman.data_received_cb.add_callback(kalman_callback)
        logconf_kalman.start()

        # Kalman extra: SOLO F/R/Z (intenta añadirlas y arranca si alguna existe)
        logconf_kf_extra = LogConfig(name='KalmanF', period_in_ms=20)
        for var in ['kalman.stateF', 'kalman.stateR', 'kalman.stateZ']:
            try:
                logconf_kf_extra.add_variable(var, 'float')
            except Exception:
                pass
        if len(logconf_kf_extra.variables) > 0:
            scf.cf.log.add_config(logconf_kf_extra)
            logconf_kf_extra.data_received_cb.add_callback(kalman_extra_callback)
            logconf_kf_extra.start()
        
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

        # --- Log Motors ---
        lc_mot = LogConfig('Motors', period_in_ms=20)
        for v in ['motor.m1','motor.m2','motor.m3','motor.m4']:
            lc_mot.add_variable(v,'float')
        scf.cf.log.add_config(lc_mot)
        lc_mot.data_received_cb.add_callback(motor_callback)
        lc_mot.start()

        # --- Log Motors_rpm ---
        lc_mot_rpm = LogConfig('Motors_rpm', period_in_ms=20)
        for v in ['rpm.m1','rpm.m2','rpm.m3','rpm.m4']:
            lc_mot_rpm.add_variable(v,'uint16_t')
        scf.cf.log.add_config(lc_mot_rpm)
        lc_mot_rpm.data_received_cb.add_callback(motor_rpm_callback)
        lc_mot_rpm.start()

        # --- Log Controller ---
        lc_st = LogConfig('Controller', period_in_ms=20)
        for v in ['controller.cmd_thrust','controller.cmd_roll','controller.cmd_pitch','controller.cmd_yaw']:
            lc_st.add_variable(v,'float')
        scf.cf.log.add_config(lc_st)
        lc_st.data_received_cb.add_callback(stab_callback)
        lc_st.start()

        # Trajectoire souhaitée
        trajectory = [
            (0, 0, 0.5, 3),
            (1.5, 0, 0.5, 4),
            (1.5, 1, 0.5, 4),
            (0, 1, 0.5, 4),
            (0, -0.5, 0.5, 4),
            (0, -0.5, 0, 1)
        ]

        # Ejecutar vuelo
        for x, y, z, duration in trajectory:
            send_position(scf.cf, x, y, z, duration)

        logconf_kalman.stop()
        if 'logconf_kf_extra' in locals():
            try:
                logconf_kf_extra.stop()
            except Exception:
                pass
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

    # [Todo el código anterior permanece igual hasta la sección de exportación CSV...]

# =============================================
# CORRECCIÓN PARA EXPORTAR CSV CON LONGITUDES IGUALES
# =============================================

# Encontramos la longitud mínima común entre todos los datasets
min_length = min(
    len(kalman_data['time']),
    len(lighthouse_data['x']),
    len(motor_data['m1']),
    len(motor_data_rpm['m1_rpm']),
    len(stab_data['thrust'])
)

# Función para recortar todas las listas a la longitud mínima
def trim_data(data_dict, length):
    for key in data_dict:
        data_dict[key] = data_dict[key][:length]

# Aplicamos a todos los conjuntos de datos
trim_data(kalman_data, min_length)
trim_data(lighthouse_data, min_length)
trim_data(motor_data, min_length)
trim_data(motor_data_rpm, min_length)
trim_data(stab_data, min_length)

# Verificación final de longitudes (opcional)
print("\n=== Longitudes después del ajuste ===")
print(f"Kalman: {len(kalman_data['time'])}")
print(f"Lighthouse: {len(lighthouse_data['x'])}")
print(f"Motores: {len(motor_data['m1'])}")
print(f"RPM: {len(motor_data_rpm['m1_rpm'])}")
print(f"Controlador: {len(stab_data['thrust'])}")

# Generar nombre automático para el archivo de salida
def get_next_filename(base_name="vuelo_datos", ext="csv"):
    i = 1
    while os.path.exists(f"{base_name}{i}.{ext}"):
        i += 1
    return f"{base_name}{i}.{ext}"

output_filename = get_next_filename()


# Creación del DataFrame con datos sincronizados
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
    'F': kalman_data['F'],
    'R': kalman_data['R'],
    'Z': kalman_data['Z'],
    'motor.m1': motor_data['m1'],
    'motor.m2': motor_data['m2'],
    'motor.m3': motor_data['m3'],
    'motor.m4': motor_data['m4'],
    'rpm.m1': motor_data_rpm['m1_rpm'],
    'rpm.m2': motor_data_rpm['m2_rpm'],
    'rpm.m3': motor_data_rpm['m3_rpm'],
    'rpm.m4': motor_data_rpm['m4_rpm'],
    'controller.cmd_thrust': stab_data['thrust'],
    'controller.cmd_roll': stab_data['roll'],
    'controller.cmd_pitch': stab_data['pitch'],
    'controller.cmd_yaw': stab_data['yaw'],
})

# Exportar a CSV
df.to_csv(output_filename, index=False)
print(f"\n✅ Datos exportados correctamente a {output_filename} (muestras: {min_length})")
