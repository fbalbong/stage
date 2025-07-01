import logging
import time
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
import pandas as pd

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
logging.basicConfig(level=logging.ERROR)

motor_data = {'m1': [], 'm2': [], 'm3': [], 'm4': []}
stab_data  = {'thrust': [], 'roll': [], 'pitch': [], 'yaw': []}
time_data  = []

def motor_callback(ts, data, logconf):
    motor_data['m1'].append(data['motor.m1'])
    motor_data['m2'].append(data['motor.m2'])
    motor_data['m3'].append(data['motor.m3'])
    motor_data['m4'].append(data['motor.m4'])
    # Timestamp SOLO si quieres un tiempo por cada log de motor.
    time_data.append(time.time() - t0)

def stab_callback(ts, data, logconf):
    stab_data['thrust'].append(data['stabilizer.thrust'])
    stab_data['roll'].append(data['stabilizer.roll'])
    stab_data['pitch'].append(data['stabilizer.pitch'])
    stab_data['yaw'].append(data['stabilizer.yaw'])
    # Opcional: podrías poner aquí también el time_data.append(), pero te saldría el doble de filas si activas ambos logs.
    # Si quieres UN SOLO CSV, mantén el append solo en motor_callback.

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Armado y desbloqueo de protección
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)
        print("Desbloqueando protección de thrust...")
        for _ in range(10):
            scf.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.05)

        # Configuración de logs
        logconf_mot = LogConfig('Motors', period_in_ms=20)
        for v in ['motor.m1','motor.m2','motor.m3','motor.m4']:
            logconf_mot.add_variable(v, 'float')
        scf.cf.log.add_config(logconf_mot)
        logconf_mot.data_received_cb.add_callback(motor_callback)
        logconf_mot.start()

        logconf_stab = LogConfig('Stabilizer', period_in_ms=20)
        for v in ['stabilizer.thrust','stabilizer.roll','stabilizer.pitch','stabilizer.yaw']:
            logconf_stab.add_variable(v, 'float')
        scf.cf.log.add_config(logconf_stab)
        logconf_stab.data_received_cb.add_callback(stab_callback)
        logconf_stab.start()

        global t0
        t0 = time.time()

        # Vuelo secuencial
        thrust_lift = 57000
        thrust_hover = 39000
        thrust_mouv = 42000
        thrust_land1 = 36000
        thrust_land2 = 28000
        dt = 0.05

        print("Despegue...")
        for _ in range(int(0.3 / dt)):
            scf.cf.commander.send_setpoint(0, 0, 0, thrust_lift)
            time.sleep(dt)
        print("Mantener hover...")
        for _ in range(int(1.0 / dt)):
            scf.cf.commander.send_setpoint(0, 0, 0, thrust_hover)
            time.sleep(dt)
        print("Roll 5º prolongado...")
        for _ in range(int(1.2 / dt)):
            scf.cf.commander.send_setpoint(5, 0, 0, thrust_mouv)
            time.sleep(dt)
        print("Volver a hover...")
        for _ in range(int(0.5 / dt)):
            scf.cf.commander.send_setpoint(0, 0, 0, thrust_hover)
            time.sleep(dt)
        print("Pitch 5º prolongado...")
        for _ in range(int(1.2 / dt)):
            scf.cf.commander.send_setpoint(0, 5, 0, thrust_mouv)
            time.sleep(dt)
        print("Volver a hover...")
        for _ in range(int(0.5 / dt)):
            scf.cf.commander.send_setpoint(0, 0, 0, thrust_hover)
            time.sleep(dt)
        print("Aterrizaje fase 1...")
        for _ in range(int(0.7 / dt)):
            scf.cf.commander.send_setpoint(0, 0, 0, thrust_land1)
            time.sleep(dt)
        print("Aterrizaje fase 2...")
        for _ in range(int(0.7 / dt)):
            scf.cf.commander.send_setpoint(0, 0, 0, thrust_land2)
            time.sleep(dt)
        print("Motores off.")
        for _ in range(20):
            scf.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(dt)

        # Parar logs
        logconf_mot.stop()
        logconf_stab.stop()

    # --- Emparejar el número de muestras ---
    min_len = min(len(time_data), len(motor_data['m1']), len(stab_data['thrust']))
    df = pd.DataFrame({
        'time': time_data[:min_len],
        'motor.m1': motor_data['m1'][:min_len],
        'motor.m2': motor_data['m2'][:min_len],
        'motor.m3': motor_data['m3'][:min_len],
        'motor.m4': motor_data['m4'][:min_len],
        'stabilizer.thrust': stab_data['thrust'][:min_len],
        'stabilizer.roll':   stab_data['roll'][:min_len],
        'stabilizer.pitch':  stab_data['pitch'][:min_len],
        'stabilizer.yaw':    stab_data['yaw'][:min_len],
    })
    df.to_csv("vuelo_datos_thrust.csv", index=False)
    print("Donnés exportés à vuelo_datos_thrust.csv")
