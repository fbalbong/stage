import time
import cflib.crtp
import matplotlib.pyplot as plt
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# URI del Crazyflie
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Inicializa radio
cflib.crtp.init_drivers()

# Variables globales
log_data = {
    'timestamp': [],
    'x': [],
    'y': [],
    'z': [],
    'vx': [],
    'vy': [],
    'vz': []
}

# Callback para guardar datos del estimador de Kalman
def kalman_log_callback(timestamp, data, logconf):
    log_data['timestamp'].append(timestamp / 1000.0)  # en segundos
    log_data['x'].append(data['stateEstimate.x'])
    log_data['y'].append(data['stateEstimate.y'])
    log_data['z'].append(data['stateEstimate.z'])
    log_data['vx'].append(data['stateEstimate.vx'])
    log_data['vy'].append(data['stateEstimate.vy'])
    log_data['vz'].append(data['stateEstimate.vz'])

# Configuración del log
logconf = LogConfig(name='Kalman', period_in_ms=100)
logconf.add_variable('stateEstimate.x', 'float')
logconf.add_variable('stateEstimate.y', 'float')
logconf.add_variable('stateEstimate.z', 'float')
logconf.add_variable('stateEstimate.vx', 'float')
logconf.add_variable('stateEstimate.vy', 'float')
logconf.add_variable('stateEstimate.vz', 'float')

# Script principal
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    print("Conectado. Forzando Kalman...")

    # Selección del estimador y reinicio
    scf.cf.param.set_value('stabilizer.estimator', '2')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.1)

    # Configuración del log
    scf.cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(kalman_log_callback)
    logconf.start()

    # Espera 10 segundos registrando datos
    print("Recolectando estimaciones del filtro de Kalman...")

    time.sleep(10)

    logconf.stop()

# ----------------------
# Gráficas después del vuelo
# ----------------------

plt.figure()
plt.plot(log_data['timestamp'], log_data['x'], label='X')
plt.plot(log_data['timestamp'], log_data['y'], label='Y')
plt.plot(log_data['timestamp'], log_data['z'], label='Z')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición (m)')
plt.title('Estimación de posición - Kalman Filter')
plt.legend()
plt.grid()
plt.show()
