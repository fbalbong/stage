from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

def list_log_vars():
    from cflib.crtp import init_drivers
    init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        log_vars = scf.cf.log.get_available_log_vars()
        for name, var in sorted(log_vars.items()):
            print(name)

list_log_vars()
