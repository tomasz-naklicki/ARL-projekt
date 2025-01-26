import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

logging.basicConfig(level=logging.ERROR)
# URI to the Crazyflie to connect to
uri = "radio://0/80/250K/E7E7E7E7E7"
previous_lux = None
direction = "R"
stop = False


def log_stab_callback(timestamp, data, logconf):
    global previous_lux
    global direction
    global stop
    print(f"[{timestamp}] Lux: {data['bh1750.lux']:.2f} lx")
    current_lux = data["bh1750.lux"]

    if current_lux > 500.0:
        stop = True
        return

    if previous_lux is not None:
        if current_lux < previous_lux:
            direction = "L" if direction == "R" else "R"
            print(
                f"ZMIEŃ KIERUNEK OBROTU NA PRAWO"
                if direction == "R"
                else "ZMIEŃ KIERUNEK OBROTU NA LEWO"
            )
    previous_lux = current_lux


def simple_log_async(scf, logconf):
    global stop
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    while True:
        if stop:
            break
        time.sleep(0.5)
    logconf.stop()


def main():
    # Initialize the low-level drivers
    global direction
    cflib.crtp.init_drivers()

    try:
        lg_stab = LogConfig(name="BH1750", period_in_ms=500)
        lg_stab.add_variable("bh1750.lux", "float")

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            with MotionCommander(scf) as mc:
                simple_log_async(scf, lg_stab)
                mc.start_forward(0.1)
                while not stop:
                    if direction == "R":
                        mc.turn_right(10)
                    else:
                        mc.turn_left(10)
                mc.stop()
                mc.land()

    except KeyboardInterrupt:
        mc.stop()
        mc.land()


if __name__ == "__main__":
    main()
