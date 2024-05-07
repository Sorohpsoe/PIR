import time
import math
from threading import Thread
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger

from classes_boid import Boid



def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)

def take_off(scf):
    commander = scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)

def land(scf):
    commander = scf.cf.high_level_commander

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()

def hover_sequence(scf):
    if scf == swarm[0]:
        take_off(scf)
        land(scf)
        
def flocking(scf,sequence):
    objet=sequence[scf.cf.link_uri]
    print(f"objet:{objet}")
    


def store_pos(scf, dict_pos):
    while True:
        dict_pos[] = swarm.get_estimated_positions()
        print(f"dico : {dict_pos}")
        time.sleep(0.2)
               
seq_args = {}
dict_pos = {}


uris = {
    'radio://0/80/2M/3',
    'radio://0/80/2M/6',
    # Add more URIs if you want more copters in the swarm
}

for i in range(len(uris)):
    seq_args[uris[i]] = Boid(0, 0)


print(seq_args)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print('Connected to Crazyflies')
        swarm.parallel_safe(light_check)
        swarm.reset_estimators()
        swarm.parallel_safe(take_off)
        time.sleep(3)
        Thread(target=store_pos,args=swarm).start()
        swarm.parallel_safe(flocking,args_dict=seq_args)
        time.sleep(3)
        swarm.parallel_safe(land)
        
        
        