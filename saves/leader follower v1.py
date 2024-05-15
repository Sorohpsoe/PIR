import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
import threading


uris = [
    'radio://0/80/2M/9',
    'radio://0/80/2M/A',
]

seq_args = {
    uris[0]: [0],
    uris[1]: [4],
}

leader_pos=(0.75,0,0.2)

def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)

def land(scf):
    commander= scf.cf.high_level_commander

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()

def store_position(pc):
    global leader_pos
    while True:
        pos = swarm.get_estimated_positions()

        leader_pos=(pos[uris[0]].x, pos[uris[0]].y, pos[uris[0]].z)
        print(leader_pos)
        time.sleep(0.1)

def simple_sequence(scf: SyncCrazyflie, sequence):
        global leader_pos
        global en_cours
        en_cours = True
        cf = scf.cf
        with PositionHlCommander(cf, default_velocity=0.2, controller=PositionHlCommander.CONTROLLER_PID) as pc:
            if sequence == 0:   
                for i in range(3):
                    pc.go_to(-0.5,-0.5,i/5.0+0.5)

                    pc.go_to(1.0,-0.5,i/5.0+0.5)

                    pc.go_to(1.0,0.5,i/5.0+0.5)

                    pc.go_to(-0.5,0.5,i/5.0+0.5)
                en_cours = False

            else:
                    while en_cours:
                        pc.go_to(leader_pos[0]+sequence/10, leader_pos[1]+sequence/10, leader_pos[2])  
                        time.sleep(0.1)
                    



if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print('Connected to  Crazyflies')

        swarm.reset_estimators()
        threading.Thread(target=store_position, args=(swarm,), daemon=True).start()   
        swarm.parallel_safe(simple_sequence, args_dict=seq_args)
