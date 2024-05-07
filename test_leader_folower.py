import time
import math

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger



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
    if scf == swarm[0]:  # Seul le premier drone exécute la séquence de vol
        take_off(scf)
        land(scf)



def follow_leader(scf, leader_position):
    # Définir la distance désirée par rapport au leader
    desired_distance = 0.5  # 50 cm
    
    if scf != swarm[0]:
        # Récupérer la position actuelle du drone
        current_position = scf.cf.position()
        
        # Calculer le vecteur directionnel du drone vers le leader
        dx = leader_position[0] - current_position[0]
        dy = leader_position[1] - current_position[1]
        dz = leader_position[2] - current_position[2]
        
        # Calculer la distance entre le drone et le leader
        distance_to_leader = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Si la distance est supérieure à la distance désirée, ajuster la position cible
        # Normaliser le vecteur directionnel
        dx /= distance_to_leader
        dy /= distance_to_leader
        dz /= distance_to_leader
        
        # Multiplier le vecteur directionnel par la distance désirée pour obtenir la position cible
        target_x = leader_position[0] - dx * desired_distance
        target_y = leader_position[1] - dy * desired_distance
        target_z = leader_position[2] - dz * desired_distance
        
        # Déplacer le drone vers la position cible
        scf.cf.high_level_commander.go_to(target_x, target_y, target_z)
            
        



leader_pos=[0,0,0]

uris = {
    'radio://0/20/2M/E7E7E7E701',
    'radio://0/20/2M/E7E7E7E702',
    'radio://0/20/2M/E7E7E7E703',
    'radio://0/20/2M/E7E7E7E704',
    # Add more URIs if you want more copters in the swarm
}

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print('Connected to Crazyflies')
        swarm.parallel_safe(light_check)
        swarm.reset_estimators()

        while True:
            leader_pos = swarm[0].cf.position()
            # Exécute la séquence de vol uniquement sur le premier drone
            swarm[0].parallel_safe(hover_sequence)
            

            
            # Reçoit la position du leader (premier drone) et fait suivre aux autres drones
            for scf in swarm:
                scf.parallel_safe(follow_leader, leader_pos)
