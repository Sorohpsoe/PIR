import time
import numpy as np
import math
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
import utils.interactions as interactions
import utils.maths as maths
import csv
import os

#Création des dictionnaires stockant toutes les positions et vélocités (pour l'analyse de données)
pos_dict_list = {}
vel_dict_list = {}

#Création des dictionnaires stockant uniquement la dernière position et vélocité (pour le flocking)
pos_dict = {}
vel_dict = {}

#Liste des addresses des drônes utilisés pour la séquence
uris = [
    'radio://0/80/2M/3',
    'radio://0/80/2M/A',
    'radio://0/80/2M/7',
#    'radio://0/80/2M/5',
]

#ID du drône leader (tous les autres sont followers)
URI_Leader = 'radio://0/80/2M/3'

#Création et initialisation des dictionnaires stockant uniquement la dernière position et vélocité
for id in uris:
    pos_dict_list[id] = []
    vel_dict_list[id] = []


#Fonctions permettant d'effectuer un lightcheck des drones
def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(0.5)


#Fonction collectant les données de position et de vitesse
def log_callback(uri, timestamp, data, logconf):
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    pos = np.array([x, y, z])
    pos_dict[uri] = pos

    vx = data['stateEstimate.vx']
    vy = data['stateEstimate.vy']
    vz = data['stateEstimate.vz']
    vel = np.array([vx, vy, vz])
    vel_dict[uri] = vel

    pos_dict_list[uri].append(pos)
    vel_dict_list[uri].append(vel)
    
    #print('uri: {} pos: ({}, {}, {})'.format(uri, x, y, z))


#Fonction exportant les données de positions dans un fichier csv pour le traitement des données
def save_to_csv(dictionnaire, filename):
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['drone_id', 'x', 'y', 'z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for drone_id, positions in dictionnaire.items():
            for i, pos in enumerate(positions):
                writer.writerow({'drone_id': drone_id, 'x': pos[0], 'y': pos[1], 'z': pos[2]})



#Fonction initialisant un log
def start_states_log(scf):
    log_conf = LogConfig(name='States', period_in_ms=10)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')

    log_conf.add_variable('stateEstimate.vx', 'float')
    log_conf.add_variable('stateEstimate.vy', 'float')
    log_conf.add_variable('stateEstimate.vz', 'float')
    uri = scf.cf.link_uri
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda timestamp, data, logconf: log_callback(uri, timestamp, data, logconf))
    log_conf.start()


#Fonction définissant une zone de laquelle le drone ne peut pas sortir
def is_in_box_limit(box_limits, positions, whichCF, v_0):
    x = positions[whichCF][0]
    y = positions[whichCF][1]
    z = positions[whichCF][2]

    back_inside_dir = [0, 0, 0]

    if abs(x) >= box_limits[0]:
        back_inside_dir[0] = -math.copysign(v_0, x)
        # print("x limit reached")    
    if abs(y) >= box_limits[1]:
        back_inside_dir[1] = -math.copysign(v_0, y)
        # print("y limit reached")
    if z >= box_limits[2]:
        back_inside_dir[2] = -v_0
        # print("z+ limit reached")
    if z < 0.2:
        back_inside_dir[2] = v_0/2
        # print("z- limit")
    
    return back_inside_dir


#Fonction de décollage du drone
def take_off(cf, height):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = height / take_off_time
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


#Fonction d'atterrissage du drone
def land(cf, position):
    print('landing...')
    landing_time = 4.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time
    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)


#Fonction définissant la séquence de vol
def fly_sequence(scf):
    try:
        cf = scf.cf
        global en_cours
        en_cours = True

        #box_limits = [-0.75, 1.15, 2.0]
        v_0 = 0.3

        #Séquence du leader
        if scf.cf.link_uri == URI_Leader:   
            with PositionHlCommander(cf, default_velocity=0.2, controller=PositionHlCommander.CONTROLLER_PID) as pc:

                for i in [0,0.5,1]:
                    pc.go_to(-0.5,-0.5,0.5+i)

                    pc.go_to(1.0,-0.5,0.5+i)

                    pc.go_to(1.0,0.5,0.5+i)

                    pc.go_to(-0.5,0.5,0.5+i)

                pc.go_to(0.0,0.0,0.5)
                pc.go_to(1.5,0.0,0.5)
                pc.go_to(0.0,0.0,0.5)
                

                """for i in range(1):
                    pc.go_to(-0.5,-0.5,0.5+i)
                
                    pc.go_to(1.0,-0.5,0.5+i)

                    pc.go_to(1.0,0.5,0.5+i)

                    pc.go_to(-0.5,0.5,0.5+i)"""
                
                print(pos_dict_list)
                print(vel_dict_list)

                save_to_csv(pos_dict_list, "positions.csv")
                save_to_csv(vel_dict_list, "velocites.csv")

                en_cours = False

        #Séquence des followers
        else :
            take_off(cf, 0.5)
            while en_cours:

                """going_back = is_in_box_limit(box_limits, pos_dict, scf.cf.link_uri, v_0)

                if all(el == 0 for el in going_back):"""

                attraction = interactions.compute_attraction_force(pos_dict, scf.cf.link_uri, 0.35, 0.3, False)
                repulsion = interactions.compute_repulsion_force(pos_dict, scf.cf.link_uri, 0.40, 0.6, False)
                self_propulsion = interactions.compute_self_propulsion(vel_dict, scf.cf.link_uri, v_0)
                alignement = interactions.compute_friction_alignment(pos_dict, vel_dict, 0.8, 0.2, 1, 5, 0.6, scf.cf.link_uri)

                force = attraction + repulsion + alignement

                cf.commander.send_velocity_world_setpoint(force[0], force[1], force[2], 0)
                time.sleep(0.1)


                """else:
                    cf.commander.send_velocity_world_setpoint(going_back[0], going_back[1], going_back[2], 0)
                    time.sleep(0.1)"""


            land(cf, pos_dict[scf.cf.link_uri])

    except Exception as e:
        print(e)


#Lancement du code : connection, lightcheck, réinitialisation, lancement du log et vol
if __name__ == '__main__':

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:

        print('Connected to  Crazyflies')

        print('Performing light check')
        swarm.parallel_safe(light_check)
        print('Light check done')

        print('Reseting estimators')
        swarm.reset_estimators()
        print('Estimators have been reset')

        swarm.parallel_safe(start_states_log)
        print('Logging states info...')

        print('Lets fly ! Put your protection glasses on')
        swarm.parallel_safe(fly_sequence)
