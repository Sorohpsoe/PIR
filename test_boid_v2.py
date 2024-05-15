import time
import numpy as np
import math
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
import utils.interactions as interactions
import utils.maths as maths
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import Commander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
import threading

from vec2 import vec2


pos_dict = {}
vel_dict = {}

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
    #print('uri: {} pos: ({}, {}, {})'.format(uri, x, y, z))


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



def start_distance_printing(scf):

    for i in pos_dict.keys():
        if i == scf.cf.link_uri: continue
        dist = maths.compute_distance(vel_dict[scf.cf.link_uri], vel_dict[i])

        print('Distance between URI {} and URI {} is {}'.format(i, scf.cf.link_uri, dist))
    print("\n")



uris = [
    'radio://0/80/2M/9',
    'radio://0/80/2M/2',
]

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

def take_off(cf, height):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = height / take_off_time
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


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
    
    
max_velocity = 0.2
max_acceleration = 0.03
    
def seek(self, positions, velocities, whichCF, target):
    
        my_pos=positions[whichCF]
        desired = target - my_pos
        desired = desired.normalized()
        desired *= max_velocity
        steer = desired - velocities[whichCF]
        steer = steer.limited(self.max_acceleration)
        return steer

def length(vect):
    return math.hypot(vect[0], vect[1], vect[2])

def normalized(vect):
    length = length(vect)
    if not length:
        length = 1.0
    return vect/length

# Separation
# Method checks for nearby boids and steers away
def separate(positions, whichCF, r_rep, p_rep):
    desired_separation = r_rep
    count = 0
    
    repulsion_force = np.zeros(3)

    # For every boid in the system, check if it's too close
    for i in positions.keys():
        if i == whichCF:
            continue
        
        
        neighbor_pos_diff = positions[i] - positions[whichCF]
        diff=normalized(neighbor_pos_diff)
        d = length(neighbor_pos_diff)

        if d > desired_separation:
            continue

        count += 1

        #repulsion_force += ((r_rep - d) * p_rep * (maths.unitVect(neighbor_pos_diff)))
        repulsion_force+=diff/d
        repulsion_force = repulsion_force*p_rep
        
    
    # strg = np.linalg.norm(repulsion_force)
    # if strg !=0:
    #     print("Repulsion vector of {} is x:{} y:{} z:{} and strength is {}".format(whichCF, repulsion_force[0], repulsion_force[1], repulsion_force[2], strg))

    return repulsion_force

def align(positions, velocities, whichCF,rayon, p_align ):
            
    pos = positions[whichCF]
    vel = velocities[whichCF]
    alignment_force = np.zeros(3)
    count=0

    for i in positions.keys():
        if i == whichCF:
            continue
        
        neighbor_pos_diff = positions[i] - positions[whichCF]
        distance  = length(neighbor_pos_diff)
        """
        neighbor_vel_diff = velocities[i] - velocities[whichCF]
        speed = np.linalg.norm(neighbor_vel_diff)
        unit_vect = maths.unitVect(neighbor_vel_diff)
        maxVelDiff = max(V_frict_l,maths.VelDecayLinSqrt(distance, p_l, Acc_l,speed, R_0_l))
        if speed > maxVelDiff:
            unit_vect *= (C_frict_l * (speed - maxVelDiff))
            alignment_force += unit_vect
            
        """
        if distance < rayon:
            alignment_force += velocities[i]
            count+=1
    alignment_force /= count
    
    return alignment_force*p_align
    
        
    
def cohesion(positions, whichCF, r_att, p_att):
    attraction_force = np.zeros(3)
    interacting_units = 0
    sum = np.zeros(3)

    for i in positions.keys():

        if i == whichCF: 
            continue

        neighbor_pos_diff = positions[i] - positions[whichCF]
        distance  = length(neighbor_pos_diff)

        print(distance)

        if distance > r_att : 
            continue

        interacting_units += 1
        sum+=positions[i]
        
    if interacting_units > 0:
        sum /= interacting_units
        attraction_force = seek(sum)    
        
    attraction_force += ((distance - r_att) * p_att * (maths.unitVect(neighbor_pos_diff)))

    if sum >0 :
        sum /= interacting_units
        desired = sum - positions[whichCF]
        desired = desired.normalized()
        desired *= p_att
        attraction_force = desired
        
    else:
        attraction_force = np.zeros(3)
        
    
    # strg = np.linalg.norm(attraction_force)
    # if strg != 0:
    #     print("Attraction vector of {} is x:{} y:{} z:{} and strength is {}".format(whichCF, attraction_force[0], attraction_force[1], attraction_force[2], strg))
    print(attraction_force)
    return attraction_force


def run_sequence(scf, sequence):
    try:
        cf = scf.cf
        start_distance_printing(scf)
        global en_cours
        en_cours = True
        start_time = time.time()
        current_time = start_time
        total_time = 5
        box_limits = [-0.75, 1.15, 2.0]
        v_0 = 0.3

        print(scf.cf.link_uri)
        print(sequence)


        if scf.cf.link_uri == uris[0]:   
            with PositionHlCommander(cf, default_velocity=0.2, controller=PositionHlCommander.CONTROLLER_PID) as pc:

                """for i in [0,0.5,1]:
                    pc.go_to(0.0,0.0,0.5)
                    pc.go_to(1.5,0.0,0.5)
                    pc.go_to(0.0,0.0,0.5)"""
                for i in [0,0.5,1]:
                    pc.go_to(-0.5,-0.5,0.5+i)

                    pc.go_to(1.0,-0.5,0.5+i)

                    pc.go_to(1.0,0.5,0.5+i)

                    pc.go_to(-0.5,0.5,0.5+i)

                en_cours = False

        else :
            take_off(cf, 0.5)
            while en_cours:

                #going_back = is_in_box_limit(box_limits, pos_dict, scf.cf.link_uri, v_0)

                #if all(el == 0 for el in going_back):
                r_att=1.5
                p_att=0.5
                
                r_rep=0.4
                p_rep=0.8
                
                
                r_align=1.0
                p_align=0.5
                
                coef_vitesse=1.0
                
                att = cohesion(pos_dict, scf.cf.link_uri, r_att,p_att)
                rep = separate(pos_dict, scf.cf.link_uri, r_rep, p_rep)
                spp = interactions.compute_self_propulsion(vel_dict, scf.cf.link_uri, v_0)
                ali = align(pos_dict, vel_dict,scf.cf.link_uri,r_align, p_align)

                force = (att + rep + ali)*coef_vitesse

                cf.commander.send_velocity_world_setpoint(force[0], force[1], force[2], 0)
                time.sleep(0.1)


                """else:
                    cf.commander.send_velocity_world_setpoint(going_back[0], going_back[1], going_back[2], 0)
                    time.sleep(0.1)"""


            land(cf, pos_dict[scf.cf.link_uri])

    except Exception as e:
        print(e)


if __name__ == '__main__':

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    print('Connecting to Crazyflies...')
    with Swarm(uris, factory=factory) as swarm:
        print('Connected to  Crazyflies')
        swarm.reset_estimators()
        print('Estimators have been reset')
        swarm.parallel_safe(start_states_log)
        print('Logging states info...')
        swarm.parallel_safe(run_sequence)