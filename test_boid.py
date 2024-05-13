import cflib.crtp
import time
import math
from threading import Thread
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie import commander


from classes_boid import Boid
from vec2 import vec2
import csv
import os



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
 
def flocking(scf):

    boids = dico_boid.values()
    print(f"boids : {boids}")
    
    objet=dico_boid[scf.cf.link_uri]
    print(f"objet : {objet}")

    while True :

        if scf.cf.link_uri != 'radio://0/80/2M/7' :
    
            objet.run(boids)

            x = objet.velocity.x
            y = objet.velocity.y
            
            with open('velocity.csv', 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([x, y])


            print(f"velocity : {x,y}")

            with MotionCommander(scf, default_height=1) as mc :
                mc.start_linear_motion(x, y, 0)
                time.sleep(0.1)
                mc.stop()
        
            #cf=scf.cf
            
            #cf.commander.send_position_setpoint(x,y,0.5,0)
            
            #cf.commander.send_velocity_world_setpoint(x,y,0,0)
            print(f"uri : {scf.cf.link_uri} velocity : {x,y}")


        else :
            cf = scf.cf
            with PositionHlCommander(cf, default_velocity=0.2, controller=PositionHlCommander.CONTROLLER_PID) as pc:
            
                            for i in range(3):
                                pc.go_to(-0.5,-0.5,0.5)

                                pc.go_to(1.0,-0.5,0.5)

                                pc.go_to(1.0,0.5,0.5)

                                pc.go_to(-0.5,0.5,0.5)





def store_pos():
    while True:
        dict_pos = swarm.get_estimated_positions()

        time.sleep(0.1)
        for uri in uris:
            dico_boid[uri].position = vec2(dict_pos[uri].x, dict_pos[uri].y)
            
            with open('positions.csv', 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([uri, dict_pos[uri].x, dict_pos[uri].y])
            
dico_boid = {}
dict_pos = {}

BOX_LIMIT = 2
max_vel = 0.2

uris = {
    'radio://0/80/2M/5',
    'radio://0/80/2M/7',
    # Add more URIs if you want more copters in the swarm
}

for i in uris:
    dico_boid[i] = Boid(0, 0)


print(f"dico_boid : {dico_boid}")


if __name__ == '__main__':
    # Reset the velocity.csv file
    with open('velocity.csv', 'w', newline='') as file:
        pass

    # Reset the positions.csv file
    with open('positions.csv', 'w', newline='') as file:
        pass
    cflib.crtp.init_drivers()
    print('Scanning interfaces for Crazyflies...')
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print('Connected to Crazyflies')
        swarm.parallel_safe(light_check)
        swarm.reset_estimators()
        swarm.parallel_safe(take_off)
        time.sleep(3)
        Thread(target=store_pos, args=(), daemon=True).start() 
        swarm.parallel_safe(flocking)
        time.sleep(3)
        swarm.parallel_safe(land)
        
        
        