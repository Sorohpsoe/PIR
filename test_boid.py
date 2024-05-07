import time
import math
from threading import Thread
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie import commander


from classes_boid import Boid
from vec2 import vec2



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
    objet=dico_boid[scf.cf.link_uri]

    while True :

        if scf.cf.link_uri != 'radio://0/80/2M/3' :
    
            objet.run(boids)

            x = objet.velocity.x
            y = objet.velocity.y

            print(x,y)

            #with MotionCommander(scf, default_height=1) as mc :
                #mc.start_linear_motion(x, y, 0)
                #time.sleep(0.2)
                #mc.stop()
        
            cf=scf.cf
            cf.commander.send_position_setpoint(x,y,0.5,0)


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
               
dico_boid = {}
dict_pos = {}

BOX_LIMIT = 2
max_vel = 0.2

uris = {
    'radio://0/80/2M/3',
    'radio://0/80/2M/6',
    # Add more URIs if you want more copters in the swarm
}

for i in uris:
    dico_boid[i] = Boid(0, 0)


print(dico_boid)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
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
        
        
        