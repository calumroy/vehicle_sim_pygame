"""
 Pygame base vehicle simulator.

 Drive with arrow keys.
"""

import pygame
import random
from collections import deque
import model_integrator as mi
 
# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
 
pygame.init()
 
FRAMES_PER_SEC = 60
UPDATE_SIM_HZ = 5
update_sim_period = 1.0 / float(UPDATE_SIM_HZ)
frames_per_sim_update = int((1.0 / float(UPDATE_SIM_HZ)) * FRAMES_PER_SEC)
print("frames_per_sim_update = {0}".format(frames_per_sim_update))

# Set the width and height of the screen [width, height]
size = (700, 500)
screen = pygame.display.set_mode(size)
pixel_to_meters_scale = 50.0  # The number of pixels per simulated meter. 
newtons_per_key_press = 10000.0
rads_per_key_press = 0.04
 
pygame.display.set_caption("Vehicle Sim")
 
# Loop until the user clicks the close button.
done = False
# keep track of which keyboard keys are pressed.
keys = {
    "left": 0.0,
    "right": 0.0,
    "up": 0.0,
    "down": 0.0
}
 
# Used to manage how fast the screen updates
clock = pygame.time.Clock()

tick_count=0

start_pos = (float(size[0]/2.0),float(size[1]/2.0))
car = mi.Vehicle(start_pos) 

def update(car, update_sim_period):
    global done
    global point
    car.control_input["Fx"]= (keys["up"] - keys["down"]) * newtons_per_key_press
    car.control_input["delta"] = (keys["left"] - keys["right"]) * rads_per_key_press
    print("  Vehicle input controls: \n\t\t Fx = {0} \n\t\t delta = {1}".format(car.control_input["Fx"], car.control_input["delta"]))
    car.state = car.simTimeStep(car.state, car.control_input, update_sim_period)
    print(" car.state[x] = {0}".format(car.state["x"]))
    done = False

def rectRotated( surface, color, pos, fill, border_radius, rotation_angle, rotation_offset_center = (0,0), nAntialiasingRatio = 1 ):
    """
    - rotation_angle: in degree
    - rotation_offset_center: moving the center of the rotation: (-100,0) will turn the rectangle around a point 100 above center of the rectangle,
                                            if (0,0) the rotation is at the center of the rectangle
    - nAntialiasingRatio: set 1 for no antialising, 2/4/8 for better aliasing
    """
    nRenderRatio = nAntialiasingRatio
    
    sw = pos[2]+abs(rotation_offset_center[0])*2
    sh = pos[3]+abs(rotation_offset_center[1])*2

    surfcenterx = sw//2
    surfcentery = sh//2
    s = pygame.Surface( (sw*nRenderRatio,sh*nRenderRatio) )
    s = s.convert_alpha()
    s.fill((0,0,0,0))
    
    rw2=pos[2]//2 # halfwidth of rectangle
    rh2=pos[3]//2

    pygame.draw.rect( s, color, ((surfcenterx-rw2-rotation_offset_center[0])*nRenderRatio,(surfcentery-rh2-rotation_offset_center[1])*nRenderRatio,pos[2]*nRenderRatio,pos[3]*nRenderRatio), fill*nRenderRatio, border_radius=border_radius*nRenderRatio )
    s = pygame.transform.rotate( s, rotation_angle )        
    if nRenderRatio != 1: s = pygame.transform.smoothscale(s,(s.get_width()//nRenderRatio,s.get_height()//nRenderRatio))
    incfromrotw = (s.get_width()-sw)//2
    incfromroth = (s.get_height()-sh)//2
    surface.blit( s, (pos[0]-surfcenterx+rotation_offset_center[0]+rw2-incfromrotw,pos[1]-surfcentery+rotation_offset_center[1]+rh2-incfromroth) )


# -------- Main Program Loop -----------
while not done:
    # --- Main event loop
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                keys["up"] += 1.0
                keys["down"] = 0.0
            elif event.key == pygame.K_DOWN:
                keys["down"] += 1.0
                keys["up"] = 0.0
            elif event.key == pygame.K_LEFT:
                keys["left"] += 1.0
                keys["right"] = 0.0
            elif event.key == pygame.K_RIGHT:
                keys["right"] += 1.0
                keys["left"] = 0.0
            elif event.key == pygame.K_q:
                done = True

 
    # --- Game logic should go here
 
    # --- Screen-clearing code goes here
 
    # Here, we clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
 
    # If you want a background image, replace this clear with blit'ing the
    # background image.
    screen.fill(WHITE)

    # --- Drawing code should go here
    fill = True
    border_radius = 2
    rotation_angle = car.state["phi"]
    rect_state = (car.state["x"],car.state["y"],
                  car.params["car_w"]*pixel_to_meters_scale,
                  car.params["car_l"]*pixel_to_meters_scale)
    angle_offset = (0.0, 0.0)
    rectRotated( screen, RED, rect_state, fill, border_radius, rotation_angle, angle_offset, 2)
    
    tick_count += 1
    if tick_count > frames_per_sim_update:
        tick_count = 0
        update(car, update_sim_period)
    # --- Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
 
    # --- Limit to FRAMES_PER_SEC frames per second
    clock.tick(FRAMES_PER_SEC)
 
# Close the window and quit.
pygame.quit()
