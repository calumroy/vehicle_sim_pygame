"""
 Pygame base truck and trailer simulator.

 Drive with arrow keys.
"""

import pygame
import random
from collections import deque
import math
import time
import truck_model_integrator as mi
#from cython_libs import model_integrator as mi
import sim_timer as simtimer
 
# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
 
pygame.init()
 
FRAMES_PER_SEC = 50
UPDATE_SIM_HZ = 50
frame_period = 1.0 / FRAMES_PER_SEC
update_sim_period = 1.0 / float(UPDATE_SIM_HZ)
frames_per_sim_update = int((1.0 / float(UPDATE_SIM_HZ)) * FRAMES_PER_SEC)
print("frames_per_sim_update = {0}".format(frames_per_sim_update))

# Set the width and height of the screen [width, height]
display_size = (800, 600)
screen = pygame.display.set_mode(display_size)
# The number of pixels per simulated meter. 
orig_pixel_to_meters_scale = 40.0
pixel_to_meters_scale = orig_pixel_to_meters_scale  
newtons_per_key_press = 1600.0
rads_per_sec_press = 16.0
zoom = 1
zoom_factor = 0.05
# These offsets center the vehicle to the display when zooming in or out.
veh_to_pixel_offset_x = 0.0
veh_to_pixel_offset_y = 0.0
# Vehicle drawing params
n_anti_aliasing_ratio = 2
border_radius = 10
fill_vehicle_box = True
 
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
tick_count = 0
last_frame_time = time.time()

# Timers
last_print_time = time.time()
print_time_period = 2.0
game_timer = simtimer.SimTimer()

# Start at a negative y position since the screens top left pos is (0,0)
start_pos = (float(display_size[0]/2.0),-1.0*float(display_size[1]/2.0))
# Convert the start pos to coordinates for the vehicle
start_pos = (start_pos[0] / pixel_to_meters_scale, start_pos[1] / pixel_to_meters_scale)
veh = mi.Vehicle(start_pos) 

def print_veh_state(veh):
    print("  Vehicle input controls: \n\t\t Fx = {0} \t\t ddelta = {1}".format(veh.control_input["Fx"], veh.control_input["ddelta"]))
    print(" veh.state[0][x] = {0}".format(veh.state[0]["x"]))
    print(" veh.state[0][y] = {0}".format(veh.state[0]["y"]))
    print(" veh.state[0][phi] = {0}".format(veh.state[0]["phi"]))
    print(" veh.state[0][vx] = {0}".format(veh.state[0]["vx"]))
    print(" veh.state[0][vy] = {0}".format(veh.state[0]["vy"]))
    print(" veh.state[0][r] = {0}".format(veh.state[0]["r"]))
    print(" veh.state[0][delta] = {0}".format(veh.state[0]["delta"]))

def update(veh, update_sim_period):
    global done
    global point
    veh.state = veh.simTimeStep(veh.state, veh.control_input, update_sim_period)
    done = False

def blitRotate(surface, rect_color, rect_state):
    """
    - surface: the pygame sureface to draw on.
    - rect_color: The color to color the rectangle
    - rect_state: Stores the state of the rectangle to draw. (x,y,length,width, rotation_angle)
    - pos: stores the rectangle (x,y,length,width)
    - rotation_angle: in degree
    """
    pos = rect_state[0:4]
    rotation_angle = rect_state[4]
    # define a surface (RECTANGLE)  
    image_orig = pygame.Surface((pos[2] , pos[3] ))  
    # for making transparent background while rotating an image  
    image_orig.set_colorkey(WHITE)  
    # fill the rectangle / surface with the desired color  
    image_orig.fill(rect_color)  
    # creating a copy of orignal image for smooth rotation  
    image = image_orig.copy()  
    image.set_colorkey(WHITE)  
    # define rect for placing the rectangle at the desired position  
    rect = image.get_rect()  
    rect.center = (pos[0]  , pos[1] )  

    # making a copy of the old center of the rectangle  
    old_center = rect.center  
    # rotating the orignal image  
    new_image = pygame.transform.rotate(image_orig , rotation_angle)  
    rect = new_image.get_rect()  
    # set the rotated rectangle to the old center  
    rect.center = old_center  
    # drawing the rotated rectangle to the screen  
    surface.blit(new_image , rect)  

def get_wheel_rect_states(veh, b_index, veh_rect_state):
    """
    Get the rectangles representing the wheels around a given vehicles body. 
    Four wheels are assumed to be located at each corner of the body.

    - veh: The vehicle class holding all vehicle state info.
    - b_index: The body index to get the wheel rectangle information for. This is an index of the trailer number to get the wheel rects for.
               E.g use b_index of zero to retunr the wheel rectangles for the fron cab of the vehicle. Use 1 for the first trailers wheels.
    - veh_rect_state: The rectangle state representing the veh stored as (x,y,width,length) in screen coordinates.
    
    return: A list storing the wheel rectangle states. (Front left, Front right, Rear Left, Rear Right) 
            Each wheel rect state stores (x,y, length, width, rectangle_angle) in screen coordinates and angle in degrees. East is zero, clockwise is positive.
    """
    rotation_angle = veh.state[b_index]["phi"] * 180.0 / math.pi  # Convert from radians to degrees.
    rect_x = veh_rect_state[0]
    rect_y = veh_rect_state[1]
    half_width_x_offset = rect_w/2.0 * math.cos(math.pi/2.0 - veh.state[b_index]["phi"])
    half_width_y_offset = rect_w/2.0 * math.sin(math.pi/2.0 - veh.state[b_index]["phi"])
    half_length_x_offset = rect_l/2.0 * math.cos(- veh.state[b_index]["phi"])
    half_length_y_offset = rect_l/2.0 * math.sin(- veh.state[b_index]["phi"])

    wheel_front_left_xy = (rect_x - half_width_x_offset + half_length_x_offset, rect_y - half_width_y_offset + half_length_y_offset)
    wheel_front_left_angle = rotation_angle + veh.state[b_index]["delta"] * 180.0 / math.pi  # Convert from radians to degrees.
    wheel_front_left_state = (wheel_front_left_xy[0], wheel_front_left_xy[1], rect_l / 4.0, rect_w / 4.0, wheel_front_left_angle)

    wheel_front_right_xy = (rect_x + half_width_x_offset + half_length_x_offset , rect_y + half_width_y_offset + half_length_y_offset)
    wheel_front_right_angle = rotation_angle + veh.state[b_index]["delta"] * 180.0 / math.pi  # Convert from radians to degrees.
    wheel_front_right_state = (wheel_front_right_xy[0], wheel_front_right_xy[1], rect_l / 4.0, rect_w / 4.0, wheel_front_right_angle)

    wheel_rear_left_xy = (rect_x - half_width_x_offset - half_length_x_offset, rect_y - half_width_y_offset - half_length_y_offset)
    wheel_rear_left_angle = rotation_angle # In degrees
    wheel_rear_left_state = (wheel_rear_left_xy[0], wheel_rear_left_xy[1], rect_l / 4.0, rect_w / 4.0, wheel_rear_left_angle)

    wheel_rear_right_xy = (rect_x + half_width_x_offset - half_length_x_offset , rect_y + half_width_y_offset - half_length_y_offset)
    wheel_rear_right_angle = rotation_angle # In degrees.
    wheel_rear_right_state = (wheel_rear_right_xy[0], wheel_rear_right_xy[1], rect_l / 4.0, rect_w / 4.0, wheel_rear_right_angle)

    return (wheel_front_left_state, wheel_front_right_state, wheel_rear_left_state, wheel_rear_right_state)

def recenter_vehicle():
    veh_to_pixel_offset_x = -veh.state[0]["x"] * pixel_to_meters_scale + float(display_size[0]/2.0)
    veh_to_pixel_offset_y = veh.state[0]["y"] * pixel_to_meters_scale + float(display_size[1]/2.0)
    print('RECENTERED')
    return veh_to_pixel_offset_x, veh_to_pixel_offset_y

# -------- Main Program Loop -----------
while not done:
    # --- Main event loop
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                keys["up"] = 1.0
            elif event.key == pygame.K_DOWN:
                keys["down"] = 1.0
            elif event.key == pygame.K_LEFT:
                keys["left"] = 1.0
            elif event.key == pygame.K_RIGHT:
                keys["right"] = 1.0
            elif event.key == pygame.K_q:
                done = True
            elif event.key == pygame.K_0:
                zoom = 1
                print('RESET')
            elif event.key == pygame.K_c:
                veh_to_pixel_offset_x, veh_to_pixel_offset_y = recenter_vehicle()
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                keys["up"] = 0.0
            elif event.key == pygame.K_DOWN:
                keys["down"] = 0.0
            elif event.key == pygame.K_LEFT:
                keys["left"] = 0.0
            elif event.key == pygame.K_RIGHT:
                keys["right"] = 0.0
        # Zoom in out functions
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 4: # wheel rolled up
            zoom += zoom_factor
            pixel_to_meters_scale = orig_pixel_to_meters_scale * zoom
            print('ZOOMING IN, pixel_to_meters_scale = ', pixel_to_meters_scale)
            veh_to_pixel_offset_x = -veh.state[0]["x"] * pixel_to_meters_scale + float(display_size[0]/2.0)
            veh_to_pixel_offset_y = veh.state[0]["y"] * pixel_to_meters_scale + float(display_size[1]/2.0)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 5: # wheel rolled down
            if (zoom - zoom_factor > 0):
                zoom -= zoom_factor
                pixel_to_meters_scale = orig_pixel_to_meters_scale * zoom
                print('ZOOMING OUT, pixel_to_meters_scale = ', pixel_to_meters_scale)
                veh_to_pixel_offset_x = -veh.state[0]["x"] * pixel_to_meters_scale + float(display_size[0]/2.0)
                veh_to_pixel_offset_y = veh.state[0]["y"] * pixel_to_meters_scale + float(display_size[1]/2.0)

    # Auto recenter the vehicle in the game view. 
    # Negate the y positions since pygame uses downwards as positive y direction.
    # Add a vehicle to pixel offset (this centers the veh when zooming in or out of the display).
    rect_x = veh.state[0]["x"] * pixel_to_meters_scale + veh_to_pixel_offset_x
    rect_y = -1.0*veh.state[0]["y"] * pixel_to_meters_scale + veh_to_pixel_offset_y
    # If the vehicle is outside the screen frame recenter the vehicle in the screen.
    if ((rect_x > display_size[0]) or (rect_x < 0 )):
        veh_to_pixel_offset_x, veh_to_pixel_offset_y = recenter_vehicle()
    # Negate the y positions since pygame uses downwards as positive y direction.
    if ((rect_y > display_size[1]) or (rect_y < 0 )):
        veh_to_pixel_offset_x, veh_to_pixel_offset_y = recenter_vehicle()

    # --- Game logic should go here

    # Update control inputs
    veh.control_input["Fx"] = (keys["up"] - keys["down"]) * newtons_per_key_press
    veh.control_input["ddelta"] = (keys["left"] - keys["right"]) * rads_per_sec_press / FRAMES_PER_SEC
    
    # Run simulation code
    tick_count += 1
    if tick_count >= frames_per_sim_update:
        tick_count = 0
        update(veh, update_sim_period)
        # Add timer to keep track of how quickly the sim is running.
        game_timer.update()

    # Print out functions.
    time_now = time.time()
    time_diff = (time_now - last_print_time)
    if ( time_diff > print_time_period):
        print_veh_state(veh)
        last_print_time = time_now
 
    # --- Screen-clearing code goes here
 
    # Here, we clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
 
    # If you want a background image, replace this clear with blit'ing the
    # background image.
    screen.fill(WHITE)

    # --- Drawing code should go here
    # For each of the rigid bodies making up the vehicle draw the vehicles body and wheels. 
    wheel_states = [[] for dbx in range(veh.num_bodies)]
    for dbx in range(veh.num_bodies):
        rotation_angle = veh.state[dbx]["phi"] * 180.0 / math.pi  # Convert from radians to degrees.
        # Negate the y positions since pygame uses downwards as positive y direction.
        # Add a vehicle to pixel offset (this centers the vehicle when zooming in or out of the display).
        rect_x = veh.state[dbx]["x"] * pixel_to_meters_scale + veh_to_pixel_offset_x
        rect_y = -1.0*veh.state[dbx]["y"] * pixel_to_meters_scale + veh_to_pixel_offset_y
        rect_l = veh.params[dbx]["veh_l"] * pixel_to_meters_scale
        rect_w = veh.params[dbx]["veh_w"] * pixel_to_meters_scale
        rect_state = (rect_x, rect_y,
                    rect_l, rect_w,
                    rotation_angle)
        #print("rect state = ", rect_state)
        angle_offset = (0.0, 0.0)
        
        blitRotate(screen, RED, rect_state)
        
        # Draw the vehicle wheels with steering angle. Place at positions around the vehicle. 
        # Remember the rectangles are drawn with the position being the center of the rectangle.
        wheel_states[dbx] = get_wheel_rect_states(veh, dbx, rect_state)

        blitRotate(screen, BLACK, wheel_states[dbx][0])
        blitRotate(screen, BLACK, wheel_states[dbx][1])
        blitRotate(screen, BLACK, wheel_states[dbx][2])
        blitRotate(screen, BLACK, wheel_states[dbx][3])

    # --- Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # --- Limit to FRAMES_PER_SEC frames per second
    #frame_duration = clock.tick(FRAMES_PER_SEC)
    if ((time.time() - last_frame_time) < (frame_period)):
        sleep_time = frame_period - (time.time() - last_frame_time)
        time.sleep(sleep_time)
    last_frame_time = time.time()

# Close the window and quit.
pygame.quit()
