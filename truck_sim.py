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
rads_per_sec_press = 1.9
zoom = 1
zoom_factor = 0.05
# These offsets center the car to the display when zooming in or out.
car_to_pixel_offset_x = 0.0
car_to_pixel_offset_y = 0.0
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
car = mi.Vehicle(start_pos) 

def print_car_state(car):
    print("  Vehicle input controls: \n\t\t Fx = {0} \n\t\t delta = {1}".format(car.control_input["Fx"], car.control_input["delta"]))
    print(" car.state["T0"][x] = {0}".format(car.state["T0"]["x"]))
    print(" car.state["T0"][y] = {0}".format(car.state["T0"]["y"]))
    print(" car.state["T0"][phi] = {0}".format(car.state["T0"]["phi"]))
    print(" car.state["T0"][vx] = {0}".format(car.state["T0"]["vx"]))
    print(" car.state["T0"][vy] = {0}".format(car.state["T0"]["vy"]))
    print(" car.state["T0"][r] = {0}".format(car.state["T0"]["r"]))

def update(car, update_sim_period):
    global done
    global point
    car.state = car.simTimeStep(car.state, car.control_input, update_sim_period)
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
    # fill the rectangle / surface with green color  
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

def get_wheel_rect_states(car, car_rect_state):
    """
    - car: The vehicle class holding all vehicle state info.
    - car_rect_state: The rectangle state representing the car stored as (x,y,width,length) in screen coordinates.
    
    return: A list storing the wheel rectangle states. (Front left, Front right, Rear Left, Rear Right) 
            Each wheel rect state stores (x,y, length, width, rectangle_angle) in screen coordinates and angle in degrees. East is zero, clockwise is positive.
    """
    rotation_angle = car.state["T0"]["phi"] * 180.0 / math.pi  # Convert from radians to degrees.
    rect_x = car_rect_state[0]
    rect_y = car_rect_state[1]
    half_width_x_offset = rect_w/2.0 * math.cos(math.pi/2.0 - car.state["T0"]["phi"])
    half_width_y_offset = rect_w/2.0 * math.sin(math.pi/2.0 - car.state["T0"]["phi"])
    half_length_x_offset = rect_l/2.0 * math.cos(- car.state["T0"]["phi"])
    half_length_y_offset = rect_l/2.0 * math.sin(- car.state["T0"]["phi"])

    wheel_front_left_xy = (rect_x - half_width_x_offset + half_length_x_offset, rect_y - half_width_y_offset + half_length_y_offset)
    wheel_front_left_angle = rotation_angle + car.control_input["delta"] * 180.0 / math.pi  # Convert from radians to degrees.
    wheel_front_left_state = (wheel_front_left_xy[0], wheel_front_left_xy[1], rect_l / 4.0, rect_w / 4.0, wheel_front_left_angle)

    wheel_front_right_xy = (rect_x + half_width_x_offset + half_length_x_offset , rect_y + half_width_y_offset + half_length_y_offset)
    wheel_front_right_angle = rotation_angle + car.control_input["delta"] * 180.0 / math.pi  # Convert from radians to degrees.
    wheel_front_right_state = (wheel_front_right_xy[0], wheel_front_right_xy[1], rect_l / 4.0, rect_w / 4.0, wheel_front_right_angle)

    wheel_rear_left_xy = (rect_x - half_width_x_offset - half_length_x_offset, rect_y - half_width_y_offset - half_length_y_offset)
    wheel_rear_left_angle = rotation_angle # In degrees
    wheel_rear_left_state = (wheel_rear_left_xy[0], wheel_rear_left_xy[1], rect_l / 4.0, rect_w / 4.0, wheel_rear_left_angle)

    wheel_rear_right_xy = (rect_x + half_width_x_offset - half_length_x_offset , rect_y + half_width_y_offset - half_length_y_offset)
    wheel_rear_right_angle = rotation_angle # In degrees.
    wheel_rear_right_state = (wheel_rear_right_xy[0], wheel_rear_right_xy[1], rect_l / 4.0, rect_w / 4.0, wheel_rear_right_angle)

    return (wheel_front_left_state, wheel_front_right_state, wheel_rear_left_state, wheel_rear_right_state)

def recenter_vehicle():
    car_to_pixel_offset_x = -car.state["T0"]["x"] * pixel_to_meters_scale + float(display_size[0]/2.0)
    car_to_pixel_offset_y = car.state["T0"]["y"] * pixel_to_meters_scale + float(display_size[1]/2.0)
    print('RECENTERED')
    return car_to_pixel_offset_x, car_to_pixel_offset_y

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
                car_to_pixel_offset_x, car_to_pixel_offset_y = recenter_vehicle()
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
            car_to_pixel_offset_x = -car.state["T0"]["x"] * pixel_to_meters_scale + float(display_size[0]/2.0)
            car_to_pixel_offset_y = car.state["T0"]["y"] * pixel_to_meters_scale + float(display_size[1]/2.0)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 5: # wheel rolled down
            if (zoom - zoom_factor > 0):
                zoom -= zoom_factor
                pixel_to_meters_scale = orig_pixel_to_meters_scale * zoom
                print('ZOOMING OUT, pixel_to_meters_scale = ', pixel_to_meters_scale)
                car_to_pixel_offset_x = -car.state["T0"]["x"] * pixel_to_meters_scale + float(display_size[0]/2.0)
                car_to_pixel_offset_y = car.state["T0"]["y"] * pixel_to_meters_scale + float(display_size[1]/2.0)

    # Auto recenter the vehicle in the game view. 
    # Negate the y positions since pygame uses downwards as positive y direction.
    # Add a car to pixel offset (this centers the car when zooming in or out of the display).
    rect_x = car.state["T0"]["x"] * pixel_to_meters_scale + car_to_pixel_offset_x
    rect_y = -1.0*car.state["T0"]["y"] * pixel_to_meters_scale + car_to_pixel_offset_y
    # If the vehicle is outside the screen frame recenter the car in the screen.
    if ((rect_x > display_size[0]) or (rect_x < 0 )):
        car_to_pixel_offset_x, car_to_pixel_offset_y = recenter_vehicle()
    # Negate the y positions since pygame uses downwards as positive y direction.
    if ((rect_y > display_size[1]) or (rect_y < 0 )):
        car_to_pixel_offset_x, car_to_pixel_offset_y = recenter_vehicle()

    # --- Game logic should go here

    # Update control inputs
    car.control_input["Fx"] = (keys["up"] - keys["down"]) * newtons_per_key_press
    car.control_input["delta"] += (keys["left"] - keys["right"]) * rads_per_sec_press / FRAMES_PER_SEC
    
    # Run simulation code
    tick_count += 1
    if tick_count >= frames_per_sim_update:
        tick_count = 0
        update(car, update_sim_period)
        # Add timer to keep track of how quickly the sim is running.
        game_timer.update()

    # Print out functions.
    time_now = time.time()
    time_diff = (time_now - last_print_time)
    if ( time_diff > print_time_period):
        print_car_state(car)
        last_print_time = time_now
 
    # --- Screen-clearing code goes here
 
    # Here, we clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
 
    # If you want a background image, replace this clear with blit'ing the
    # background image.
    screen.fill(WHITE)

    # --- Drawing code should go here
    rotation_angle = car.state["T0"]["phi"] * 180.0 / math.pi  # Convert from radians to degrees.
    # Negate the y positions since pygame uses downwards as positive y direction.
    # Add a car to pixel offset (this centers the car when zooming in or out of the display).
    rect_x = car.state["T0"]["x"] * pixel_to_meters_scale + car_to_pixel_offset_x
    rect_y = -1.0*car.state["T0"]["y"] * pixel_to_meters_scale + car_to_pixel_offset_y
    rect_l = car.params["car_l"] * pixel_to_meters_scale
    rect_w = car.params["car_w"] * pixel_to_meters_scale
    rect_state = (rect_x, rect_y,
                  rect_l, rect_w,
                  rotation_angle)
    #print("rect state = ", rect_state)
    angle_offset = (0.0, 0.0)
    
    blitRotate(screen, RED, rect_state)
    
    # Draw the Car wheels with steering angle. Place at positions around the car. 
    # Remember the rectangles are drawn with the position being the center of the rectangle.
    (wheel_front_left_state, wheel_front_right_state, wheel_rear_left_state, wheel_rear_right_state) = get_wheel_rect_states(car, rect_state)
    
    blitRotate(screen, BLACK, wheel_front_left_state)
    blitRotate(screen, BLACK, wheel_front_right_state)
    blitRotate(screen, BLACK, wheel_rear_left_state)
    blitRotate(screen, BLACK, wheel_rear_right_state)

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
