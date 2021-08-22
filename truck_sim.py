"""
 Pygame base truck and trailer simulator.

 Drive with arrow self.keys.
"""

import pygame
import random
from collections import deque
import math
import os
import time
import truck_model_integrator as mi
import sim_timer as simtimer
 
# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
 
class TruckSimView:
    def __init__(self):
        pygame.init()
        
        self.FRAMES_PER_SEC = 50
        self.UPDATE_SIM_HZ = 50
        self.frame_period = 1.0 / self.FRAMES_PER_SEC
        self.update_sim_period = 1.0 / float(self.UPDATE_SIM_HZ)
        self.frames_per_sim_update = int((1.0 / float(self.UPDATE_SIM_HZ)) * self.FRAMES_PER_SEC)
        print("self.frames_per_sim_update = {0}".format(self.frames_per_sim_update))

        # Set the width and height of the self.screen [width, height]
        self.display_size = (800, 600)
        self.screen = pygame.display.set_mode(self.display_size)
        # The number of pixels per simulated meter. 
        self.orig_pixel_to_meters_scale = 40.0
        self.pixel_to_meters_scale = self.orig_pixel_to_meters_scale  
        self.newtons_per_key_press = 1600.0
        self.rads_per_sec_press = 16.0
        self.zoom = 1
        self.zoom_factor = 0.05
        # These offsets center the vehicle to the display when zooming in or out.
        self.veh_to_pixel_offset_x = 0.0
        self.veh_to_pixel_offset_y = 0.0
        
        pygame.display.set_caption("vehicle Sim")
        
        # Loop until the user clicks the close button.
        self.done = False
        # keep track of which keyboard self.keys are pressed.
        self.keys = {
            "left": 0.0,
            "right": 0.0,
            "up": 0.0,
            "down": 0.0
        }
        
        # Used to manage how fast the self.screen updates
        self.clock = pygame.time.Clock()
        self.tick_count = 0
        self.last_frame_time = time.time()

        # Timers
        self.last_print_time = time.time()
        self.print_time_period = 2.0
        self.game_timer = simtimer.SimTimer()

        # Start at a negative y position since the self.screens top left pos is (0,0)
        self.start_pos = (float(self.display_size[0]/2.0),-1.0*float(self.display_size[1]/2.0))
        # Convert the start pos to coordinates for the vehicle
        self.start_pos = (self.start_pos[0] / self.pixel_to_meters_scale, self.start_pos[1] / self.pixel_to_meters_scale)

        # Get the parameter file we are using for the simulator
        self.FILE_PATH = os.path.dirname(os.path.abspath(__file__))
        self.param_file = self.FILE_PATH + '/config/truck_6_trailer.json'
        self.veh = mi.Vehicle(self.start_pos, self.param_file) 

    def print_veh_state(self, veh):
        print("  vehicle input controls: \n\t\t Fx = {0} \t\t ddelta = {1}".format(veh.control_input["Fx"], veh.control_input["ddelta"]))
        print(" veh.state[0][x] = {0}".format(veh.state[0]["x"]))
        print(" veh.state[0][y] = {0}".format(veh.state[0]["y"]))
        print(" veh.state[0][phi] = {0}".format(veh.state[0]["phi"]))
        print(" veh.state[0][vx] = {0}".format(veh.state[0]["vx"]))
        print(" veh.state[0][vy] = {0}".format(veh.state[0]["vy"]))
        print(" veh.state[0][r] = {0}".format(veh.state[0]["r"]))
        print(" veh.state[0][delta] = {0}".format(veh.state[0]["delta"]))

    def update(self, veh, update_sim_period):
        self.done
        veh.state = veh.simTimeStep(veh.state, veh.control_input, update_sim_period)
        self.done = False

    def blitRotate(self, surface, rect_color, rect_state):
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
        # drawing the rotated rectangle to the self.screen  
        surface.blit(new_image , rect)  

    def get_wheel_rect_states(self, veh, b_index, veh_rect_state):
        """
        Get the rectangles representing the wheels around a given vehicles body. 
        Four wheels are assumed to be located at each corner of the body.

        - veh: The vehicle class holding all vehicle state info.
        - b_index: The body index to get the wheel rectangle information for. This is an index of the trailer number to get the wheel rects for.
                E.g use b_index of zero to retunr the wheel rectangles for the fron cab of the vehicle. Use 1 for the first trailers wheels.
        - veh_rect_state: The rectangle state representing the veh stored as (x,y,length,width) in self.screen coordinates.
        
        return: A list storing the wheel rectangle states. (Rear Left, Rear Right, Front left, Front right) 
                Each wheel rect state stores (x,y, length, width, rectangle_angle) in self.screen coordinates and angle in degrees. East is zero, self.clockwise is positive.
                If the vehicle state contains a wheel count of less then 4 then only the num_tires of wheel states are returned.
        """
        rotation_angle = veh.state[b_index]["phi"] * 180.0 / math.pi  # Convert from radians to degrees.
        rect_x = veh_rect_state[0]
        rect_y = veh_rect_state[1]
        rect_l = veh_rect_state[2]
        rect_w = veh_rect_state[3]
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

        wheel_states = (wheel_rear_left_state, wheel_rear_right_state, wheel_front_left_state, wheel_front_right_state)
        return wheel_states[0:veh.params[b_index]["num_tires"]]

    def recenter_vehicle(self):
        self.veh_to_pixel_offset_x = -self.veh.state[0]["x"] * self.pixel_to_meters_scale + float(self.display_size[0]/2.0)
        self.veh_to_pixel_offset_y = self.veh.state[0]["y"] * self.pixel_to_meters_scale + float(self.display_size[1]/2.0)
        print('RECENTERED')
        return self.veh_to_pixel_offset_x, self.veh_to_pixel_offset_y

    def scale_view(self): 
        """
        Magnify the view. This adjust the global view variables:
            self.zoom, self.pixel_to_meters_scale, self.veh_to_pixel_offset_x, self.veh_to_pixel_offset_y
            This funciton updates these vars to self.zoom the view in or out.
        
        return: None
        """
        self.pixel_to_meters_scale = self.orig_pixel_to_meters_scale * self.zoom
        print('MAGNIFYING VIEW, new self.pixel_to_meters_scale = ', self.pixel_to_meters_scale)
        self.veh_to_pixel_offset_x = -self.veh.state[0]["x"] * self.pixel_to_meters_scale + float(self.display_size[0]/2.0)
        self.veh_to_pixel_offset_y = self.veh.state[0]["y"] * self.pixel_to_meters_scale + float(self.display_size[1]/2.0)

    def run_sim(self):
        # -------- Main Program Loop -----------
        while not self.done:
            # --- Main event loop
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.done = True
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        self.keys["up"] = 1.0
                    elif event.key == pygame.K_DOWN:
                        self.keys["down"] = 1.0
                    elif event.key == pygame.K_LEFT:
                        self.keys["left"] = 1.0
                    elif event.key == pygame.K_RIGHT:
                        self.keys["right"] = 1.0
                    elif event.key == pygame.K_q:
                        self.done = True
                    elif event.key == pygame.K_0:
                        self.zoom = 1
                        print('RESET')
                    elif event.key == pygame.K_c:
                        self.veh_to_pixel_offset_x, self.veh_to_pixel_offset_y = self.recenter_vehicle()
                    elif event.key == pygame.K_MINUS:
                        # self.zoom in out functions
                        if (self.zoom - self.zoom_factor > 0):
                            self.zoom = self.zoom - self.zoom_factor
                            self.scale_view()
                    elif event.key == pygame.K_EQUALS:
                        self.zoom = self.zoom + self.zoom_factor
                        self.scale_view()
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_UP:
                        self.keys["up"] = 0.0
                    elif event.key == pygame.K_DOWN:
                        self.keys["down"] = 0.0
                    elif event.key == pygame.K_LEFT:
                        self.keys["left"] = 0.0
                    elif event.key == pygame.K_RIGHT:
                        self.keys["right"] = 0.0
                # self.zoom in out functions
                if (event.type == pygame.MOUSEBUTTONDOWN and event.button == 4): # wheel rolled up
                    self.zoom = self.zoom + self.zoom_factor
                    view_vars = self.scale_view()
                elif (event.type == pygame.MOUSEBUTTONDOWN and event.button == 5): # wheel rolled down
                    if (self.zoom - self.zoom_factor > 0):
                        self.zoom = self.zoom - self.zoom_factor
                        view_vars = self.scale_view()

            # Auto recenter the vehicle in the game view. 
            # Negate the y positions since pygame uses downwards as positive y direction.
            # Add a vehicle to pixel offset (this centers the veh when self.zooming in or out of the display).
            rect_x = self.veh.state[0]["x"] * self.pixel_to_meters_scale + self.veh_to_pixel_offset_x
            rect_y = -1.0*self.veh.state[0]["y"] * self.pixel_to_meters_scale + self.veh_to_pixel_offset_y
            # If the vehicle is outside the self.screen frame recenter the vehicle in the self.screen.
            if ((rect_x > self.display_size[0]) or (rect_x < 0 )):
                self.veh_to_pixel_offset_x, self.veh_to_pixel_offset_y = self.recenter_vehicle()
            # Negate the y positions since pygame uses downwards as positive y direction.
            if ((rect_y > self.display_size[1]) or (rect_y < 0 )):
                self.veh_to_pixel_offset_x, self.veh_to_pixel_offset_y = self.recenter_vehicle()

            # --- Game logic should go here

            # Update control inputs
            self.veh.control_input["Fx"] = (self.keys["up"] - self.keys["down"]) * self.newtons_per_key_press
            self.veh.control_input["ddelta"] = (self.keys["left"] - self.keys["right"]) * self.rads_per_sec_press / self.FRAMES_PER_SEC
            
            # Run simulation code
            self.tick_count += 1
            if self.tick_count >= self.frames_per_sim_update:
                self.tick_count = 0
                self.update(self.veh, self.update_sim_period)
                # Add timer to keep track of how quickly the sim is running.
                self.game_timer.update()

            # Print out functions.
            time_now = time.time()
            time_diff = (time_now - self.last_print_time)
            if ( time_diff > self.print_time_period):
                self.print_veh_state(self.veh)
                self.last_print_time = time_now
        
            # --- self.screen-clearing code goes here
        
            # Here, we clear the self.screen to white. Don't put other drawing commands
            # above this, or they will be erased with this command.
        
            # If you want a background image, replace this clear with blit'ing the
            # background image.
            self.screen.fill(WHITE)

            # --- Drawing code should go here
            # For each of the rigid bodies making up the vehicle draw the vehicles body and wheels. 
            wheel_states = [[] for dbx in range(self.veh.num_bodies)]
            for dbx in range(self.veh.num_bodies):
                rotation_angle = self.veh.state[dbx]["phi"] * 180.0 / math.pi  # Convert from radians to degrees.
                # Negate the y positions since pygame uses downwards as positive y direction.
                # Add a vehicle to pixel offset (this centers the vehicle when self.zooming in or out of the display).
                rect_x = self.veh.state[dbx]["x"] * self.pixel_to_meters_scale + self.veh_to_pixel_offset_x
                rect_y = -1.0*self.veh.state[dbx]["y"] * self.pixel_to_meters_scale + self.veh_to_pixel_offset_y
                rect_l = self.veh.params[dbx]["veh_l"] * self.pixel_to_meters_scale
                rect_w = self.veh.params[dbx]["veh_w"] * self.pixel_to_meters_scale
                rect_state = (rect_x, rect_y,
                            rect_l, rect_w,
                            rotation_angle)
                #print("rect state = ", rect_state)
                angle_offset = (0.0, 0.0)
                
                self.blitRotate(self.screen, RED, rect_state)
                
                # Draw the vehicle wheels with steering angle. Place at positions around the vehicle. 
                # Remember the rectangles are drawn with the position being the center of the rectangle.
                wheel_states[dbx] = self.get_wheel_rect_states(self.veh, dbx, rect_state)

                for idx, wheel in enumerate(wheel_states[dbx]):
                    self.blitRotate(self.screen, BLACK, wheel_states[dbx][idx])

            # --- Go ahead and update the self.screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to self.FRAMES_PER_SEC frames per second
            #frame_duration = self.clock.tick(self.FRAMES_PER_SEC)
            if ((time.time() - self.last_frame_time) < (self.frame_period)):
                sleep_time = self.frame_period - (time.time() - self.last_frame_time)
                time.sleep(sleep_time)
            self.last_frame_time = time.time()

        # End of the main simualtion loop.
        # Close the window and quit.
        pygame.quit()

if __name__ == "__main__":
    truck_sim_view = TruckSimView()
    truck_sim_view.run_sim()


