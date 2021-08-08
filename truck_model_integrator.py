import math
import numpy as np

class TireForces:
    def __init__(self, F_x, F_y):
        self.F_x = F_x
        self.F_y = F_y

# Vehicle
class Vehicle:
    """A simple class defining a truck and trailers.
        The model used is a bicycle model for the truck and the trailers.
        This models both the truck and trailers kinematics 
        and at faster speeds the dynamics as well (using a smoothing factor to
        switch between the models at different speeds). 
        At low speeds use the kinematic model where the dynamics are determined through 
        the center of rotation of each truck and trailer section.
        At high speeds use the dynamical model which models the tyre frictions of the truck
        and trailers.
    """
    # start_pos is a tuple (x,y) a 2d position
    def __init__(self, start_pos):

        self.start_pos = (start_pos[0], start_pos[1])

        # Number of trailers
        self.num_trailers = 2
        # The number of rigid bodies in the model. There is a drivers cab of the truck plus the number of trailers.
        self.num_bodies = self.num_trailers + 1
        # Number of state variables.
        self.NX = 6 * (self.num_bodies)
        # Number of input variables
        self.NU = 2
        # The amount of time to step through each timestep. 
        # This is used in the model integrator when progating the state space forward.
        self.fine_time_step_ = 0.001  # [secs]

        # Used internally to convert from vec to state dict.
        # These save us from having to create new arrays each time. 
        self.xk_ = np.array([0.0 for i in range(self.NX)])
        self.uk_ = np.array([0.0 for i in range(self.NU)])
        self.f_ = np.array([0.0 for i in range(self.NX)])

        self.params = dict()
        # The drivers cab is classified at trailer zero "T0". The following params all relate to just T0.
        self.params["T0"] = {
                "mass" 	: 2000.0, # Mass of this section of the vehicle [kg]
                "Iz" 	: 5200.0, # The rotation inertia in the z axis (yaw) [kg*m^2] of this section of the vehicle
                "lf" 	: 1.463,  # Distance from rear axel to COG along this section of the vehicles center line. 
                "lr" 	: 1.757,  # Distance from front axel to COG along this section of the vehicles center line. 

                "car_l": 5.426,   # The vehicles length [m]
                "car_w": 2.163,   # The vehicles width [m]

                "g"   : 9.81,      # Gravity acceleration.
                
                # Tire parameters (front)
                "Bf"	: 11.0, 
                "Cf" 	: 1.4,
                "Df" 	: 0.45,
                # Tire parameters (rear)
                "Br" 	: 11.0,
                "Cr" 	: 1.4,
                "Dr" 	: 0.5,  
                # Friction characteritic params
                "Cr0" : 0.0518,  # Zero offset friciton force. 
                "Cr2" : 0.00035, # quadratic friciton multiplier on the vehicle velocity. 
            }
        # Add to the params dictionary the state variables for each trailer. 
        for bdx in range(self.num_trailers):
            trailer_number = bdx + 1
            self.params["T" + str(trailer_number)] = {
                "mass" 	: 2000.0, # Mass of this section of the vehicle [kg]
                "Iz" 	: 5200.0, # The rotation inertia in the z axis (yaw) [kg*m^2] of this section of the vehicle
                "lf" 	: 1.463,  # Distance from rear axel to COG along this section of the vehicles center line. 
                "lr" 	: 1.757,  # Distance from front axel to COG along this section of the vehicles center line. 

                "car_l": 5.426,   # The vehicles length [m]
                "car_w": 2.163,   # The vehicles width [m]

                "g"   : 9.81,      # Gravity acceleration.
                
                # Tire parameters (front)
                "Bf"	: 11.0, 
                "Cf" 	: 1.4,
                "Df" 	: 0.45,
                # Tire parameters (rear)
                "Br" 	: 11.0,
                "Cr" 	: 1.4,
                "Dr" 	: 0.5,  
                # Friction characteritic params
                "Cr0" : 0.0518,  # Zero offset friciton force. 
                "Cr2" : 0.00035, # quadratic friciton multiplier on the vehicle velocity. 
            }

        self.control_input = {
                        "Fx": 0.0,  # Force in the longitudinal direction on rear wheels
                        "delta": 0.0 # Front steering tire angle. 
                    }

        # Create a python dictionary storing all the state space variables for the model. 
        # The first entries are the state space for the trucks front drivers cab.
        self.state = dict()
        # The drivers cab is classified at trailer zero "T0". The following state variables all relate to just T0.
        self.state["T0"] = {
                            "x": self.start_pos[0], # X position of the center of gravity of this section of the vehicle.
                            "y": self.start_pos[1], # Y position of the center of gravity of this section of the vehicle.
                            "phi": 0.0,             # The heading angle of this section of the vehicle
                            "vx": 0.0,              # The forwards velocity in vehicle frame of references.
                            "vy": 0.0,              # The lateral velocity in vehicle frame of references.
                            "r": 0.0                # Yaw rate of this section of the vehicle, this is the rate of phi.
                            }
        # Add to the state dictionary the state variables for each trailer. 
        for bdx in range(self.num_trailers):
            trailer_number = bdx + 1
            t_str = "T" + str(trailer_number)
            prev_t_str = "T" + str(trailer_number - 1)
            # Get the starting position of the trailers. This is based on the length and position/heading of the preceeding trailer.
            start_xpos = self.start_pos[0] - self.params[prev_t_str]["lr"]*math.cos(self.state[prev_t_str]["phi"]) - self.params[t_str]["lf"]*math.cos(self.state[t_str]["phi"])
            start_ypos = self.start_pos[0] - self.params[prev_t_str]["lr"]*math.sin(self.state[prev_t_str]["phi"]) - self.params[t_str]["lf"]*math.sin(self.state[t_str]["phi"])
            self.state[t_str] = {
                            "x": start_xpos, # X position of the center of gravity of this section of the vehicle.
                            "y": start_ypos, # Y position of the center of gravity of this section of the vehicle.
                            "phi": 0.0,             # The heading angle of this section of the vehicle
                            "vx": 0.0,              # The forwards velocity in vehicle frame of references.
                            "vy": 0.0,              # The lateral velocity in vehicle frame of references.
                            "r": 0.0                # Yaw rate of this section of the vehicle, this is the rate of phi.
                            }

    def stateToVector(self, x):
        # Go through each trailers state variables, zero is the truck cab, plus one for the end trailer.
        for bdx in range(self.num_bodies):
            t_str = "T" + str(bdx)
            # The state vars are stored a dict of dicts. There are 6 vars stored per trailer section.  
            self.xk_[bdx*6+0] = x[t_str]["x"]
            self.xk_[bdx*6+1] = x[t_str]["y"]
            self.xk_[bdx*6+2] = x[t_str]["phi"]
            self.xk_[bdx*6+3] = x[t_str]["vx"]
            self.xk_[bdx*6+4] = x[t_str]["vy"]
            self.xk_[bdx*6+5] = x[t_str]["r"]
        return self.xk_
    
    def vectorToState(self, x_vec):
        # Return a new state dict of dicts representing all the states of each seciton of the vehicle. 
        new_state = dict()
        for bdx in range(self.num_bodies):
            t_str = "T" + str(bdx)
            self.state[t_str] = {
                            "x": x_vec[bdx*6+0],   # X position of the center of gravity of this section of the vehicle.
                            "y": x_vec[bdx*6+1],   # Y position of the center of gravity of this section of the vehicle.
                            "phi": x_vec[bdx*6+2], # The heading angle of this section of the vehicle
                            "vx": x_vec[bdx*6+3],  # The forwards velocity in vehicle frame of references.
                            "vy": x_vec[bdx*6+4],  # The lateral velocity in vehicle frame of references.
                            "r": x_vec[bdx*6+5]    # Yaw rate of this section of the vehicle, this is the rate of phi.
                            }
        return new_state

    def inputToVector(self, u):
        
        self.uk_[0] = u["Fx"]
        self.uk_[1] = u["delta"]
        return self.uk_

    def getSlipAngleFront(self, x, u):
        # compute slip angels given current state
        slip_ang_f = -math.atan2(x["vy"]+x["r"]*self.params["lf"],x["vx"]) + u["delta"]
        #print("SlipAngleRear = ", slip_ang_r)
        return slip_ang_f

    def getSlipAngleRear(self, x, u):
        # compute slip angels given current state
        slip_ang_r = -math.atan2(x["vy"]-x["r"]*self.params["lr"],x["vx"])
        return slip_ang_r

    def getForceFront(self, x, u):

        alpha_f = self.getSlipAngleFront(x, u)
        F_y = self.params["Df"] * math.sin(self.params["Cf"] * math.atan(self.params["Bf"] * alpha_f ))
        F_y = F_y * self.params["mass"]
        # Rear wheel drive vehicle so no forwards force on the front tires.
        F_x = 0.0
        #print(" getForceFront fx,fy = {0}, {1}  alpha_f = {2}".format(F_x,F_y,alpha_f))
        f_tire = TireForces(F_x, F_y)
        return f_tire
    
    def getForceRear(self, x, u):

        alpha_r = self.getSlipAngleRear(x, u)
        F_y = self.params["Dr"] * math.sin(self.params["Cr"] * math.atan(self.params["Br"] * alpha_r ))
        F_y = F_y * self.params["mass"]
        # Rear wheel drive vehicle with a force related to the control input force on rear wheels u["Fx"]
        #F_x = self.params["Cm1"]*u["Fx"] - self.params["Cm2"]*u["Fx"]*x["vx"]  # - param_.Cr0 - param_.Cr2*std::pow(x.vx,2.0);
        F_x = u["Fx"]  # Perfect control simulated. The exact rear wheel desired force is acheived.
        
        r_tire = TireForces(F_x, F_y)
        return r_tire

    def getForceFriction(self, x):
        #return -params.Cr0 - params.Cr2*std::pow(x.vx,2.0);
        F_f = -self.params["Cr0"] - self.params["Cr2"]*math.pow(x["vx"],2.0)
        return F_f

    def getF(self, x, u, param):
        """
        Return the next state vector based on the state space equation of the model.
        This is a kinematic and dynamic model. 
        At speeds slower then a minimum the kinematic model is used, at speeds faster a dynamic model is used.
        The models are smoothly transitioned between certain speeds. 

        Args:
            x(dict of dict of vehicle state): The input state space structure.
            u(dict vehicle controls): The control input structure.
            param(dict of dict of vehicle parameters): The vehicle parameters for each section. 
        """

        # Dynamic Model.
        # tire_forces_front = self.getForceFront(x,u)
        # tire_forces_rear  = self.getForceRear(x,u)
        # friction_force = self.getForceFriction(x)

        # # state_vector
        # self.f_[0] = x["vx"]*math.cos(x["phi"]) - x["vy"]*math.sin(x["phi"])
        # self.f_[1] = x["vy"]*math.cos(x["phi"]) + x["vx"]*math.sin(x["phi"])
        # self.f_[2] = x["r"]
        # self.f_[3] = 1.0/float(param["mass"]) * (tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*math.sin(u["delta"]) + param["mass"]*x["vy"]*x["r"])
        # self.f_[4] = 1.0/float(param["mass"]) * (tire_forces_rear.F_y + tire_forces_front.F_y*math.cos(u["delta"]) - param["mass"]*x["vx"]*x["r"])
        # self.f_[5] = 1.0/float(param["Iz"]) * (tire_forces_front.F_y*param["lf"]*math.cos(u["delta"]) - tire_forces_rear.F_y*param["lr"])
        # # f(6) = vs
        # # f(7) = dD
        # # f(8) = dDelta
        # # f(9) = dVs
        
        # Kinematic Model
        # state_vector differential equations. State 
        mass_list = [param[t_str]["mass"] for t_str in param.keys()]
        total_mass = float(sum(mass_list))
        t_str_list = ["T" + str(i-1) for i in range(self.num_bodies)]
        
        for bdx in range(self.num_bodies):
            prev_t_str = "T" + str(bdx-1)
            t_str = "T" + str(bdx)
            self.f_[bdx*6+0] = x[t_str]["vx"]*math.cos(x[t_str]["phi"]) - x[t_str]["vy"]*math.sin(x[t_str]["phi"]) # The rate of change of the X position of the center of gravity of this section of the vehicle.
            self.f_[bdx*6+1] = x[t_str]["vy"]*math.cos(x[t_str]["phi"]) + x[t_str]["vx"]*math.sin(x[t_str]["phi"]) # The rate of change of the Y position of the center of gravity of this section of the vehicle.
            self.f_[bdx*6+2] = x[t_str]["r"]                                                                       # The rate of change of the heading angle of this section of the vehicle
            # The rate of change of the forwards velocity in vehicle frame of references.
            # The cab is treated differently to the trailers since it is driving the forwards accel
            if bdx == 0:
                self.f_[bdx*6+3] = 1.0/total_mass * (u["Fx"])
            else:        
                # The value x[prev_t_str]["phi"] - x[t_str]["phi"] is effectively the steering angle of each trailer (the cab has a steering angle of delta).                                             

                self.f_[bdx*6+3] = 1.0/total_mass * (math.cos(x[prev_t_str]["phi"] - x[t_str]["phi"]))
            self.f_[bdx*6+4] = 1.0/float(param["mass"]) * (tire_forces_rear.F_y + tire_forces_front.F_y*math.cos(u["delta"]) - param["mass"]*x["vx"]*x["r"]) # The rate of change of the lateral velocity in vehicle frame of references.
            self.f_[bdx*6+5] = 1.0/float(param["Iz"]) * (tire_forces_front.F_y*param["lf"]*math.cos(u["delta"]) - tire_forces_rear.F_y*param["lr"]) # The rate of change of the Yaw rate of this section of the vehicle, this is the rate of phi.

        return self.f_

    def RK4(self, x, u, ts):
        """4th order Runge Kutta (RK4) implementation
           4 evaluation points of continuous dynamics
        Args:
            x(dict vehicle state): The input state space structure.
            u(dict vehicle controls): The control input structure
            ts (int): The time step length [seconds]
        """
        # 4th order Runge Kutta (RK4) implementation
        # 4 evaluation points of continuous dynamics
        x_vec = self.stateToVector(x)
        u_vec = self.inputToVector(u)
        #import ipdb; ipdb.set_trace()
        # if (x["r"] < -1e-9):
        #     import ipdb; ipdb.set_trace()
        # evaluating the 4 points
        k1 = self.getF(self.vectorToState(x_vec),u,self.params)
        k2 = self.getF(self.vectorToState(x_vec+ts/2.*k1),u,self.params)
        k3 = self.getF(self.vectorToState(x_vec+ts/2.*k2),u,self.params)
        k4 = self.getF(self.vectorToState(x_vec+ts*k3),u,self.params)
        # combining to give output
        x_next = x_vec + ts*(k1/6.+k2/3.+k3/3.+k4/6.)
        return self.vectorToState(x_next)

    def simTimeStep(self, x, u, ts):
        """Simulate the vehicle continuous dynamics
        Args:
            x(dict vehicle state): The input state space structure.
            u(dict vehicle controls): The control input structure
            ts (int): The time step length [seconds]
        """
        #print(" before sim car.state[phi] = {0}".format(x["phi"]))
        #print(" before sim car.state[r] = {0}".format(x["r"]))
        x_next = x
        integration_steps = (int)(ts/self.fine_time_step_)   
        if(ts/self.fine_time_step_ != integration_steps):
            print("Warning ts/self.fine_time_step_{0} integration_steps = {1}".format(ts/self.fine_time_step_,integration_steps))
        for i in range(0,integration_steps):
            x_next = self.RK4(x_next,u,self.fine_time_step_)
        #print(" after sim car.state[x] = {0}".format(x_next["x"]))
        return x_next

    