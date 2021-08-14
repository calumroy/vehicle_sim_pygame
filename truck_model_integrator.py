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
        self.num_trailers = 6
        # The number of rigid bodies in the model. There is a drivers cab of the truck plus the number of trailers.
        self.num_bodies = self.num_trailers + 1
        # Number of state vars per rigid body (per trailer/cab)
        self.N_PB = 7
        # Number of state variables.
        self.NX = self.N_PB * (self.num_bodies)
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

        self.params = list()
        # The drivers cab is classified at trailer zero. The following params all relate to just just the cab.
        self.params.append( 
            {
                "mass" 	: 2000.0, # Mass of this section of the vehicle [kg]
                "Iz" 	: 5200.0, # The rotation inertia in the z axis (yaw) [kg*m^2] of this section of the vehicle
                "lf" 	: 2.213,  # Distance from rear axel to COG along this section of the vehicles center line. 
                "lr" 	: 2.213,  # Distance from front axel to COG along this section of the vehicles center line. 

                "veh_l": 5.426,   # The vehicles length [m]
                "veh_w": 2.163,   # The vehicles width [m]

                "g"   : 9.81,      # Gravity acceleration.
                
                # Number of tires(If 2 then only rear tires otherwise rear and front)
                "num_tires": 4,
                # Tire parameters (front)
                "num_tires": 4,
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
        )
        # Add to the params dictionary the state variables for each trailer. 
        for bdx in range(self.num_trailers):
            self.params.append(
                {
                    "mass" 	: 2000.0, # Mass of this section of the vehicle [kg]
                    "Iz" 	: 5200.0, # The rotation inertia in the z axis (yaw) [kg*m^2] of this section of the vehicle
                    "lf" 	: 2.213,  # Distance from rear axel to COG along this section of the vehicles center line. 
                    "lr" 	: 2.213,  # Distance from front axel to COG along this section of the vehicles center line. 

                    "veh_l": 5.426,   # The vehicles length [m]
                    "veh_w": 2.163,   # The vehicles width [m]

                    "g"   : 9.81,      # Gravity acceleration.
                    
                    # Number of tires(If 2 then only rear tires otherwise rear and front)
                    "num_tires": 2,
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
            )

        self.control_input = {
            "Fx": 0.0,  # Force in the longitudinal direction on rear wheels
            "ddelta": 0.0 # Front steering tire angle rate. We control the rate of steerign not the steering angle directly.
        }

        # Create a python list storing python dicts of all the state space variables for the model. 
        # The first entries are the state space for the trucks front drivers cab.
        self.state = list()
        # The drivers cab is classified at trailer zero. The following state variables all relate to just T0.
        self.state.append(
            {
                "x": self.start_pos[0], # X position of the center of gravity of this section of the vehicle.
                "y": self.start_pos[1], # Y position of the center of gravity of this section of the vehicle.
                "phi": 0.0,             # The heading angle of this section of the vehicle
                "vx": 0.0,              # The forwards velocity in vehicle frame of references.
                "vy": 0.0,              # The lateral velocity in vehicle frame of references.
                "r": 0.0,               # Yaw rate of this section of the vehicle, this is the rate of phi.
                "delta": 0.0            # The steering angle of this section of the vehicle. If this is the front cab then this comes form the the input command otherwise it is just the heading of the preceeding trailer minus the current trailers heading.
            }
        )
        # Add to the state dictionary the state variables for each trailer. 
        for bdx in range(self.num_trailers):
            t_num = bdx + 1
            prev_t_num = bdx
            # Get the starting position of the trailers. This is based on the length and position/heading of the preceeding trailer.
            # TODO
            # Set phi from a parameter input
            start_phi = 0.0 
            start_xpos = self.state[prev_t_num]["x"] - self.params[prev_t_num]["lr"]*math.cos(self.state[prev_t_num]["phi"]) - self.params[t_num]["lf"]*math.cos(start_phi)
            start_ypos = self.state[prev_t_num]["y"] - self.params[prev_t_num]["lr"]*math.sin(self.state[prev_t_num]["phi"]) - self.params[t_num]["lf"]*math.sin(start_phi)
            self.state.append(
                {
                    "x": start_xpos, # X position of the center of gravity of this section of the vehicle.
                    "y": start_ypos, # Y position of the center of gravity of this section of the vehicle.
                    "phi": 0.0,             # The heading angle of this section of the vehicle
                    "vx": 0.0,              # The forwards velocity in vehicle frame of references.
                    "vy": 0.0,              # The lateral velocity in vehicle frame of references.
                    "r": 0.0,               # Yaw rate of this section of the vehicle, this is the rate of phi.
                    "delta": 0.0            # The steering angle of this section of the vehicle. If this is the front cab then this comes form the the input command otherwise it is just the heading of the preceeding trailer minus the current trailers heading.
                }
            )

    def stateToVector(self, x):
        # Go through each trailers state variables, zero is the truck cab, plus one for the end trailer.
        for bdx in range(self.num_bodies):
            # The state vars are stored as a list of dicts. There are self.N_PB vars stored per trailer section.  
            self.xk_[bdx*self.N_PB+0] = x[bdx]["x"]
            self.xk_[bdx*self.N_PB+1] = x[bdx]["y"]
            self.xk_[bdx*self.N_PB+2] = x[bdx]["phi"]
            self.xk_[bdx*self.N_PB+3] = x[bdx]["vx"]
            self.xk_[bdx*self.N_PB+4] = x[bdx]["vy"]
            self.xk_[bdx*self.N_PB+5] = x[bdx]["r"]
            self.xk_[bdx*self.N_PB+6] = x[bdx]["delta"]
        return self.xk_
    
    def vectorToState(self, x_vec):
        # Return a new state list of dicts representing all the states of each section of the vehicle. 
        new_state = list()
        for bdx in range(self.num_bodies):
            new_state.append(
                {
                    "x": x_vec[bdx*self.N_PB+0],    # X position of the center of gravity of this section of the vehicle.
                    "y": x_vec[bdx*self.N_PB+1],    # Y position of the center of gravity of this section of the vehicle.
                    "phi": x_vec[bdx*self.N_PB+2],  # The heading angle of this section of the vehicle
                    "vx": x_vec[bdx*self.N_PB+3],   # The forwards velocity in vehicle frame of references.
                    "vy": x_vec[bdx*self.N_PB+4],   # The lateral velocity in vehicle frame of references.
                    "r": x_vec[bdx*self.N_PB+5],    # Yaw rate of this section of the vehicle, this is the rate of phi.
                    "delta": x_vec[bdx*self.N_PB+6] # The effective steering angle of each trailer. This is actually just the heading of the preceeding trailer minus the current trailers heading. For the front drivers cab this is actually a control input. 
                }
            )
        return new_state

    def inputToVector(self, u):
        
        self.uk_[0] = u["Fx"]
        self.uk_[1] = u["ddelta"]
        return self.uk_

    def getSlipAngleFront(self, x, u):
        # Compute slip angle given current state
        # TODO
        # Fix the x[dbx]["delta"] dbx is the index of each trailer. Calculate a slip per trailer.
        slip_ang_f = -math.atan2(x["vy"]+x["r"]*self.params["lf"],x["vx"]) + x["delta"]
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
            x(list of dict of vehicle state): The input state space structure.
            u(dict vehicle controls): The control input structure.
            param(list of dict of vehicle parameters): The vehicle parameters for each section. 
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
        # state_vector differential equations.  
        mass_list = [param[i]["mass"] for i in range(self.num_bodies)]
        total_mass = float(sum(mass_list))
        # The value x[bdx-1]["phi"] - x[bdx]["phi"] is effectively the steering angle of each trailer (the cab has a steering angle of delta).
        # Create an array storing the real steerinG angle of the front cab and the effective steering angle of each trailer. 
        delta_arr = np.array([x[i-1]["phi"] - x[i]["phi"] if i != 0 else x[i]["delta"] for i in range(self.num_bodies)])
        #delta_arr = np.array([x[i]["delta"] for i in range(self.num_bodies)])

        for bdx in range(self.num_bodies):
            self.f_[bdx*self.N_PB+0] = x[bdx]["vx"]*math.cos(x[bdx]["phi"]) - x[bdx]["vy"]*math.sin(x[bdx]["phi"]) # The rate of change of the X position of the center of gravity of this section of the vehicle.
            self.f_[bdx*self.N_PB+1] = x[bdx]["vy"]*math.cos(x[bdx]["phi"]) + x[bdx]["vx"]*math.sin(x[bdx]["phi"]) # The rate of change of the Y position of the center of gravity of this section of the vehicle.
            self.f_[bdx*self.N_PB+2] = x[bdx]["r"]  # The rate of change of the heading angle of this section of the vehicle
            # The rate of change of the forwards velocity in vehicle frame of references.
            # The cab is treated differently to the trailers since it is driving the forwards acceleration.
            if bdx == 0:
                # vx' = Fx / mass
                self.f_[bdx*self.N_PB+3] = 1.0/total_mass * (u["Fx"])
                # vy' = [delta'*Vx/cos^2(delta) + tan(delta)*vx'] * ( Lr / (Lr + Lf))
                self.f_[bdx*self.N_PB+4] = (u["ddelta"] * x[bdx]["vx"] / math.cos(x[bdx]["delta"])**2 + self.f_[bdx*self.N_PB+3] * math.tan(x[bdx]["delta"])) * (param[bdx]["lr"]/(param[bdx]["lr"] + param[bdx]["lf"]))
                # For small delta this is approximately equal to this:
                # vy' = [delta'*Vx + delta*vx'] * ( Lr / (Lr + Lf))
                # self.f_[bdx*self.N_PB+4] = (u["ddelta"] * x[bdx]["vx"] + self.f_[bdx*self.N_PB+3] * x[bdx]["delta"]) * (param[bdx]["lr"]/(param[bdx]["lr"] + param[bdx]["lf"]))
                # r' = [delta'*Vx/cos^2(delta) + tan(delta)*vx'] * ( 1 / (Lr + Lf))
                # This is the same as r' = vy' / Lr
                self.f_[bdx*self.N_PB+5] = self.f_[bdx*self.N_PB+4] / param[bdx]["lr"] 
                self.f_[bdx*self.N_PB+6] = u["ddelta"]     # The steering angle rate of the cab is from the control inputs.
            else:                                               
                # How quickly each trailer accelerates depends on the steering angle of the prev trailer.
                # vx' = Fx*cos(delta[1:dbx])/total_mass
                self.f_[bdx*self.N_PB+3] = 1.0/total_mass * (u["Fx"] * np.product(np.cos(delta_arr[1:bdx+1])))
                # delta' (steering rate) for a trailer = prev_trailer_heading' - current_trailer_heading'
                #self.f_[bdx*self.N_PB+6] = (self.f_[(bdx-1)*self.N_PB+2] - self.f_[(bdx)*self.N_PB+2])
                self.f_[bdx*self.N_PB+6] = (x[bdx-1]["r"] - x[bdx]["r"])
                # vy' = [delta'*Vx/cos^2(delta) + tan(delta)*vx'] * ( Lr / (Lr + Lf)) + Fx*sin(delta[1:dbx])/total_mass
                self.f_[bdx*self.N_PB+4] = (self.f_[bdx*self.N_PB+6] * x[bdx]["vx"] / math.cos(x[bdx]["delta"])**2 + self.f_[bdx*self.N_PB+3] * math.tan(x[bdx]["delta"])) * (param[bdx]["lr"]/(param[bdx]["lr"] + param[bdx]["lf"])) + 1.0/total_mass * (u["Fx"]*np.product(np.sin(delta_arr[1:bdx+1])))
                # r' = [delta'*Vx/cos^2(delta) + tan(delta)*vx'] * ( 1 / (Lr + Lf))
                self.f_[bdx*self.N_PB+5] = (self.f_[bdx*self.N_PB+6] * x[bdx]["vx"] / math.cos(x[bdx]["delta"])**2 + self.f_[bdx*self.N_PB+3] * math.tan(x[bdx]["delta"])) * (1.0/(param[bdx]["lr"] + param[bdx]["lf"]))

        return self.f_

    def apply_contraints(self, x_next):
        """
        Apply any constraints to the state space. 

        Args:
            x_next(list of dicts): Representing all the next state variables of each section of the vehicle. 
        Return: 
            x_next_constrained(list of dicts): Representing all the next state variables but after apply physical constraints.
        """
        # Make sure each trailer stays connected to the preceeding trailer.
        for bdx in range(self.num_trailers):
            t_num = bdx + 1
            prev_t_num = bdx
            x_next[t_num]["x"] = x_next[prev_t_num]["x"] - self.params[prev_t_num]["lr"]*math.cos(x_next[prev_t_num]["phi"]) - self.params[t_num]["lf"]*math.cos(x_next[t_num]["phi"])
            x_next[t_num]["y"] = x_next[prev_t_num]["y"] - self.params[prev_t_num]["lr"]*math.sin(x_next[prev_t_num]["phi"]) - self.params[t_num]["lf"]*math.sin(x_next[t_num]["phi"])
        return x_next

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
        # evaluating the 4 points
        k1 = self.getF(self.vectorToState(x_vec),u,self.params)
        k2 = self.getF(self.vectorToState(x_vec+ts/2.*k1),u,self.params)
        k3 = self.getF(self.vectorToState(x_vec+ts/2.*k2),u,self.params)
        k4 = self.getF(self.vectorToState(x_vec+ts*k3),u,self.params)
        # combining to give output
        x_next = x_vec + ts*(k1/6.+k2/3.+k3/3.+k4/6.)
        # Add additional hard constraints to the state vect.
        # E.g All trailers must stay attached to the preceeding trailer.
        new_state = self.vectorToState(x_next)
        self.apply_contraints(new_state)
        return new_state

    def simTimeStep(self, x, u, ts):
        """Simulate the vehicle continuous dynamics
        Args:
            x(dict vehicle state): The input state space structure.
            u(dict vehicle controls): The control input structure
            ts (int): The time step length [seconds]
        """
        #print(" before sim veh.state[phi] = {0}".format(x["phi"]))
        #print(" before sim veh.state[r] = {0}".format(x["r"]))
        x_next = x
        integration_steps = (int)(ts/self.fine_time_step_)   
        if(ts/self.fine_time_step_ != integration_steps):
            print("Warning ts/self.fine_time_step_{0} integration_steps = {1}".format(ts/self.fine_time_step_,integration_steps))
        for i in range(0,integration_steps):
            x_next = self.RK4(x_next,u,self.fine_time_step_)
        #print(" after sim veh.state[x] = {0}".format(x_next["x"]))
        return x_next

    