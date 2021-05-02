import math
import numpy as np

class TireForces:
    def __init__(self, F_y, F_x):
        self.F_y = F_y
        self.F_x = F_x

# Vehicle
class Vehicle:
    """A simple class defining a vehicle."""
    # start_pos is a tuple (x,y) a 2d position
    def __init__(self, start_pos):

        self.start_pos = (start_pos[0], start_pos[1])
        # Number of state variables.
        self.NX = 6
        # Number of input variables
        self.NU = 2
        # The amount of time to step through each timestep. 
        # Thsi is used in the model integrator when progating the state space forward.
        self.fine_time_step_ = 0.001  # [secs]

        # Used internally to convert from vec to state dict.
        # These save us from having to create new arrays each time. 
        self.xk_ = np.array([0.0 for i in range(self.NX)])
        self.uk_ = np.array([0.0 for i in range(self.NU)])
        self.f_ = np.array([0.0 for i in range(self.NX)])

        self.control_input = {
                        "Fx": 0.0,
                        "delta": 0.0
                    }

        self.state = {
                        "x": self.start_pos[0],
                        "y": self.start_pos[1],
                        "phi": 0.0,
                        "vx": 0.0,
                        "vy": 0.0,
                        "r": 0.0
                     }

        self.params = {
                        "mass" 	: 2000.0, # Mass of the vehicle [kg]
                        "Iz" 	: 5200.0, # The rotation inertia in the z axis (yaw) [kg*m^2]
                        "lf" 	: 1.463,  # Distance from rear axel to COG along vehicle center line. 
                        "lr" 	: 1.757,  # Distance from front axel to COG along vehicle center line. 

                        "car_l": 5.426,   # The vehicles length [m]
                        "car_w": 2.163,   # The vehicles width [m]

                        "g"   : 9.81,      # Gravity acceleration.
                        
                        # Tire parameters
                        "Bf"	: 2.579, 
                        "Cf" 	: 1.2,
                        "Df" 	: 0.192,
                    }
    def stateToVector(self, x):
        
        self.xk_[0] = x["x"]
        self.xk_[1] = x["y"]
        self.xk_[2] = x["phi"]
        self.xk_[3] = x["vx"]
        self.xk_[4] = x["vy"]
        self.xk_[5] = x["r"]
        return self.xk_
    
    def vectorToState(self, x_vec):
        new_state = {
                    "x": x_vec[0],
                    "y": x_vec[1],
                    "phi": x_vec[2],
                    "vx": x_vec[3],
                    "vy": x_vec[4],
                    "r": x_vec[5]
        }
        return new_state

    def inputToVector(self, u):
        
        self.uk_[0] = u["Fx"]
        self.uk_[1] = u["delta"]
        return self.uk_

    def getSlipAngleFront(self, x, u):
        # compute slip angels given current state
        slip_ang_f = -math.atan2(x["vy"]+x["r"]*self.params["lf"],x["vx"]) + u["delta"]
        return slip_ang_f

    def getForceFront(self, x, u):

        alpha_f = self.getSlipAngleFront(x, u)
        F_y = self.params["Df"] * math.sin(self.params["Cf"] * math.atan(self.params["Bf"] * alpha_f ))
        # Rear wheel drive vehicle so no forwards force on the front tires.
        F_x = 0.0

        f_tire = TireForces(F_x, F_y)
        return f_tire

    def getForceFriction(self, x):
        #return -params.Cr0 - params.Cr2*std::pow(x.vx,2.0);
        return 1.0

    def getF(self, x, u, param):
        """Return the next state vector based on the state space equation of the model

        Args:
            x(dict vehicle state): The input state space structure.
            u(dict vehicle controls): The control input structure
        """
        # tire_forces_front = getForceFront(x)
        # tire_forces_rear  = getForceRear(x)
        # friction_force = getForceFriction(x)
        tire_forces_front = self.getForceFront(x,u)
        tire_forces_rear  = self.getForceFront(x,u)
        friction_force = self.getForceFriction(x)

        # state_vector
        self.f_[0] = x["vx"]*math.cos(x["phi"]) - x["vy"]*math.sin(x["phi"])
        self.f_[1] = x["vy"]*math.cos(x["phi"]) + x["vx"]*math.sin(x["phi"])
        self.f_[2] = x["r"]
        self.f_[3] = 1.0/float(param["mass"]) * (tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*math.sin(u["delta"]) + param["mass"]*x["vy"]*x["r"])
        self.f_[4] = 1.0/float(param["mass"]) * (tire_forces_rear.F_y + tire_forces_front.F_y*math.cos(u["delta"]) - param["mass"]*x["vx"]*x["r"])
        self.f_[5] = 1.0/float(param["Iz"]) * (tire_forces_front.F_y*param["lf"]*math.cos(u["delta"]) - tire_forces_rear.F_y*param["lr"])
        # f(6) = vs
        # f(7) = dD
        # f(8) = dDelta
        # f(9) = dVs

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
        print(" before sim car.state[x] = {0}".format(x["x"]))
        x_next = x
        integration_steps = (int)(ts/self.fine_time_step_)   
        if(ts/self.fine_time_step_ != integration_steps):
            print("Warning")
        for i in range(0,integration_steps):
            x_next = self.RK4(x_next,u,self.fine_time_step_)
        print(" after sim car.state[x] = {0}".format(x_next["x"]))
        return x_next

    