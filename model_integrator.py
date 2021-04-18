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
        self.width = 50
        self.length = 100
        # Number of state variables.
        self.NX = 6
        # Number of input variables
        self.NU = 2

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
                        "lf" 	: 0.029,
                        "lr" 	: 0.033,

                        "car_l": 0.06,
                        "car_w": 0.03,

                        "g"   : 9.81
                    }
    def stateToVector(x):
        xk = np.array(0 for i in range(self.NX))
        xk[0] = x.X
        xk[1] = x.Y
        xk[2] = x.phi
        xk[3] = x.vx
        xk[4] = x.vy
        xk[5] = x.r
        return xk

    def inputToVector(u):
        uk = np.array(0 for i in range(self.NU))
        uk[0] = u.Fx
        uk[1] = u.delta
        return uk

    def getForceFront(x):

        # const double alpha_f = getSlipAngleFront(x);
        # const double F_y = param_.Df * std::sin(param_.Cf * std::atan(param_.Bf * alpha_f ));
        # const double F_x = 0.0;

        f_tire = TireForces(1.0, 1.0)
        return f_tire

    def getF(self, x, u):
        """Return the next state vector based on the state space equation of the model

        Args:
            x(dict vehicle state): The input state space structure.
            u(dict vehicle controls): The control input structure
        """
        # tire_forces_front = getForceFront(x)
        # tire_forces_rear  = getForceRear(x)
        # friction_force = getForceFriction(x)
        tire_forces_front = getForceFront(x)
        tire_forces_rear  = getForceFront(x)
        friction_force = getForceFront(x)

        # state_vector
        f = np.array(0 for i in range(self.NX))

        f[0] = x.vx*math.cos(x.phi) - x.vy*math.sin(x.phi)
        f[1] = x.vy*math.cos(x.phi) + x.vx*math.sin(x.phi)
        f[2] = x.r
        f[3] = 1.0/self.mass*(tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*math.sin(u.delta) + self.mass*x.vy*x.r)
        f[4] = 1.0/self.mass*(tire_forces_rear.F_y + tire_forces_front.F_y*math.cos(u.delta) - self.mass*x.vx*x.r)
        f[5] = 1.0/self.Iz*(tire_forces_front.F_y*param_.lf*math.cos(u.delta) - tire_forces_rear.F_y*param_.lr)
        # f(6) = vs
        # f(7) = dD
        # f(8) = dDelta
        # f(9) = dVs

        return f

    def RK4(x, u, ts):
        """4th order Runge Kutta (RK4) implementation
           4 evaluation points of continuous dynamics
        Args:
            x(dict vehicle state): The input state space structure.
            u(dict vehicle controls): The control input structure
            ts (int): The time step length [seconds]
        """
        # 4th order Runge Kutta (RK4) implementation
        # 4 evaluation points of continuous dynamics
        x_vec = stateToVector(x)
        u_vec = inputToVector(u)
        # evaluating the 4 points
        k1 = self.getF(vectorToState(x_vec),u)
        k2 = self.getF(vectorToState(x_vec+ts/2.*k1),u)
        k3 = self.getF(vectorToState(x_vec+ts/2.*k2),u)
        k4 = self.getF(vectorToState(x_vec+ts*k3),u)
        # combining to give output
        x_next = x_vec + ts*(k1/6.+k2/3.+k3/3.+k4/6.)
        return vectorToState(x_next)
