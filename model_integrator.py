import math
import numpy as np

class TireForces:
    F_y
    F_x

# Vehicle
class Vehicle:
    """A simple class defining a vehicle."""
    # start_pos is a tuple (x,y) a 2d position
    def __init__(self, start_pos):

        self.start_pos = (start_pos[0], start_pos[1])
        self.width = 50
        self.length = 100
        self.mass = 2000 # Mass of the vehicle [kg]
        self.Iz = 5200.0 # The rotation inertia in the z axis (yaw) [kg*m^2]
        # Number of state variables.
        self.NX = 10

        self.state = {
                        "x": self.start_pos[0],
                        "y": self.start_pos[1],
                        "phi": 0.0,
                        "vx": 0.0,
                        "vy": 0.0,
                        "r": 0.0
                }

    def getF(self, x, u):
        """Return the next state vector based on the state space equation of the model

        Args:
            x(struct vehicle state): The input state space structure.
            u(struct vehicle controls): The control input structure
        """
        # tire_forces_front = getForceFront(x)
        # tire_forces_rear  = getForceRear(x)
        # friction_force = getForceFriction(x)

        # state_vector
        f = np.array(0 for i in range(self.NX))

        f(0) = x.vx*math.cos(x.phi) - x.vy*math.sin(x.phi)
        f(1) = x.vy*math.cos(x.phi) + x.vx*math.sin(x.phi)
        f(2) = x.r
        f(3) = 1.0/self.mass*(tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*math.sin(delta) + self.mass*vy*r);
        f(4) = 1.0/self.mass*(tire_forces_rear.F_y + tire_forces_front.F_y*math.cos(delta) - self.mass*vx*r);
        f(5) = 1.0/param_.Iz*(tire_forces_front.F_y*param_.lf*math.cos(delta) - tire_forces_rear.F_y*param_.lr);
        f(6) = vs
        f(7) = dD
        f(8) = dDelta
        f(9) = dVs