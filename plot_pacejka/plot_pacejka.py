

"""
Plot the pacejka formula describing 
tire friction forces of a vehcile modeld by 
the bycycle model.
"""

import plotly.graph_objects as go

# Create random data with numpy
import numpy as np
np.random.seed(1)

max_vy = 2.0 # m/s, sideways velocity
max_vx = 20.0 # m/s, forwards velocity
max_delta = 0.79 # radians 0.79 = 45 degrees. Maximum steering angle.
max_r = 0.79 # radians/s 0.79 = 45 degs/sec. Maximum yaw rate of the vehicle
num_steps = 1000 # m/s

lr = 1.757 # Distance from front axel to COG along vehicle center line. 
lf = 1.463  # Distance from rear axel to COG along vehicle center line. 



# Pacejka magic values determined through system identification tests.
# Driving of the real vehicle.
Br = 3.3852
Cr = 1.2691
Dr = 0.1737

Bf = 2.579
Cf = 1.2
Df = 0.192

vx = np.linspace(-max_vx, max_vx, num_steps)
vy = np.linspace(-max_vy, max_vy, num_steps)
delta = np.linspace(-max_delta, max_delta, num_steps)
#r is the yaw_rate of the vehicle.
r = np.linspace(-max_r, max_r, num_steps)

# alpha's are the rear and front slip angles of the vehicle.
alpha_r = np.arctan(np.divide((vy - lr*r),vx))
alpha_f = np.arctan(np.divide((vy - lf*r),vx) - delta)

# JUST PLOT THE LATERAL FORCE VS SLIP ANGLE
max_alpha_r = 0.79  # radians 0.79 rads = 45 degs
max_alpha_f = 2*0.79  # radians 
alpha_r = np.linspace(-max_alpha_r, max_alpha_r, num_steps)
alpha_f = np.linspace(-max_alpha_f, max_alpha_f, num_steps)

# The estimated force in the lateral direction (y) for the rear and front tires.
Fry = Dr*np.sin(Cr*np.arctan(Br*alpha_r))
Ffy = Df*np.sin(Cf*np.arctan(Bf*alpha_f))

#import ipdb; ipdb.set_trace()
# Create traces
#fig = go.Figure(data=[go.Surface(z=Fry, x=delta, y=vy)])
fig = go.Figure()
fig.add_trace(go.Scatter(x=alpha_r, y=Fry,
                    mode='lines+markers',
                    name='Fry'))
fig.add_trace(go.Scatter(x=alpha_f, y=Ffy,
                    mode='lines+markers',
                    name='Ffy'))

#go.Surface(z=z2, showscale=False, opacity=0.9),
fig.update_layout(title='Pacejka forward sideways and steering angle', autosize=False,
                  width=800, height=600)
fig.show()