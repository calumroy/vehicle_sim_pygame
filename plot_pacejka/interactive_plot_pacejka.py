"""
Plot the pacejka formula describing 
tire friction forces of a vehcile modeld by 
the bycycle model.
"""

import numpy as np
import pandas as pd
from plotly import __version__
import plotly.offline as pyo
import plotly.graph_objs as go

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

max_vy = 2.0 # m/s, sideways velocity
max_vx = 20.0 # m/s, forwards velocity
max_delta = 0.79 # radians 0.79 = 45 degrees. Maximum steering angle.
max_r = 0.79 # radians/s 0.79 = 45 degs/sec. Maximum yaw rate of the vehicle
num_steps = 1000 # m/s

lr = 1.757 # Distance from front axel to COG along vehicle center line. 
lf = 1.463  # Distance from rear axel to COG along vehicle center line. 



# Pacejka magic values determined through system identification tests.
# Driving of the real vehicle.
Br = 11.0
Cr = 1.4
Dr = 0.45

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



app = dash.Dash()
app.layout = html.Div(
      html.Div([
            html.Div([html.H5("Br, Cr, Dr"),

                    dcc.Slider(id='slider_input_Br',
                                min=0,
                                max=10,
                                step=0.005,
                                value=Br,
                    ),
                    dcc.Slider(id='slider_input_Cr',
                                min=0,
                                max=10,
                                step=0.005,
                                value=Cr,
                    ),
                    dcc.Slider(id='slider_input_Dr',
                                min=0,
                                max=10,
                                step=0.005,
                                value=Cr,
                    )
                    ],style={'width': '400'}
                ),

            html.Div(style={'height': '10'}),

            html.Div(dcc.Input( id='text_input',
                        placeholder='Enter a value...',
                        type='text',
                        value=0.0
                    ),style={'width': '50'}),

            dcc.Graph(id='example',
                     figure={'data':[{'x':alpha_r, 
                                      'y':Fry,
                                      'type':'lines',
                                      'marker':dict(color='#ffbf00')
                                     }],
                              'layout': go.Layout(title='Plot',
                                                  #xaxis = list(range = c(2, 5)),
                                                  yaxis=dict(range=[0, 1])
                                                   )
                               })

          ], style={'width':'600', 'height':'600','display':'inline-block'})
)

# callback - 1 (from slider)
@app.callback(Output('example', 'figure'),
             [Input('slider_input_Br', 'value'),
             Input('slider_input_Cr', 'value'),
             Input('slider_input_Dr', 'value'),
             Input('text_input', 'value')])

def update_plot(slider_input_Br, slider_input_Cr, slider_input_Dr, text_input):
    # if (float(text_input)==0.0):
    #     q = float(slider_input)
    # else:
    #     q = float(text_input)

    #Global variables break dash so reassign thim before using them.
    Br_ = float(slider_input_Br)
    Cr_ = float(slider_input_Cr)
    Dr_ = float(slider_input_Dr)

    Bf_ = Bf
    Cf_ = Cf
    Df_ = Df

    max_alpha_r_ = 0.79  # radians 0.79 rads = 45 degs
    max_alpha_f_ = 2*0.79  # radians 
    alpha_r_ = alpha_r
    alpha_f_ = alpha_f

    # The estimated force in the lateral direction (y) for the rear and front tires.
    Fry_ = Dr_*np.sin(Cr_*np.arctan(Br_*alpha_r_))
    Ffy_ = Df_*np.sin(Cf_*np.arctan(Bf_*alpha_f_))

    figure = {'data': [go.Scatter(x=alpha_r_, y=Fry_,
                                mode='lines+markers',
                                name='Fry')
                      ],
              'layout': go.Layout(title='Pacejka magic formula tire stiffness', 
                                autosize=False,
                                width=600, height=600
                                )
            }
    return figure

if __name__ == '__main__':
    app.run_server()