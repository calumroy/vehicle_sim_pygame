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

app = dash.Dash()
app.layout = html.Div(
      html.Div([
            html.Div([html.H5("Level"),

                    dcc.Slider(id='slider_input',
                                min=0,
                                max=1,
                                step=0.005,
                                value=0.1,
                    )],style={'width': '200'}
                ),

            html.Div(style={'height': '10'}),

            html.Div(dcc.Input( id='text_input',
                        placeholder='Enter a value...',
                        type='text',
                        value=0.0
                    ),style={'width': '50'}),

            dcc.Graph(id='example',
                     figure={'data':[{'x':[1,2],
                                      'y':[0,1],
                                      'type':'bar',
                                      'marker':dict(color='#ffbf00')
                                     }],
                              'layout': go.Layout(title='Plot',
                                                  #xaxis = list(range = c(2, 5)),
                                                  yaxis=dict(range=[0, 1])
                                                   )
                               })

          ], style={'width':'500', 'height':'200','display':'inline-block'})
)

# callback - 1 (from slider)
@app.callback(Output('example', 'figure'),
             [Input('slider_input', 'value'),
             Input('text_input', 'value')])

def update_plot(slider_input, text_input):
    if (float(text_input)==0.0):
        q = float(slider_input)
    else:
        q = float(text_input)

    figure = {'data': [go.Bar(x=[1,2],
                              y=[q, 1-q],
                              marker=dict(color='#ffbf00'),
                              width=0.5
                       )],
              'layout': go.Layout(title='plot',
                                  #xaxis = list(range = c(2, 5)),
                                  yaxis=dict(range=[0, 1])
                                )
            }
    return figure

if __name__ == '__main__':
    app.run_server()