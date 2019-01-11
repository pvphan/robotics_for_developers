#!/usr/bin/env python
from os.path import expanduser

import plotly.plotly as py
from r4d_common.plot_tools import get_data, simple_plot, odom_pos

def position_comparison():
    s = [
            ['cv', expanduser('~/fiducial_slam/out_dataset_table.bag'), '/p_cv', odom_pos('z')],
            ['gtsam', expanduser('~/fiducial_slam/out_dataset_table.bag'), '/p_gtsam', odom_pos('z')],
        ]

    df = get_data(s).iloc[:100:2]

    fig = {
        'data': [
            {
                'x': df.index - df.index[0],
                'y': df[who],
                'name': who, 'mode': 'markers',
            } for who in ('cv', 'gtsam')
        ],
        'layout': {
            'xaxis': {'title': 'Time [s]'},
            'yaxis': {'title': "Camera Z position [m]"}
        }
    }

    url = py.plot(fig, filename='r4d_1_camera_position', auto_open=False)

if __name__ == "__main__":
    position_comparison()
