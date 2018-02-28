from flask import Flask, render_template

import json
import plotly

import pandas as pd
import numpy as np
from collections import deque

app = Flask(__name__)
app.debug = True


@app.route('/')
def index():
    # rng = pd.date_range('1/1/2011', periods=7500, freq='H')
    # ts = pd.Series(np.random.randn(len(rng)), index=rng)
    file = 'cmake-build-debug/data.txt'
    with open(file, 'r') as f:
        ds = deque(f.read().split(' '))
        ds.popleft()
        ds.pop()
        ds = map(lambda x:float(x), ds)

    y = np.random.random(3)
    graphs = [
        dict(
            data=[
                dict(
                    x = np.arange(len(ds)),
                    y = ds,
                    type = 'scatter'
                ),
                dict(
                    x = np.arange(len(ds)),
                    y = np.ones(len(ds))*7.815,
                    type = "scatter"
                )
            ],
            layout=dict(
                title='first graph'
            )
        )
    ]

    # Add "ids" to each of the graphs to pass up to the client
    # for templating
    ids = ['graph-{}'.format(i) for i, _ in enumerate(graphs)]

    # Convert the figures to JSON
    # PlotlyJSONEncoder appropriately converts pandas, datetime, etc
    # objects to their JSON equivalents
    graphJSON = json.dumps(graphs, cls=plotly.utils.PlotlyJSONEncoder)

    return render_template('layouts/index.html',
                           ids=ids,
                           graphJSON=graphJSON)


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=9999)
