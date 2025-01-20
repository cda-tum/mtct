import plotly.graph_objects as go
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

traj = pd.read_csv('tmp/trajectory.csv', index_col=False)

n_trains = int(traj["train_idx"].max() + 1)
max_timestep = int(traj["timestep"].max() + 1)

for train in range(n_trains):
    data = traj.loc[(traj["train_idx"] == train)]
    plt.plot(data['timestep'], data['speed'], label="train " + str(train))

plt.legend()
plt.show()
