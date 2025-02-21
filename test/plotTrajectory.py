import plotly.graph_objects as go
import networkx as nx
import numpy as np
import pandas as pd
import sys

traj = pd.read_csv(sys.argv[1], index_col=False)
G = nx.read_graphml("example-networks-unidirec/" + sys.argv[2] + "/network/tracks.graphml")

n_trains = int(traj["train_idx"].max() + 1)
max_timestep = int(traj["timestep"].max() + 1)
pos=nx.fruchterman_reingold_layout(G)
adj_matrix = nx.adjacency_matrix(G)

edge_x = []
edge_y = []
for count, edge in enumerate(G.edges()):
    x0, y0 = pos[edge[0]]
    x1, y1 = pos[edge[1]]
    edge_x.append(x0)
    edge_x.append(x1)
    edge_x.append(None)
    edge_y.append(y0)
    edge_y.append(y1)
    edge_y.append(None)

edge_trace = go.Scatter(
    x=edge_x, y=edge_y,
    line=dict(width=1, color='black'),
    hoverinfo='none',
    mode='lines')

node_x = []
node_y = []
n_nodes = len(G.nodes())
for node in G.nodes():
    x, y = pos[node]
    node_x.append(x)
    node_y.append(y)

node_trace = go.Scatter(
    x=node_x, y=node_y,
    mode='markers+text',
    textposition="top center",
    text=[str(x) for x in range(1,n_nodes+1)],
    marker=dict(
        size=3,
        color='black',
        line_width=2)
)

train_traces = []
n_slider_steps = 200
edge_nodes_1, edge_nodes_2 = np.nonzero(adj_matrix)

for timestep in range(0, max_timestep, int(np.floor(max_timestep/n_slider_steps))):
    train_x = []
    train_y = []
    for train in range(n_trains):
        train_state = traj.loc[(traj["train_idx"] == train) & (traj["timestep"] == timestep)]
        if train_state.empty:
            x = 0.02 * train
            y = 0
        else:
            pos_node_1 = pos[train_state["edge_src_node"].squeeze()]
            pos_node_2 = pos[train_state["edge_dst_node"].squeeze()]
            x = pos_node_1[0] + train_state["edge_pos"].squeeze() * (pos_node_2[0] - pos_node_1[0])
            y = pos_node_1[1] + train_state["edge_pos"].squeeze() * (pos_node_2[1] - pos_node_1[1])
        train_x.append(x)
        train_y.append(y)

    train_traces.append(go.Scatter(
        x=train_x, y=train_y,
        visible=False,
        mode='markers+text',
        textposition="top center",
        text=[str(x) for x in range(1, n_trains + 1)],
        marker=dict(
            color=list(range(n_trains)),
            colorscale="rainbow",
            size=10,
            line_width=2)
    ))

# Create and add slider
steps = []
n_steps = len(train_traces) + 2
for i in range(2,n_steps):
    visibility = [False] * n_steps
    visibility[i] = True
    visibility[0] = True
    visibility[1] = True
    step = dict(
        method="update",
        args=[{"visible": visibility}],
        label=str((i-2) * np.floor(max_timestep/n_slider_steps))
    )
    steps.append(step)

sliders = [dict(
    active=0,
    currentvalue={"prefix": "Timestep: "},
    pad={"t": 50},
    steps=steps
)]

train_store_bg = [go.Scatter(x=[-0.03, -0.03, 0.2, 0.2, -0.03], y=[-0.03, 0.03, 0.03, -0.03, -0.03], line=go.Line(color="black"), text="Outside Network")]

fig = go.Figure(data=[edge_trace, node_trace] + train_traces + train_store_bg,
             layout=go.Layout(
                title='Train Network',
                titlefont_size=16,
                showlegend=False,
                hovermode='closest',
                uirevision = True,
                margin=dict(b=20,l=5,r=5,t=40),
                sliders=sliders,
                xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
                yaxis=dict(showgrid=False, zeroline=False, showticklabels=False))
                )

# Make initial timestep visible
fig.data[2].visible = True

fig.show()
