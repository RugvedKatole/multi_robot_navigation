import networkx as nx
from matplotlib import pyplot as plt

G = nx.read_graphml("/home/arms04/catkin_ws/src/mrpp_sumo/graph_ml/grid_5_5.graphml")
edge_nodes ={}
for e in G.edges:
    edge_nodes[G[e[0]][e[1]]['name']] = e
print(G.nodes["12"])
