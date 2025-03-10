# Add any missing reciprocal edges to graph and adjust successor_list

# Solutions from the Simulator can be exported only to bidirectional networks generated from the same base as the unidirectional network

#
# Usage:
# python convertToBidirec.py NETWORK_NAME_IN_DIRECTORY_STRUCTURE
#

import sys
import os
import json
import networkx as nx
import shutil
import ast

def parse_tuple(string):
    try:
        s = ast.literal_eval(str(string))
        if type(s) == tuple:
            return s
        return
    except:
        return


def readJson(path):
    with open(path,"r") as f:
        data = f.read()

    jsonObj = json.loads(data)
    return jsonObj

def writeJson(obj, path):
    with open(path, 'w') as g:
        json.dump(obj, g, indent=4)

origin_p = "./example-networks/" + sys.argv[1]
destination_p = "./example-networks-sim-bidirec/" + sys.argv[1]

print("Opening files ...")
old_graph = nx.read_graphml(origin_p + "/network/tracks.graphml")
graph = old_graph.copy()

succ = readJson(origin_p + "/network/successors_cpp.json")
stations = readJson(origin_p + "/timetable/stations.json")
routes = readJson(origin_p + "/routes/routes.json")

attb_mbl = nx.get_edge_attributes(graph, "min_block_length")
attb_sp = nx.get_edge_attributes(graph, "max_speed")
attb_le = nx.get_edge_attributes(graph, "length")
attb_br = nx.get_edge_attributes(graph, "breakable")

for (u, v) in old_graph.edges:
    # Add reciprocal
    if graph.has_edge(u, v) and not(graph.has_edge(v, u)):
        print("Graph: Adding reciprocal edge to ", [u, v], "because", [v, u], " does not exist")
        graph.add_edge(v, u, min_block_length=attb_mbl[u,v], max_speed=attb_sp[u,v], length=attb_le[u,v], breakable=attb_br[u,v])

        ## Edit successor list
        key_old = "('"+ u + "', '" + v + "')"
        key_new = "('"+ v + "', '" + u + "')"

        # For each successor of the old edge
        for [o,d] in succ[key_old]:
            # The new edge is a successor of the reciprocal
            print("Successors: Adding new edge ", [v, u]," as a successor of ", [d, o])
            key_succ_recip = "('"+ d + "', '" + o + "')"
            if key_succ_recip in succ:
                succ[key_succ_recip].append([v,u])
            else:
                succ[key_succ_recip] = [[v,u]]

        succ_iter = succ.copy()
        for edge in succ_iter:
            edge_p = parse_tuple(edge)
            o = edge_p[0]
            d = edge_p[1]
            # For each predicessor of the old edge
            if [u, v] in succ_iter[edge]:
                # The new edge can reach the reciprocal
                print("Successors: Adding new edge ", [v, u]," as a predicessor of", [d, o])
                key_pred_recip = "('"+ d + "', '" + o + "')"
                if key_new in succ:
                    succ[key_new].append([d,o])
                else:
                    succ[key_new] = [[d,o]]

os.makedirs(destination_p + "/network", exist_ok=True)
os.makedirs(destination_p + "/routes", exist_ok=True)
os.makedirs(destination_p + "/timetable", exist_ok=True)

print("Saving modified files ...")
nx.write_graphml(graph, destination_p + "/network/tracks.graphml")
writeJson(succ, destination_p + "/network/successors_cpp.json")
writeJson(stations, destination_p + "/timetable/stations.json")
writeJson(routes, destination_p + "/routes/routes.json")

shutil.copyfile(origin_p + "/timetable/schedules.json", destination_p + "/timetable/schedules.json")
shutil.copyfile(origin_p + "/timetable/trains.json", destination_p + "/timetable/trains.json")
