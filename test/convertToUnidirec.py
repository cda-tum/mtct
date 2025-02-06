# Remove all reciprocal edges from graph and adjust successor_list, routes and timetable accordingly
#
# Usage:
# python convertToUnidirec.py NETWORK_NAME_IN_DIRECTORY_STRUCTURE
#

import sys
import os
import json
import networkx as nx
import shutil

def readJson(path):
    with open(path,"r") as f:
        data = f.read()

    jsonObj = json.loads(data)
    return jsonObj

def writeJson(obj, path):
    with open(path, 'w') as g:
        json.dump(obj, g, indent=4)

origin_p = "./example-networks/" + sys.argv[1]
destination_p = "./example-networks-unidirec/" + sys.argv[1]

print("Opening files ...")
old_graph = nx.read_graphml(origin_p + "/network/tracks.graphml")
graph = old_graph.copy()

succ = readJson(origin_p + "/network/successors_cpp.json")
stations = readJson(origin_p + "/timetable/stations.json")
routes = readJson(origin_p + "/routes/routes.json")

for (u, v) in old_graph.edges:
    # Detect double edge
    if not(graph.has_edge(u, v) and graph.has_edge(v, u)):
        continue

    # Remove edge from graph
    print("Graph: Clearing edge", [u, v], "because", [v, u], "exists")
    graph.remove_edge(u, v)

    ## Edit successor list

    # Edit other entries with this edge as destination to remaining edge
    for origin in succ:
        if [u, v] in succ[origin]:
            print("Successors: Replacing removed edge", [u, v],"with remaining edge", [v, u]," as successor of", origin)
            succ[origin].remove([u, v])
            succ[origin].append([v, u])

    # Remove edge from successor list
    print("Successors: Removing edge",[u, v])
    key = "('"+ u + "', '" + v + "')"
    succ.pop(key, None)

    ## Edit rest of data

    # Replace edge in stations with remaining edge if not already present
    for station_key, station in stations.items():
        if [u, v] in station:
            print("Stations: Removed edge", [u, v], "from station", station_key)
            stations[station_key].remove([u, v])

    # Replace edge in routes with remaining edge
    for route_key, route in routes.items():
        if [u, v] in route:
            print("Routes: Replaced edge", [u, v], "with", [v, u], "for route", route_key)
            routes[route_key] = [[v, u] if x==[u, v] else x for x in routes[route_key]]

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
