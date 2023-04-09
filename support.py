import numpy as np
from shapely.geometry.polygon import Polygon
from shapely.geometry import Point
import itertools
import plotly.graph_objects as go


def get_nodes_from_poly(poly, z_min, z_max):
    """
    Provides list of nodes within a polygon given z heights
    :param poly: (list of lists) list of x,y
    :param z_min: (int) height of z minimum
    :param z_max: (list) height of z maximum
    :return: (list of lists) nodes within shape, 3D
    """
    x_max = poly[0][0]
    x_min = poly[0][0]
    y_min = poly[0][1]
    y_max = poly[0][1]
    for point in poly:
        if point[0] > x_max:
            x_max = point[0]
        if point[0] < x_min:
            x_min = point[0]
        if point[1] > y_max:
            y_max = point[1]
        if point[1] < y_min:
            y_min = point[1]
    x = np.linspace(x_min, x_max, x_max - x_min + 1)
    y = np.linspace(y_min, y_max, y_max - y_min + 1)
    z = np.linspace(z_min, z_max, z_max - z_min + 1)
    nodes_ = np.dstack(np.meshgrid(x, y)).reshape(-1, 2)
    polygon = Polygon(poly)
    nodes = []
    for i in nodes_:
        point = Point(i)
        if polygon.contains(point) or polygon.intersection(point):
            nodes.append(list(i))
        else:
            print(f"{i} is outside of poly")
    np_nodes = []
    for i in z:
        for j in nodes:
            a = list(itertools.chain(j, [i]))
            np_nodes.append(tuple(a))
    return np_nodes


class AStarNode:
    def __init__(self, start, finish, zones_dic):
        self.start = start
        self.finish = finish
        self.no_fly_nodes = []
        self.walkable_nodes = self.get_walkable_nodes(zones_dic)
        self.current_node = start
        self.nodes = {start: {'parent': None,
                              'status': 'open',
                              'f': np.linalg.norm(np.array(self.finish) - np.array(self.start)),
                              'g': 0,
                              'h': np.linalg.norm(np.array(self.finish) - np.array(self.start))}
                      }
        self.fly_fig = None
        self.path = []

    def get_walkable_nodes(self, zones):
        """
        Finds all nodes that are NOT within a no fly zone
        :param zones: dictionary based on the zones.json file
        :return fly_nodes: [list] list of tuples that are able to be walked
        """
        fly_nodes = get_nodes_from_poly([tuple(i) for i in zones['fly_zone']['poly']], zones['fly_zone']['H_min'],
                                        zones['fly_zone']['H_max'])
        for i in zones["no_fly_zone"]:
            for j in get_nodes_from_poly([tuple(j) for j in i['poly']], i['H_min'], i['H_max']):
                self.no_fly_nodes.append(tuple(j))

        for node in self.no_fly_nodes:
            if node in fly_nodes:
                fly_nodes.remove(node)
        return fly_nodes

    def add_node(self, location, parent):
        """
        Creates the node within the node dictonary, find f, g, and h, then sets to open
        :param location: (tuple) x, y, z - This is the node location to be created
        :param parent: (tuple) x, y, z - location node is neighbor to parent, exploring node
        :return: None
        """
        if location == self.start:
            return
        self.nodes[(location[0], location[1], location[2])] = {'parent': parent,
                                                               'status': 'open',
                                                               'f': None,
                                                               'g': None,
                                                               'h': None}
        self.calculate_fgh((location[0], location[1], location[2]), parent)

    def calculate_fgh(self, node_loc, parent):
        """
        given a node, calculate the cost from start node, cost to finish node, sum
        Modifies self.node dictionary after calculation
        :param node_loc: (list) x, y, z
        :param parent: (list) x, y, z
        :return: None
        """
        try:
            h = np.linalg.norm(np.array(self.finish) - np.array(node_loc))
            g = self.nodes[parent]['g'] + 1
            f = h + g
            self.nodes[node_loc]['h'] = h
            self.nodes[node_loc]['g'] = g
            self.nodes[node_loc]['f'] = f
        except TypeError as e:
            raise TypeError(f'type error: {node_loc}, {self.nodes[parent]}, {e}')

    def compare_g(self, node_loc, parent):
        if self.nodes[parent]['g'] + 1 < self.nodes[node_loc]['g']:
            self.nodes[node_loc]['g'] = self.nodes[parent]['g'] + 1
            self.nodes[node_loc]['status'] = 'open'
            self.nodes[node_loc]['parent'] = parent

    def open_all_neighbor_node(self, parent):
        """
        from parent node, define all surrounding (available) nodes and define them as open
        :param parent: node within node dictionary
        :return: None
        """
        for i in range(-1, 2):
            for j in range(-1, 2):
                for k in range(-1, 2):
                    loc = (parent[0] + i, parent[1] + j, parent[2] + k)
                    if parent == loc:
                        continue
                    try:
                        if loc not in self.walkable_nodes:
                            continue
                        elif self.nodes[loc]['status'] == 'open':
                            if self.nodes[loc]['g'] <= self.nodes[parent]['g']:
                                continue
                        elif self.nodes[loc]['status'] == 'closed':
                            if self.nodes[loc]['g'] <= self.nodes[parent]['g']:
                                continue
                            self.nodes[loc]['status'] = 'open'
                        self.add_node(loc, parent)
                    except KeyError:
                        self.add_node(loc, parent)

    def check_for_open(self):
        """
        Quickly check for any open nodes
        :return: (bool) returns true at first open node
        """
        for i in list(self.nodes.keys()):
            if self.nodes[i]['status'] == 'open':
                return True
        return False

    def get_lowest_cost(self):
        """
        This function finds the next open node to use based on lowest f cost
        :return lc_node: (tuple) lowest cost node
        """
        lowest_cost = None
        lc_node = None
        for i in self.get_open():
            if not lowest_cost:
                lowest_cost = self.nodes[i]['f']
                lc_node = i
            elif self.nodes[i]['f'] < lowest_cost:
                lowest_cost = self.nodes[i]['f']
                lc_node = i
        self.current_node = lc_node
        self.nodes[lc_node]['status'] = "closed"
        return lc_node

    def get_open(self):
        """
        gets list of all open nodes
        :return open: (list) list of nodes that are open
        """
        open = []
        for i in list(self.nodes.keys()):
            if self.nodes[i]['status'] == 'open':
                open.append(i)
        return open

    def get_path(self):
        """
        Starts at current node and works backward based on parent node until start node
        :return: None - sets self.path
        """
        current_node = self.current_node
        while current_node != self.start:
            self.path.append(current_node)
            current_node = self.nodes[current_node]['parent']
        self.path.append(self.start)
        self.path.reverse()

    def show_graph(self):
        """
        Presents plotly graph of no fly zones and path
        :return: None
        """
        fly_fig = go.Figure(data=[
            go.Scatter3d(x=[i[0] for i in self.no_fly_nodes], y=[i[1] for i in self.no_fly_nodes],
                         z=[i[2] for i in self.no_fly_nodes],
                         mode="markers", marker_color='blue', name='No Fly Zone'),
            go.Scatter3d(x=[i[0] for i in self.path], y=[i[1] for i in self.path], z=[i[2] for i in self.path],
                         mode="markers", marker_color='red', name='Route')])

        fly_fig.update_layout(scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Z'),
                              title='Route To Avoid No Fly Zones')
        fly_fig.show()
