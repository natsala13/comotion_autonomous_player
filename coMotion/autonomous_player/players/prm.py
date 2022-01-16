import random

import numpy as np
import networkx as nx
from sklearn.neighbors import NearestNeighbors

from bindings import Point_2, Segment_2
from coMotion.game.comotion_game import CoMotion_Game
import geometry_utils.collision_detection as collision_detection


class Prm:
    def __init__(self, game: CoMotion_Game, robots: list, k_nearest: int):
        self.game = game
        self.robots = robots
        self.k_nearest = k_nearest

        self.scene_prm = None

    def __getitem__(self, item):
        return self.scene_prm[item]

    @staticmethod
    def l2_norm(p, q):
        return ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5

    @staticmethod
    def l2_norm_square(p, q):
        return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2

    def sample_points(self, x_range, y_range, num_landmarks: int) -> [Point_2]:
        """Sample num_landmarks points outside of obstacles"""
        i = 0
        points = []

        while i < num_landmarks:
            rand_x = random.uniform(*x_range)
            rand_y = random.uniform(*y_range)
            point = Point_2(rand_x, rand_y)

            if self.game.obstacles_cd.is_point_valid(point):
                points += [point]
                # scene_prm.add_node(point)
                i += 1
                if i % 500 == 0:
                    print(f'{i} landmarks samples')

        return points

    def connect_edge(self, points: [Point_2], _points: np.array, nearest_neighbors: NearestNeighbors):
        edges = []

        for i, point, _point in enumerate(zip(points, _points)):
            k_neighbors_indexes: [int] = nearest_neighbors.kneighbors([_point], return_distance=False)[0]
            k_neighbors: [Point_2] = [point[i] for i in k_neighbors_indexes]

            for neighbor in k_neighbors:
                edge = Segment_2(point, neighbor)

                if self.game.obstacles_cd.is_edge_valid(edge):
                    weight = self.l2_norm([_point[0], _point[1]],
                                          [neighbor.x().to_double(), neighbor.y().to_double()])
                    # scene_prm.add_edge(p, neighbor, weight=weight)
                    edges += [(point, neighbor, {'weight': weight})]

            if i % 100 == 0:
                print("Connected", i, "landmarks to their nearest neighbors")

        return edges

    def create_graph(self, num_landmarks: int):
        self.scene_prm = nx.Graph()

        # Compute the scene bounding box and sampling range
        bbox = collision_detection.calc_bbox(self.game.obstacles)
        x_range = (bbox[0].to_double(), bbox[1].to_double())
        y_range = (bbox[2].to_double(), bbox[3].to_double())

        # Init the PRM with the robot location vertices and sample points
        robot_location_points = [robot.location for robot in self.robots]
        samples_points = self.sample_points(x_range, y_range, num_landmarks)
        points: [Point_2] = robot_location_points + samples_points
        self.scene_prm.add_nodes_from(points)
        _points: np.array = np.array([[p.x().to_double(), p.y().to_double()] for p in points])

        nearest_neighbors = NearestNeighbors(n_neighbors=self.k_nearest,
                                             metric=self.l2_norm,
                                             algorithm='auto')
        nearest_neighbors.fit(_points)

        edges = self.connect_edge(points, _points, nearest_neighbors)
        self.scene_prm.add_edges_from(edges)
