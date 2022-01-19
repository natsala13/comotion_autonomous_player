import random
from typing import Callable

import numpy as np
import networkx as nx
from sklearn.neighbors import NearestNeighbors

from bindings import Point_2, Segment_2
from coMotion.game.comotion_game import CoMotion_Game
import geometry_utils.collision_detection as collision_detection


class Prm:
    def __init__(self, obstacles: list, robots: list, k_nearest: int, norm: Callable, collision_detector):
        self.obstacles = obstacles
        self.robots = robots
        self.k_nearest = k_nearest

        self.scene_prm = nx.Graph()
        self.sampled_points: [Point_2] = []
        self.knn = NearestNeighbors(n_neighbors=self.k_nearest,
                                    metric=self.l2_norm,
                                    algorithm='auto')

        self.norm = norm
        self.collision_detector = collision_detector

    def __getitem__(self, item):
        return self.scene_prm[item]

    @staticmethod
    def l2_norm(p, q):
        return ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5

    @staticmethod
    def l2_norm_square(p, q):
        return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2

    def sample_points(self, x_range, y_range, num_landmarks: int) -> [float]:
        """Sample num_landmarks points outside of obstacles"""
        i = 0
        points = []

        while i < num_landmarks:
            rand_x = random.uniform(*x_range)
            rand_y = random.uniform(*y_range)
            point = (rand_x, rand_y)

            if self.collision_detector.is_point_valid(Point_2(*point)):
                points += [point]
                # scene_prm.add_node(point)
                i += 1
                if i % 500 == 0:
                    print(f'{i} landmarks samples')

        return points

    @staticmethod
    def to_segment(e):
        return Segment_2(Point_2(*e[0]), Point_2(*e[1]))

    def connect_edges(self, points: [float], nearest_neighbors: NearestNeighbors):
        edges = []

        for i, point in enumerate(points):
            k_neighbors_indexes: [int] = nearest_neighbors.kneighbors([point], return_distance=False)[0]
            k_neighbors: np.array = [points[i] for i in k_neighbors_indexes]
            # k_neighbors: np.array = points[k_neighbors_indexes]

            for neighbor in k_neighbors:
                edge = (point, neighbor)

                if self.collision_detector.is_edge_valid(self.to_segment(edge)):
                    # weight = self.norm([point[0], point[1]], [neighbor.x().to_double(), neighbor.y().to_double()])
                    # scene_prm.add_edge(p, neighbor, weight=weight)
                    edges += [(point, neighbor, {'weight': self.norm(point, neighbor)})]

            if i % 100 == 0:
                print("Connected", i, "landmarks to their nearest neighbors")

        return edges

    def create_graph(self, num_landmarks: int):
        # Compute the scene bounding box and sampling range
        bbox = collision_detection.calc_bbox(self.obstacles)
        x_range = (bbox[0].to_double(), bbox[1].to_double())
        y_range = (bbox[2].to_double(), bbox[3].to_double())

        # Init the PRM with the robot location vertices and sample points
        # robot_location_points = [robot.location for robot in self.robots]
        # points: [Point_2] = robot_location_points + samples_points

        self.sampled_points = self.sample_points(x_range, y_range, num_landmarks)
        self.knn.fit(self.sampled_points)

        edges = self.connect_edges(self.sampled_points, self.knn)

        # import ipdb;ipdb.set_trace()
        self.scene_prm.add_nodes_from(self.sampled_points)
        self.scene_prm.add_edges_from(edges)
