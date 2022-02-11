import time
import random
from typing import Callable

import networkx as nx
from dataclasses import dataclass
from sklearn.neighbors import NearestNeighbors

from bindings import Point_2, Segment_2
from coMotion.game.comotion_player import CoMotion_Robot
import geometry_utils.collision_detection as collision_detection

from autonomous_player.utils import utils
from autonomous_player.utils.utils import Point, Segment, Entity


class Prm:
    def __init__(self, obstacles: list, robots: list, k_nearest: int, norm: Callable, collision_detector):
        """Probabilistic road map - sample randomly (in some different ways) some points, then connect them using knn"""
        self.obstacles = obstacles
        self.robots = robots
        self.k_nearest = k_nearest

        self.graph = nx.Graph()
        self.sampled_points: [Point] = []
        self.knn = NearestNeighbors(n_neighbors=self.k_nearest,
                                    metric=utils.l2_norm,
                                    algorithm='auto')

        self.norm = norm
        self.collision_detector = collision_detector

        # Save opponent robot current locations since we'd want to re-add them back once the robots moved.
        self.opponent_nodes = []
        self.opponent_edges = []

    def __getitem__(self, item):
        return self.graph[item]

    def sample_points(self, x_range, y_range, num_landmarks: int) -> [Point]:
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

    def connect_one_point_to_all_neigbhors(self, knn: NearestNeighbors,
                                           point: Point,
                                           all_points: [Point]):
        edges = []
        k_neighbors_indexes: [int] = knn.kneighbors([tuple(point)], return_distance=False)[0]
        k_neighbors = [all_points[i] for i in k_neighbors_indexes]

        for neighbor in k_neighbors:
            edge = Segment(point, neighbor)
            if self.collision_detector.is_edge_valid(edge.comotion_segment):
                edges += [(point, neighbor, {'weight': self.norm(point, neighbor)})]

        return edges

    def connect_edges(self, points: [Point], nearest_neighbors: NearestNeighbors):
        edges = []

        for i, point in enumerate(points):
            new_edges = self.connect_one_point_to_all_neigbhors(nearest_neighbors, point, points)
            edges += new_edges
            if i % 100 == 0:
                print(f"Connected {i} landmarks to their nearest neighbors")

        return edges

    def create_graph(self, num_landmarks: int, fix_points: tuple[Entity] = None):
        # Compute the scene bounding box and sampling range
        bbox = collision_detection.calc_bbox(self.obstacles)
        x_range = (bbox[0].to_double(), bbox[1].to_double())
        y_range = (bbox[2].to_double(), bbox[3].to_double())

        self.sampled_points = self.sample_points(x_range, y_range, num_landmarks)
        self.sampled_points += [tuple(p) for p in fix_points] if fix_points else []
        self.knn.fit(self.sampled_points)

        edges = self.connect_edges(self.sampled_points, self.knn)

        self.graph.add_nodes_from(self.sampled_points)
        self.graph.add_edges_from(edges)

    def add_team_robots_location(self, robots_locations: [Point]) -> None:
        self.graph.add_nodes_from(robots_locations)

        for robot_location in robots_locations:
            new_edges = self.connect_one_point_to_all_neigbhors(self.knn, robot_location, self.sampled_points)
            self.graph.add_edges_from(new_edges)

    @staticmethod
    def flatten(my_list: list[list]):
        return [item for sublist in my_list for item in sublist]

    def remove_opponent_robots_location(self, robots: [Point]):
        """remove from prm opponent robot location so we won't collide."""
        my_radius = 2  # TODO: Change my_radius to robot's radius.
        # k_neighbors_indexes = [self.knn.kneighbors([tuple(robot)], return_distance=False)[0] for robot in robots]
        # k_neighbors_indexes = self.flatten(k_neighbors_indexes)
        # k_neighbors = [self.sampled_points[i] for i in k_neighbors_indexes]
        robots_points = [point for point in self.sampled_points if
                         any([utils.l2_norm(robot, point) < my_radius for robot in robots])]

        self.opponent_edges = self.flatten([self.graph.edges(neighbor) for neighbor in robots_points])
        self.opponent_nodes = robots_points

        self.graph.remove_nodes_from(robots_points)

    def re_add_opponent_robots_locations(self):
        self.graph.add_nodes_from(self.opponent_nodes)
        self.graph.add_edges_from(self.opponent_edges)

    @dataclass
    class PathCollection:
        distances: dict[Point, float]
        paths: dict[Point, [Point]]

    def find_all_dests_up_to_length_from_source(self, source: Point, max_len: float = 3) -> PathCollection:
        """Find all paths from robot source to all nodes at length <= max_len - all possible moves for this robot.
        Returns: Pair of dictionnairies each containing <node>: <path> and <node>: <distances>
        """
        distances, paths = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.graph,
                                                                                        source,
                                                                                        cutoff=max_len)

        return self.PathCollection(distances, paths)

    def find_all_couple_of_paths(self, robots: [Point], max_len: float = 3) -> dict:
        """find all pairs of paths for both robots to do such that both of them don't exceed max_len

        Returns: A dictionary containing <tuple of two final nodes>: <tuple of paths>
        """
        start_time = time.time()
        possible_paths: dict[CoMotion_Robot, Prm.PathCollection] = {
            robot: self.find_all_dests_up_to_length_from_source(robot, max_len) for
            robot in robots}
        print(
            f'\t Run Dijkstra x3 - {time.time() - start_time}, N = {[len(possible_paths[p].distances) for p in possible_paths]}')

        best_paths = {}
        counter, times = 0, 0
        first_robot_distances = possible_paths[robots[0]]
        second_robot_distances = possible_paths[robots[1]]

        start_time = time.time()
        for destination_node1 in possible_paths[robots[0]].distances:
            for destination_node2 in possible_paths[robots[1]].distances:
                dist1 = first_robot_distances.distances[destination_node1]
                dist2 = second_robot_distances.distances[destination_node2]

                times += 1
                if dist1 + dist2 < max_len:
                    counter += 1

                if dist1 + dist2 < max_len:
                    best_paths[(destination_node1, destination_node2)] = (
                        first_robot_distances.paths[destination_node1],
                        second_robot_distances.paths[destination_node2])

        print(f'\t Found all couples at {times} iteration - {time.time() - start_time}')
        return best_paths
