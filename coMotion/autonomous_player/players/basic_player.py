import time
import random
import importlib

import numpy as np
import networkx as nx
import sklearn.neighbors

from bindings import Segment_2, Point_2
from coMotion.autonomous_player.utils import utils
from coMotion.game.comotion_player import CoMotion_Player
from coMotion.autonomous_player.algorithms.prm import Prm
from coMotion.autonomous_player.plotter.plotter import Plotter
import geometry_utils.collision_detection as collision_detection
from coMotion.autonomous_player.utils.utils import Point, Segment
from coMotion.autonomous_player.heuristic import basic_heuristic


class BasicPlayer(CoMotion_Player):
    """
    A player that plays random PRM for its robots
    This is meant only as a test/example to how true logic should work.
    """

    def __init__(self, player_id, num_samples, k_nearest=15, heuristic_class='BonusDistanceHeuristic'):
        super().__init__(player_id)

        # Store a (static) probabilistic roadmap of the scene
        # Also all robot locations are vertices in the roadmap
        self.prm = None
        self.k_nearest = int(k_nearest)
        self.num_landmarks = int(num_samples)
        # game is inited once attach game is called. I would prefer it here on init...

        self.heuristic_class = getattr(basic_heuristic, heuristic_class)
        self.heuristic = None

    def preprocess(self):
        self.heuristic = self.heuristic_class(self.game)
        obstacles_cd = collision_detection.Collision_detector(self.game.obstacles, [], self.game.radius)
        self.prm = Prm(self.game.obstacles, self.robots, self.k_nearest, utils.l2_norm, obstacles_cd)
        self.prm.create_graph(self.num_landmarks)

    @staticmethod
    def point_list_to_segment_list(path: [Point]) -> [Segment_2]:
        if len(path) == 1:
            return [Segment_2(Point_2(*path[0]), Point_2(*path[0]))]
        return [Segment_2(Point_2(*a), Point_2(*b)) for a, b in zip(path[:-1], path[1:])]

    @staticmethod
    def equalise_paths_length(paths: dict) -> None:
        """Duplicate last node of all shortest paths so they would all have the same length"""
        # import ipdb;ipdb.set_trace()
        max_len = max([len(paths[robot]) for robot in paths])
        for robot in paths:
            if len(paths[robot]) < max_len:
                paths[robot] += [paths[robot][-1]] * (max_len - len(paths[robot]))

    def get_turn_robot_paths(self) -> dict:
        """Apparently I should return a dict of paths..."""
        start_time = time.time()
        self.heuristic.update_before_turn()
        robots_locations = tuple(
            (robot.location.x().to_double(), robot.location.y().to_double()) for robot in self.robots)
        opponent_robots = tuple(
            (robot.location.x().to_double(), robot.location.y().to_double()) for robot in self.game.other_player.robots)

        self.prm.add_team_robots_location(robots_locations)
        self.prm.remove_opponent_robots_location(opponent_robots)
        print(f'### preprocessing bullshit - {time.time() - start_time}')

        best_paths = self.prm.find_all_couple_of_paths(robots_locations, self.game.makespan)

        print(f'### All couples - {time.time() - start_time}')

        self.prm.re_add_opponent_robots_locations()

        print(f'Re add robot locations- {time.time() - start_time}')

        heuristics = {node: self.heuristic.score(node) for node in best_paths}

        print(f'### heuristics - {time.time() - start_time}')

        sorted_nodes = sorted(heuristics, key=lambda n: heuristics[n])

        print(f'### Sort - {time.time() - start_time}')
        plotter = Plotter()
        plotter.plot_prm_heat_map(heuristics)

        best_node = sorted_nodes[0]
        paths = {robot: self.point_list_to_segment_list(path) for robot, path in
                 zip(self.robots[:2], best_paths[best_node])}

        paths[self.robots[2]] = [Segment_2(self.robots[2].location, self.robots[2].location)]
        self.equalise_paths_length(paths)

        print(f'### End- {time.time() - start_time}')

        return paths
