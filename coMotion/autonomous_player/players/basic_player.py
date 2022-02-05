import time
import random
import importlib
from functools import cached_property

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


class RobotsData:
    def __init__(self, game, robots):
        self.game = game
        self.my_robots = robots

    @cached_property
    def robots(self):
        return tuple((robot.location.x().to_double(), robot.location.y().to_double()) for robot in self.my_robots)

    @cached_property
    def opponent_robots(self):
        return tuple(
            (robot.location.x().to_double(), robot.location.y().to_double()) for robot in self.game.other_player.robots)

    @cached_property
    def bonuses(self):
        return tuple((bonus.location.x().to_double(), bonus.location.y().to_double()) for bonus in self.game.bonuses if
                     not bonus.is_collected)

    @cached_property
    def end_circles(self):
        return tuple((bonus.location.x().to_double(), bonus.location.y().to_double()) for bonus in self.game.goals)
        # If end circle is free ?


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

        self.data: RobotsData = None
        self.turn = 0

    @property
    def turns_left(self):
        return self.game.max_turns - self.turn

    def preprocess(self):
        self.heuristic = self.heuristic_class(self.game)
        obstacles_cd = collision_detection.Collision_detector(self.game.obstacles, [], self.game.radius)
        self.prm = Prm(self.game.obstacles, self.robots, self.k_nearest, utils.l2_norm, obstacles_cd)

        self.data = RobotsData(self.game, self.robots)
        self.prm.create_graph(self.num_landmarks, fix_points=self.data.bonuses + self.data.end_circles)

    @staticmethod
    def point_list_to_segment_list(path: [Point]) -> [Segment_2]:
        if len(path) == 1:
            return [Segment_2(Point_2(*path[0]), Point_2(*path[0]))]
        return [Segment_2(Point_2(*a), Point_2(*b)) for a, b in zip(path[:-1], path[1:])]

    @staticmethod
    def equalise_paths_length(paths: dict) -> None:
        """Duplicate last node of all shortest paths so they would all have the same length"""
        max_len = max([len(paths[robot]) for robot in paths])
        for robot in paths:
            if len(paths[robot]) < max_len:
                paths[robot] += [paths[robot][-1]] * (max_len - len(paths[robot]))

    def preprocess_turn(self) -> None:
        """Preprocess all information before turn and return 2 list of robots"""
        self.turn += 1
        self.data = RobotsData(self.game, self.robots)

        self.prm.add_team_robots_location(self.data.robots)
        self.prm.remove_opponent_robots_location(self.data.opponent_robots)

    def postprocess_turn(self) -> None:
        self.prm.re_add_opponent_robots_locations()

    def find_best_path(self):
        best_paths = self.prm.find_all_couple_of_paths(self.data.robots, self.game.makespan)
        heuristics = {node: self.heuristic.score(node) for node in best_paths}
        sorted_nodes = sorted(heuristics, key=lambda n: heuristics[n])

        plotter = Plotter()
        plotter.plot_prm_heat_map(heuristics)

        best_node = sorted_nodes[0]
        paths = {robot: self.point_list_to_segment_list(path) for robot, path in
                 zip(self.robots[:2], best_paths[best_node])}

        paths[self.robots[2]] = [Segment_2(self.robots[2].location, self.robots[2].location)]
        self.equalise_paths_length(paths)

        return paths

    def get_turn_robot_paths(self) -> dict:
        """Apparently I should return a dict of paths..."""
        self.preprocess_turn()

        paths = self.find_best_path()

        self.postprocess_turn()

        return paths
