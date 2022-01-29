import random
import time

import numpy as np
import networkx as nx
import sklearn.neighbors

from bindings import Segment_2, Point_2
from coMotion.autonomous_player.algorithms.prm import Prm
from coMotion.game.comotion_player import CoMotion_Player
import geometry_utils.collision_detection as collision_detection
from coMotion.autonomous_player.utils.utils import Point, Segment
from coMotion.autonomous_player.plotter.plotter import Plotter
from coMotion.autonomous_player.heuristic.basic_heuristic import RandomHeuristic, BonusDistanceHeuristic


class BasicPlayer(CoMotion_Player):
    """
    A player that plays random PRM for its robots
    This is meant only as a test/example to how true logic should work.
    """

    def __init__(self, player_id, num_landmarks, k_nearest):
        super().__init__(player_id)

        # Store a (static) probabilistic roadmap of the scene
        # Also all robot locations are vertices in the roadmap
        self.prm = None
        self.k_nearest = int(k_nearest)
        self.num_landmarks = int(num_landmarks)
        # game is inited once attach game is called. I would prefer it here on init...

        # self.heuristic = RandomHeuristic(self.game)
        self.heuristic = None

    def preprocess(self):
        self.heuristic = BonusDistanceHeuristic(self.game)
        obstacles_cd = collision_detection.Collision_detector(self.game.obstacles, [], self.game.radius)
        self.prm = Prm(self.game.obstacles, self.robots, self.k_nearest, Prm.l2_norm, obstacles_cd)
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
        self.heuristic.update_before_turn()
        robots_locations = tuple(
            (robot.location.x().to_double(), robot.location.y().to_double()) for robot in self.robots)
        opponent_robots = tuple(
            (robot.location.x().to_double(), robot.location.y().to_double()) for robot in self.game.other_player.robots)

        self.prm.add_team_robots_location(robots_locations)
        self.prm.remove_opponent_robots_location(opponent_robots)

        best_paths = self.prm.find_all_couple_of_paths(robots_locations, self.game.makespan)

        self.prm.re_add_opponent_robots_locations()

        heuristics = {node: self.heuristic.score(node) for node in best_paths}
        sorted_nodes = sorted(heuristics, key=lambda n: heuristics[n])

        # heuristics = {k: heuristics[k] for k in sorted_nodes}
        plotter = Plotter()
        plotter.plot_prm_heat_map(heuristics)

        best_node = sorted_nodes[0]
        paths = {robot: self.point_list_to_segment_list(path) for robot, path in
                 zip(self.robots[:2], best_paths[best_node])}

        # import ipdb;ipdb.set_trace()

        paths[self.robots[2]] = [Segment_2(self.robots[2].location, self.robots[2].location)]
        self.equalise_paths_length(paths)

        return paths

    def _advance_one_step(self):
        # Advance all robots in one random step, and return the total makespan
        makespan = 0
        steps = {}

        for robot in self.robots:
            neighbors = self.prm[robot.location]  # all possible next steps.
            random_next = random.choice(list(neighbors.keys()))  # choose one randomally

            weight = neighbors[random_next]['weight']  # next random step weight.
            edge = Segment_2(robot.location, random_next)  # corresponding edge.

            # All chosen paths from before plus the new one.
            parallel_edges = [edge] + [steps[other_robot] for other_robot in steps]

            other_team_edges = [Segment_2(oponent_robot.location, oponent_robot.location) for oponent_robot in
                                self.game.other_player.robots]  # oponent player robot new locations.

            total_robot_edges = parallel_edges + other_team_edges
            if collision_detection.check_intersection_against_robots(total_robot_edges,
                                                                     [robot.radius] * len(total_robot_edges)):
                print('Intersection between robots detected...')
                steps[robot] = Segment_2(robot.location, robot.location)  # stay in place
            else:
                steps[robot] = edge
                makespan += weight
                robot.location = random_next  # Typically dangerous, but we return all robots back to place in the end

        return steps, makespan

    def _get_turn_robot_paths(self):
        total_makespan = 0
        paths = {robot: [] for robot in self.robots}

        original_locations = {robot: robot.location for robot in self.robots}

        while True:
            steps, makespan = self.advance_one_step()
            if total_makespan + makespan >= self.game.makespan:
                break
            total_makespan += makespan
            for robot in steps:
                paths[robot].append(steps[robot])

        # restore original locations
        for robot in original_locations:
            robot.location = original_locations[robot]

        return paths
