import time
import logging
import itertools
from typing import Union

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

from bindings import Segment_2, Point_2
from coMotion.game.comotion_player import CoMotion_Robot

from autonomous_player.plotter import plotter
from autonomous_player.utils.profiler import Profiler
from autonomous_player.players.basic_player import BasicPlayer
from autonomous_player.heuristic.basic_heuristic import BONUS_SCORE
from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity, Robot, l2_norm


class FocusedPlayer(BasicPlayer):

    def __init__(self, player_id, num_samples, k_nearest=15, **kwargs):
        super(FocusedPlayer, self).__init__(player_id, num_samples, k_nearest, 'PreProcessingHeuristic')

    def preprocess(self):
        super(FocusedPlayer, self).preprocess()
        self.heuristic.preprocess(self.prm)

    @staticmethod
    def robot_self_cycle(robot) -> tuple[tuple]:
        return tuple([tuple(robot)])

    def create_distance_dictionary(self, robots: list[tuple], goals: tuple[Entity], max_distance) -> tuple[dict, dict]:
        distance_matrix = {robot: {} for robot in robots}
        graph_per_robot = {robot: {} for robot in robots}

        for robot in robots:
            distances, paths = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.prm.graph,
                                                                                            robot)
            for goal in goals:
                if tuple(goal) in distances:
                    distance_matrix[robot][goal] = distances[tuple(goal)]
                    graph_per_robot[robot][goal] = paths[tuple(goal)]
                else:
                    logging.debug(f'goal {goal} is not reachable...')
                    distance_matrix[robot][goal] = 1000  # TODO: Add a path to let us closer to that bonus.
                    graph_per_robot[robot][goal] = self.robot_self_cycle(robot)

        return graph_per_robot, distance_matrix

    @staticmethod
    def sub_path_up_to_distance(path: tuple[Point], max_distance) -> tuple[tuple[Point], float]:
        distance = 0
        new_path = [path[0]]

        for point in path[1:]:
            edge_dist = l2_norm(point, new_path[-1])
            if distance + edge_dist < max_distance:
                new_path += [point]
                distance += edge_dist
            else:
                break

        return tuple(new_path), distance

    def fix_path(self, robots, match: tuple[Entity], distance_matrix: dict[Robot, dict[Entity, float]],
                 paths: dict[Robot, tuple[Point]], makespan: float):
        """Receive a path longer than makespan and shorten it per robot until it fits."""
        distances = [distance_matrix[robot][bonus] for robot, bonus in zip(self.data.robots, match)]
        current_distance = sum(distances)

        if current_distance < makespan:
            return paths, dict(zip(match, distances))

        while current_distance >= makespan:
            maximum_robot = robots[np.argmax(distances)]
            allowed_distance = makespan - (current_distance - max(distances))

            new_path, new_distance = self.sub_path_up_to_distance(paths[maximum_robot], allowed_distance)
            paths[maximum_robot] = new_path
            current_distance -= (max(distances) - new_distance)
            distances[np.argmax(distances)] = new_distance

            if allowed_distance < 0:
                assert new_path == self.robot_self_cycle(maximum_robot)
                assert new_distance == 0

        return paths, dict(zip(match, distances))

    def fix_all_paths(self, all_endpoints, graph_per_robot, distance_matrix) -> tuple[dict, dict]:
        fixed_actions: dict[tuple[Entity], dict[Robot, tuple[Point]]] = {}
        distances_per_match: dict[tuple[Entity], dict[Entity, float]] = {}

        for match in all_endpoints:
            graphs = {robot: graph_per_robot[robot][goal] if goal in graph_per_robot[robot] else [] for robot, goal in
                      zip(graph_per_robot, match)}

            fixed_paths, distances = self.fix_path(self.data.robots, match, distance_matrix, graphs, self.game.makespan)

            fixed_actions[match] = fixed_paths
            distances_per_match[match] = distances

        return fixed_actions, distances_per_match

    def find_best_match_for_bonus_choice(self, bonus_choice: tuple[Entity],
                                         distance_matrix: dict[Robot, dict[Entity, float]]) -> tuple[Entity]:
        all_states = [(sum([distance_matrix[robot][bonus] for robot, bonus in zip(self.data.robots, match)]), match) for
                      match in itertools.permutations(bonus_choice)]
        # TODO: fix state if needed
        return min(all_states, key=lambda x: x[0])[1]

    def all_best_actions(self, goals, distance_matrix) -> tuple[tuple[Entity]]:
        return tuple(self.find_best_match_for_bonus_choice(choice, distance_matrix) for choice in
                     itertools.combinations(goals, len(self.data.robots)))

    @staticmethod
    def match_length(match, robots, distance_matrix):
        return sum([distance_matrix[robot][goal] for robot, goal in zip(robots, match)])

    def find_best_path(self) -> dict[CoMotion_Robot, list[Segment_2]]:
        makespan = self.game.makespan
        goals: tuple[Entity] = self.data.bonuses + self.data.end_circles

        profiler = Profiler()

        graph_per_robot, distance_matrix = self.create_distance_dictionary(self.data.robots, goals, makespan)

        profiler.log('distance matrix')

        all_endpoints = self.all_best_actions(goals, distance_matrix)

        profiler.log('choose all actions')

        fixed_actions, distance_from_endpath_to_bonus_per_match = self.fix_all_paths(all_endpoints,
                                                                                     graph_per_robot,
                                                                                     distance_matrix)

        profiler.log('Fix all actions')

        # TODO: State and match refactor names.
        # TODO: verify it again
        # TODO: Add options for paths in which certain robots don't move.
        # TODO: Check whether issue above makes fixing paths inutil?

        better_states = {match: self.heuristic.upgrade_path_for_best_robot(match,
                                                                           self.data,
                                                                           makespan - self.match_length(match,
                                                                                                        self.data.robots,
                                                                                                        distance_matrix),
                                                                           distance_matrix,
                                                                           graph_per_robot) for match in fixed_actions}
        profiler.log('Run all greedy searches')

        for match in better_states:
            best_robot, better_path, bonus_num = better_states[match]
            if best_robot:
                fixed_actions[match][best_robot] = better_path

        profiler.log(f'Fix paths after greedy search.')

        heuristic_states = [(self.heuristic.score(distance_from_endpath_to_bonus_per_match[match], self.turn) +
                             better_states[match][2] * BONUS_SCORE, match)
                            for match in
                            fixed_actions]

        profiler.log('heuristic all states')

        best_state = max(heuristic_states, key=lambda x: x[0])[1]
        best_graphs = fixed_actions[best_state]

        paths = {comotion_robot: self.point_list_to_segment_list(best_graphs[robot]) for comotion_robot, robot in
                 zip(self.robots, best_graphs)}

        self.equalise_paths_length(paths)

        profiler.log('end bullshit...')
        profiler.total()

        return paths
