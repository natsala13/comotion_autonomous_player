import time
import logging
import itertools
from typing import Union

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

from bindings import Segment_2, Point_2
from coMotion.game.comotion_player import CoMotion_Robot

from autonomous_player.algorithms.greedy_upgrade_path import UpgradePath
from autonomous_player.plotter import plotter
from autonomous_player.utils.profiler import Profiler
from autonomous_player.players.basic_player import BasicPlayer
from autonomous_player.heuristic.basic_heuristic import BONUS_SCORE
from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity, Robot, Path, l2_norm


class FocusedPlayer(BasicPlayer):

    def __init__(self, player_id, num_samples, k_nearest=15, **kwargs):
        super(FocusedPlayer, self).__init__(player_id, num_samples, k_nearest, 'PreProcessingHeuristic')
        self.upgrade_path = None

    def preprocess(self):
        super(FocusedPlayer, self).preprocess()
        self.heuristic.preprocess(self.prm)
        self.upgrade_path = UpgradePath(self.prm, self.heuristic.bonuses_distances)

    @staticmethod
    def robot_self_cycle(robot) -> Path:
        return Path([tuple(robot)], length=0)

    def create_distance_dictionary(self, robots: list[Robot],
                                   goals: tuple[Entity],
                                   max_distance) -> tuple[dict[Robot, Path], dict[Robot, dict[Entity, float]]]:
        distance_matrix = {robot: {} for robot in robots}
        graph_per_robot = {robot: {} for robot in robots}

        for robot in robots:
            distances, paths = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.prm.graph,
                                                                                            robot)
            for goal in goals:
                if tuple(goal) in distances:
                    distance_matrix[robot][goal] = distances[tuple(goal)]
                    graph_per_robot[robot][goal] = Path(paths[tuple(goal)], length=distances[tuple(goal)],
                                                        endpoint=goal)
                else:
                    logging.debug(f'goal {goal} is not reachable...')
                    distance_matrix[robot][goal] = 1000  # TODO: Add a path to let us closer to that bonus.
                    graph_per_robot[robot][goal] = self.robot_self_cycle(robot)

        return graph_per_robot, distance_matrix

    @staticmethod
    def sub_path_up_to_distance(path: Path, max_distance) -> tuple[Path, float]:
        distance = 0
        new_path = [path[0]]

        for point in path[1:]:
            edge_dist = l2_norm(point, new_path[-1])
            if distance + edge_dist < max_distance:
                new_path += [point]
                distance += edge_dist
            else:
                break

        # TODO: Update bonus number of path to zero
        return Path(new_path, length=distance), distance

    def fix_path(self, robots: [Robot],
                 match: tuple[Entity],
                 distance_matrix: dict[Robot, dict[Entity, float]],
                 paths: dict[Robot, Path],
                 makespan: float) -> (dict[Robot, Path], dict[Entity, float]):
        """Receive a path longer than makespan and shorten it per robot until it fits."""
        distances = [distance_matrix[robot][bonus] for robot, bonus in zip(self.data.robots, match)]
        current_distance = sum(distances)

        if current_distance < makespan:
            return paths, dict(zip(match, distances))

        while current_distance >= makespan:
            maximum_robot = robots[np.argmax(distances)]
            allowed_distance = makespan - (current_distance - max(distances))

            new_path, new_distance = self.sub_path_up_to_distance(paths[maximum_robot], allowed_distance)
            new_path.distance_to_goal = max(distances) - new_distance

            paths[maximum_robot] = new_path
            current_distance -= (max(distances) - new_distance)
            distances[np.argmax(distances)] = new_distance

            if allowed_distance < 0:
                assert new_path == self.robot_self_cycle(maximum_robot)
                assert new_distance == 0

        return paths, dict(zip(match, distances))

    def fix_all_paths(self, all_endpoints: [Point],
                      graph_per_robot: dict[Robot, Path],
                      distance_matrix: dict[Robot, dict[Entity, float]]) -> tuple[dict, dict]:

        fixed_actions: dict[tuple[Entity], dict[Robot, Path]] = {}
        distances_per_match: dict[tuple[Entity], dict[Entity, float]] = {}

        # print('*************** All distances BEFORE fix ******************')
        # for match in all_endpoints:
        #     debug_distances = [distance_matrix[robot][bonus] for robot, bonus in zip(self.data.robots, match)]
        #     print(f'*\t {match} - {debug_distances} = {sum(debug_distances)}')
        # print('***********************************************************')

        for match in all_endpoints:
            graphs = {robot: graph_per_robot[robot][goal] if goal in graph_per_robot[robot] else Path([]) for
                      robot, goal in
                      zip(graph_per_robot, match)}

            fixed_paths, distances = self.fix_path(self.data.robots, match, distance_matrix, graphs, self.game.makespan)

            fixed_actions[match] = fixed_paths
            distances_per_match[match] = distances

        # print('*************** All distances AFTER fix ******************')
        # for match in all_endpoints:
        #     print(f'*\t {match} - {list(distances_per_match[match].values())} = {sum(distances_per_match[match].values())}')
        # print('***********************************************************')

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

    @staticmethod
    def distance_from_match(original_distances: [float], new_distances: dict[Entity, float]):
        return {entity: distance - new_distances[entity] for distance, entity in zip(original_distances, new_distances)}

    def find_best_path(self) -> dict[CoMotion_Robot, list[Segment_2]]:
        makespan = self.game.makespan
        goals: tuple[Entity] = self.data.all_entities

        profiler = Profiler()

        graph_per_robot, distance_matrix = self.create_distance_dictionary(self.data.robots, goals, makespan)

        profiler.log('distance matrix')

        all_endpoints = self.all_best_actions(goals, distance_matrix)

        profiler.log('choose all actions')

        fixed_actions: dict[tuple[Entity], dict[Robot, Path]]
        distance_from_endpath_to_bonus_per_match: dict[Entity, float]
        fixed_actions, distance_from_endpath_to_bonus_per_match = self.fix_all_paths(all_endpoints,
                                                                                     graph_per_robot,
                                                                                     distance_matrix)

        profiler.log('Fix all actions')
        for match in fixed_actions:
            my_dists = [distance_matrix[robot][goal] for robot, goal in zip(self.data.robots, match)]
            logging.debug(f'{match=} - {my_dists} = {sum(my_dists)}')

        # TODO: State and match refactor names.
        # TODO: Add options for paths in which certain robots don't move.
        # TODO: Check whether issue above makes fixing paths inutil?
        pseudo_heuristics = {match: self.upgrade_path.pseudo_heuristic_bonuses_along_match(match,
                                                                                            self.data,
                                                                                            makespan - self.match_length(
                                                                                                match,
                                                                                                self.data.robots,
                                                                                                distance_matrix),
                                                                                            distance_matrix) for match
                              in fixed_actions}
        profiler.log('Run all pseudo heuristics')

        for match in pseudo_heuristics:
            print(f'{match} - {pseudo_heuristics[match]}')

        heuristic_states = {match: self.heuristic.score(fixed_actions[match], self.turn) + pseudo_heuristics[match] * BONUS_SCORE
                            for match in
                            fixed_actions}

        for match in heuristic_states:
            print(f'{match} - {heuristic_states[match]} (Including bonus ({pseudo_heuristics[match]})')
        # print(heuristic_states)
        # print(fixed_actions)

        profiler.log('heuristic all states')

        # print('################ HEURISTIC #################')
        # for score, match in heuristic_states:
        #     distances_left = self.distance_from_match([distance_matrix[robot][bonus] for robot,
        #     bonus in zip(self.data.robots, match)] ,distance_from_endpath_to_bonus_per_match[match])
        #     print(f'{match} - {distances_left}, score - {score}')
        # print('#############################################')

        best_state = max(heuristic_states, key=lambda k: heuristic_states[k])
        best_graphs = fixed_actions[best_state]

        print(f'##### Chosen match - {best_state}')

        my_dists = [distance_matrix[robot][goal] for robot, goal in zip(self.data.robots, best_state)]
        # import ipdb;ipdb.set_trace(context=10)
        best_graphs_upgrade = self.upgrade_path.upgrade_path_for_best_robot(best_state,
                                                                           self.data,
                                                                           makespan - self.match_length(best_state,
                                                                                                        self.data.robots,
                                                                                                        distance_matrix),
                                                                           distance_matrix,
                                                                           graph_per_robot)
        if not best_graphs_upgrade:
            best_graphs_upgrade = best_graphs

        logging.debug(f'Chosen action - {best_state} with heuristic: {heuristic_states[best_state]}')

        paths = {comotion_robot: best_graphs_upgrade[robot].comotion_path for comotion_robot, robot in
                 zip(self.robots, best_graphs_upgrade)}

        self.equalise_paths_length(paths)
        profiler.log('end bullshit...')
        profiler.total()

        # print(paths)
        return paths
