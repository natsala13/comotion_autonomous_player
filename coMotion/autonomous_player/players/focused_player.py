import time
import itertools

import numpy as np
import networkx as nx

from bindings import Segment_2, Point_2
from coMotion.game.comotion_player import CoMotion_Robot
from coMotion.autonomous_player.utils.utils import Point, Segment
from coMotion.autonomous_player.players.basic_player import BasicPlayer
from coMotion.autonomous_player.heuristic.basic_heuristic import BonusAndCirclesDistanceHeuristic


class FocusedPlayer(BasicPlayer):

    def __init__(self, player_id, num_samples, k_nearest=15, **kwargs):
        super(FocusedPlayer, self).__init__(player_id, num_samples, k_nearest, 'BonusAndCirclesDistanceHeuristic')

    def create_ditance_matrix(self, robots, goals, max_distance):
        distance_matrix = np.zeros((len(robots), len(goals))) + 100
        graph_per_robot = {robot: {} for robot in robots}

        for robot_i, robot in enumerate(robots):
            distances, paths = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.prm.graph,
                                                                                            robot,
                                                                                            cutoff=max_distance)
            for goal_i, goal in enumerate(goals):
                if goal in distances:
                    distance_matrix[robot_i][goal_i] = distances[goal]
                    graph_per_robot[robot][goal] = paths[goal]

            if graph_per_robot[robot] == {}:
                graph_per_robot[robot] = {'self cycle': [Segment(robot, robot).comotion_segment]}

        return graph_per_robot, distance_matrix

    def find_best_path(self) -> dict[CoMotion_Robot, list[Segment_2]]:
        # TODO: Is it shorter using dictionaries instead of arrays?
        makespan = self.game.makespan
        goals = self.data.bonuses + self.data.end_circles
        all_endpoints = list(itertools.permutations(np.arange(len(goals)), len(self.data.robots)))

        graph_per_robot, distance_matrix = self.create_ditance_matrix(self.data.robots, goals, makespan)

        # This represents the best state to be in - list associating every robot to a goal index
        sum_states = [sum([distance_matrix[robot_i][bonus_j] for robot_i, bonus_j in enumerate(endpoint_state)]) for
                      endpoint_state in all_endpoints]

        ###### Debug #########
        # print(f'{makespan=}')
        # all_states = [(state_sum, state) for state_sum, state in zip(sum_states, all_endpoints)]
        # sorted_states = sorted(all_states)
        # print('All best states to be at...')
        # print(sorted_states[:10])
        ######################

        valid_states = [(state_sum, state) for state_sum, state in zip(sum_states, all_endpoints) if
                        state_sum < makespan]
        # TODO: remove statue_sum I don't want that except knowing whether or not the state is valid
        # TODO: Fix all invalid states ? Maybe it would be a shame to let them go.
        # TODO: heuristic every state and chose the maximum one
        # TODO: heuristic should take in account the goal's type the robot's distance from here and number of turns left


        min_state = min(valid_states)[1]
        min_goal_list = [goals[goal_i] for goal_i in min_state]

        paths = {robot: graph_per_robot[robot][min_goal_list[robot_i]] for robot_i, robot in
                 enumerate(self.data.robots)}
        paths = {comotion_robot: self.point_list_to_segment_list(paths[robot]) for comotion_robot, robot in
                 zip(self.robots, paths)}
        self.equalise_paths_length(paths)

        return paths
