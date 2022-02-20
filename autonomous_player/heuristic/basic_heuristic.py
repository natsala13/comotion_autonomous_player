import itertools
import numpy as np
import networkx as nx
from random import random

from autonomous_player.algorithms.prm import Prm
from autonomous_player.utils.profiler import Profiler
from autonomous_player.utils.robot_data import RobotsData
from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity, l2_norm, Robot
from coMotion.game.comotion_entities import CoMotion_Entity, CoMotion_Bonus, CoMotion_Goal

BONUS_SCORE = 10
GOAL_SCORE = 25
ENTITY_TO_SCORE = {Bonus: BONUS_SCORE, Goal: GOAL_SCORE}

EPSILON = 0.1

class AbstractHeuristic:
    def __init__(self, game):
        self.game = game

    def preprocess(self, *args, **kwargs):
        pass

    def update_before_turn(self):
        pass

    def score(self, robots_location):
        raise NotImplementedError


class RandomHeuristic(AbstractHeuristic):
    def score(self, robots_location):
        return random()


class BonusDistanceHeuristic(AbstractHeuristic):
    def __init__(self, game):
        super().__init__(game)
        self.bonuses = None
        self.update_before_turn()

    @staticmethod
    def distance(p0, p1):
        return np.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2)

    def score(self, robots_location, **kwargs):
        min_distances = [min([self.distance(robot, bonus) for robot in robots_location]) for bonus in self.bonuses]
        return sum(min_distances)

    def update_before_turn(self):
        self.bonuses = tuple(
            (bonus.location.x().to_double(), bonus.location.y().to_double()) for bonus in self.game.bonuses if
            not bonus.is_collected)


class BruteForceHeuristic(BonusDistanceHeuristic):
    """This method reveive a list of robots and bonuses, and calculates all exponential possibilities to match
        This method takes a too big amount of time
    """

    def brute_search(self, distances):
        if not distances:
            return 0

        min_score = 1000
        for i, robot_bonus in enumerate(distances[0]):
            my_score = self.brute_search(
                [[r for j, r in enumerate(bonus) if j != i] for bonus in distances[1:]]) + robot_bonus
            if my_score < min_score:
                min_score = my_score

        return min_score

    def pseudo_closest_match(self, robots, bonuses):
        len_permutations = min(len(bonuses), len(robots))
        return min([sum([self.distance(robot, bonus) for robot, bonus in zip(robots, bonus_permutations)]) for
                    bonus_permutations in itertools.permutations(bonuses, len_permutations)])

    def score(self, robots_location, **kwargs):
        return self.pseudo_closest_match(robots_location, self.bonuses)


class TimeDecreasingHeuristic(BonusDistanceHeuristic):
    """ This heuristic receives a list of matches between robots and bonuses and calculate distances times some factors.
    """

    @staticmethod
    def decreasing_score(value: float, turns=1):
        """ As turns advance, get less score for points 'close' to goals and not on them."""
        # return 1 / np.power((value + 1), turns)
        return 1 / (value + 1)

    def score(self, robot_bonuses_distances: dict[Entity, float], turns=3, **kwargs):
        return sum(
            [self.decreasing_score(robot_bonuses_distances[entity], turns) * ENTITY_TO_SCORE[type(entity)] for
             entity in robot_bonuses_distances])


class PreProcessingHeuristic(TimeDecreasingHeuristic):
    def __init__(self, game):
        super(PreProcessingHeuristic, self).__init__(game)
        self.prm = None
        self.bonuses_distances = None
        self.bonuses_graphs = None

    @staticmethod
    def distances_to_all_other_bonuses(distances, data):
        return {bonus: distances[tuple(bonus)] if tuple(bonus) in distances else 1000 for bonus in data.all_entities}

    @staticmethod
    def paths_to_all_other_bonuses(paths, data):
        return {bonus: paths[tuple(bonus)] if tuple(bonus) in paths else [] for bonus in data.all_entities}

    def print_distances(self):
        print('############# BONUS Distances #################')
        for bonus in self.bonuses_distances:
            print(f'{bonus} - {self.bonuses_distances[bonus]}')
        print('###############################################')

    def preprocess(self, prm: Prm) -> None:
        self.prm = prm
        data = RobotsData(self.game, None)
        all_graphs = {bonus: nx.algorithms.shortest_paths.weighted.single_source_dijkstra(prm.graph, tuple(bonus))
                      for
                      bonus in data.all_entities}

        self.bonuses_distances = {bonus: self.distances_to_all_other_bonuses(all_graphs[bonus][0], data) for bonus in
                                  all_graphs}

        self.bonuses_graphs = {bonus: self.paths_to_all_other_bonuses(all_graphs[bonus][1], data) for bonus in
                               all_graphs}

        self.print_distances()

    def add_bonus_to_path(self, all_bonuses: dict[Bonus, float], starting_location: Entity, end_bonus: Entity,
                          max_distance: float, bonus_path: [Entity]):
        for bonus in all_bonuses:
            if bonus in bonus_path:
                continue

            # profiler.log('Start loop')
            distance1 = self.bonuses_distances[starting_location][tuple(bonus)]
            path = self.bonuses_graphs[starting_location][tuple(bonus)]
            #TODO: distance1 and 2 are only heuristic! I should change them to real paths.

            # distance1, path = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.prm.graph,
            #                                                                                tuple(starting_location),
            #                                                                                target=tuple(bonus))
            # profiler.log('Dijkestra')
            try:
                distance2 = self.bonuses_distances[bonus][end_bonus]
            except KeyError:
                import ipdb;ipdb.set_trace()
            # print(f'Trying to reach bonus {bonus} with {distance1=} and {distance2=}')

            # import ipdb;ipdb.set_trace()
            print(f'Trying to add {bonus} when {distance1=} and {distance2=} with total of {distance1 + distance2}')
            if distance1 + distance2 < max_distance and bonus not in bonus_path:
                # print(f'\t Added!')
                return bonus, path

        return None, None

    def greedy_longets_path_up_to_distance(self, starting_location: Entity, end_bonus: Entity, max_distance: float,
                                           distances: dict[Bonus, float], robot_graphs: dict[Bonus, [Point]]):
        self.bonuses_distances[starting_location] = distances
        self.bonuses_graphs[starting_location] = robot_graphs

        # profiler = Profiler(tabs=2)

        bonus_path = [end_bonus, starting_location]
        total_path = []
        bonus = True

        print('##########################################################')
        remember_max_distance = max_distance
        remember_original_distance = distances[end_bonus]
        print(f'Trying to add bonuses to path from {starting_location} to {end_bonus} with {max_distance=}')

        while bonus is not None:
            # profiler.log('start loop')
            sorted_bonuses = list(dict(sorted(distances.items(), key=lambda item: item[1])).keys())
            # profiler.log('sprt bonuses')

            bonus, path = self.add_bonus_to_path(sorted_bonuses, starting_location, end_bonus, max_distance, bonus_path)
            # profiler.log('add bonus to path')

            if bonus:
                bonus_path += [bonus]
                total_path += path
                max_distance -= distances[bonus]

                starting_location = bonus
                distances = self.bonuses_distances[bonus]
                print(f'Added Bonus {bonus} to path!')

            # profiler.log('added bonus to path')

        path_length = remember_max_distance - max_distance
        extra_length = path_length - remember_original_distance
        print(f'Added all bonuses - {bonus_path[2:]} with total extra path length of {extra_length}')
        print('############################################################')

        # _, path = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.prm.graph,
        #                                                                        tuple(bonus_path[-1]),
        #                                                                        target=tuple(end_bonus))
        path = self.bonuses_graphs[bonus_path[-1]][tuple(end_bonus)]

        total_path += path
        bonus_path += [end_bonus]

        return total_path, len(bonus_path) - 3

    def upgrade_path_for_best_robot(self, match: [Entity],
                                    data: RobotsData,
                                    max_distance: float,
                                    distances: dict[tuple, dict[Entity, float]],
                                    graphs: dict[tuple, dict[Entity, [Point]]]) -> (Robot, [Point], int):
        all_fixes = {robot: self.greedy_longets_path_up_to_distance(robot, robot_match,
                                                                    max_distance + distances[robot][robot_match],
                                                                    {b: distances[robot][b] for b in data.all_entities  },
                                                                    graphs[robot]) for robot, robot_match in
                     zip(data.robots, match) if max_distance > 0}

        all_fixes = {robot: all_fixes[robot] for robot in all_fixes if all_fixes[robot][1] > 0}

        if all_fixes:
            best_robot = max(all_fixes, key=lambda x: all_fixes[x][1])
            return best_robot, *all_fixes[best_robot]
        else:
            return None, [], 0

    def score(self, robot_bonuses_distances: dict[Entity, float], turns=3, **kwargs):
        return sum(
            [self.decreasing_score(robot_bonuses_distances[entity], turns) * ENTITY_TO_SCORE[type(entity)] for
             entity in robot_bonuses_distances])