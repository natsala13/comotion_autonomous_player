import itertools
import numpy as np
import networkx as nx
from random import random

from autonomous_player.algorithms.prm import Prm
from autonomous_player.players.basic_player import RobotsData
from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity
from coMotion.game.comotion_entities import CoMotion_Entity, CoMotion_Bonus, CoMotion_Goal


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
    BONUS_SCORE = 10
    GOAL_SCORE = 30
    ENTITY_TO_SCORE = {Bonus: BONUS_SCORE, Goal: GOAL_SCORE}

    @staticmethod
    def decreasing_score(value: float, turns=1):
        """ As turns advance, get less score for points 'close' to goals and not on them."""
        return 1 / np.power((value + 1), turns)

    def score(self, robot_bonuses_distances: dict[Entity, float], turns=3, **kwargs):
        return sum(
            [self.decreasing_score(robot_bonuses_distances[entity], turns) * self.ENTITY_TO_SCORE[type(entity)] for
             entity in robot_bonuses_distances])


class PreProcessingHeuristic(TimeDecreasingHeuristic):
    def __init__(self, game):
        super(PreProcessingHeuristic, self).__init__(game)
        self.prm = None
        self.bonuses_graphs = None

    @staticmethod
    def diatnces_to_all_other_bonuses(distances, paths, data):
        return {bonus: distances[bonus] if bonus in distances else 1000 for bonus in data.bonuses},\
               {bonus: paths[bonus] if bonus in paths else [] for bonus in data.bonuses}

    def preprocess(self, prm: Prm):
        self.prm = prm
        data = RobotsData(self.game, None)
        all_graphs = {bonus: nx.algorithms.shortest_paths.weighted.single_source_dijkstra(prm.graph, tuple(bonus)) for
                      bonus in data.bonuses}
        self.bonuses_graphs = {bonus: self.diatnces_to_all_other_bonuses(*all_graphs[bonus], data) for bonus in
                               all_graphs}

        ###############################
        for bonus in self.bonuses_graphs:
            print(f'{bonus} - {self.bonuses_graphs[bonus]}')
        ###############################

    def add_bonus_to_path(self, all_bonuses: dict[Bonus, float], starting_location: Entity, end_bonus: Entity, max_distance: float, bonus_path: [Entity]):
        for bonus in all_bonuses:
            distance1, path = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.prm.graph,
                                                                                           starting_location,
                                                                                           target=bonus)

            distance2 = self.bonuses_graphs[bonus][0][end_bonus]
            print(f'Trying to reach bonus {bonus} with {distance1=} and {distance2=}')

            if distance1 + distance2 < max_distance and bonus not in bonus_path:
                print(f'\t Added!')
                return bonus, path

        return None, None

    def greedy_longets_path_up_to_distance(self, starting_location: Entity, end_bonus: Entity, max_distance: float,
                                           distances: dict[Bonus, float]):
        bonus_path = []
        total_path = []
        bonus = True

        print('##########################################################')
        remember_max_distance = max_distance
        remember_original_distance = distances[end_bonus]
        print(f'Trying to add bonuses to path from {starting_location} to {end_bonus}')

        while bonus is not None:
            sorted_bonuses = dict(sorted(distances.items(), key=lambda item: item[1]))

            bonus, path = self.add_bonus_to_path(sorted_bonuses, starting_location, end_bonus, max_distance, bonus_path)
            if bonus:
                bonus_path += [bonus]
                total_path += path
                max_distance -= distances[bonus]

                starting_location = bonus
                distances = self.bonuses_graphs[bonus][0]

        path_length = remember_max_distance - max_distance
        extra_length = path_length - remember_original_distance
        print(f'Added all bonuses - {bonus_path} with total extra path length of {extra_length}')
        print('############################################################')
        return bonus_path, total_path
