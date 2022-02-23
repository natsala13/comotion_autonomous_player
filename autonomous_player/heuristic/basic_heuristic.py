import logging
import itertools
import numpy as np
import networkx as nx
from random import random

from autonomous_player.plotter import plotter
from autonomous_player.algorithms.prm import Prm
from autonomous_player.utils.profiler import Profiler
from autonomous_player.utils.robot_data import RobotsData
from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity, l2_norm, Robot, Path
from coMotion.game.comotion_entities import CoMotion_Entity, CoMotion_Bonus, CoMotion_Goal

BONUS_SCORE = 10
GOAL_SCORE = 25
ENTITY_TO_SCORE = {Bonus: BONUS_SCORE, Goal: GOAL_SCORE, type(None): BONUS_SCORE}

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
        logging.debug('############# BONUS Distances #################')
        for bonus in self.bonuses_distances:
            logging.debug(f'{bonus} - {self.bonuses_distances[bonus]}')
        logging.debug('###############################################')

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

    def score(self, robot_bonuses_distances: dict[Entity, float], turns=3, **kwargs):
        return sum(
            [self.decreasing_score(robot_bonuses_distances[entity], turns) * ENTITY_TO_SCORE[type(entity)] for
             entity in robot_bonuses_distances])

    def score2(self, paths: dict[Robot, Path], turns=3, **kwargs):
        endpoint_score = sum(
            [self.decreasing_score(paths[robot].distance_to_goal, turns) * ENTITY_TO_SCORE[type(paths[robot].endpoint)]
             for robot in paths])
        extra_score = sum(paths[robot].bonuses for robot in paths) * ENTITY_TO_SCORE[Bonus]

        return endpoint_score + extra_score
