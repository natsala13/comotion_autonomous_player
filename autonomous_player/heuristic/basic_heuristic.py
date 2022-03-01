import logging
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
