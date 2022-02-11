import itertools
import numpy as np
from random import random

from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity
from coMotion.game.comotion_entities import CoMotion_Entity, CoMotion_Bonus, CoMotion_Goal

class AbstractHeuristic:
    def __init__(self, game):
        self.game = game

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


class BonusSmartDistanceHeuristic(BonusDistanceHeuristic):
    """This method takes a too big amount of time"""
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


class BonusAndCirclesDistanceHeuristic(BonusDistanceHeuristic):
    BONUS_SCORE = 10
    GOAL_SCORE = 30
    ENTITY_TO_SCORE = {Bonus: BONUS_SCORE, Goal: GOAL_SCORE}

    @staticmethod
    def decresing_score(value: float, turns=1):
        return 1 / np.power((value + 1), turns)

    def score(self, robot_bonuses_distances: dict[Entity, float], turns=3, **kwargs):
        return sum([self.decresing_score(robot_bonuses_distances[entity], turns) * self.ENTITY_TO_SCORE[type(entity)] for entity in robot_bonuses_distances])
