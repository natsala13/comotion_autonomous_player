from random import random
import numpy as np


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

    def score(self, robots_location):
        min_distances = [min([self.distance(robot, bonus) for robot in robots_location]) for bonus in self.bonuses]
        return min(min_distances)

    def update_before_turn(self):
        self.bonuses = tuple(
            (bonus.location.x().to_double(), bonus.location.y().to_double()) for bonus in self.game.bonuses if
            not bonus.is_collected)

