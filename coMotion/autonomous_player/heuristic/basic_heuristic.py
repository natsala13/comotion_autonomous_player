from random import random


class AbstractHeuristic:
    def score(self, game):
        raise NotImplementedError


class BasicHeuristic(AbstractHeuristic):
    def score(self, game):
        return random()

