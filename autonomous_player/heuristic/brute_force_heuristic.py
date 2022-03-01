import itertools

from autonomous_player.heuristic.basic_heuristic import BonusDistanceHeuristic


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
