import networkx as nx

from autonomous_player.algorithms.prm import Prm
from autonomous_player.utils.robot_data import RobotsData
from autonomous_player.utils.utils import Robot, Path, Bonus
from autonomous_player.heuristic.basic_heuristic import TimeDecreasingHeuristic, ENTITY_TO_SCORE


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

    def score(self, paths: dict[Robot, Path], turns=3, **kwargs):
        endpoint_score = sum(
            [self.decreasing_score(paths[robot].distance_to_goal, turns) * ENTITY_TO_SCORE[type(paths[robot].endpoint)]
             for robot in paths])
        extra_score = sum(paths[robot].bonuses for robot in paths) * ENTITY_TO_SCORE[Bonus]

        return endpoint_score + extra_score
