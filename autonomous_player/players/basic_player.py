import logging

from bindings import Segment_2, Point_2
from coMotion.game.comotion_player import CoMotion_Player
import geometry_utils.collision_detection as collision_detection

from autonomous_player.utils import utils
from autonomous_player.algorithms.prm import Prm
from autonomous_player.plotter import plotter
from autonomous_player.heuristic import basic_heuristic
from autonomous_player.utils.robot_data import RobotsData
from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity


class BasicPlayer(CoMotion_Player):
    """
    A player that plays random PRM for its robots
    This is meant only as a test/example to how true logic should work.
    """

    def __init__(self, player_id, num_samples, k_nearest=15, heuristic_class='BonusDistanceHeuristic'):
        super().__init__(player_id)

        # Store a (static) probabilistic roadmap of the scene
        # Also all robot locations are vertices in the roadmap
        self.prm = None
        self.k_nearest = int(k_nearest)
        self.num_landmarks = int(num_samples)
        # game is inited once attach game is called. I would prefer it here on init...

        self.heuristic_class = getattr(basic_heuristic, heuristic_class)
        self.heuristic = None

        self.data: RobotsData = None
        self.turn = 0

    @property
    def turns_left(self):
        return self.game.max_turns - self.turn

    def preprocess(self):
        self.heuristic = self.heuristic_class(self.game)
        obstacles_cd = collision_detection.Collision_detector(self.game.obstacles, [], self.game.radius)
        self.prm = Prm(self.game.obstacles, self.robots, self.k_nearest, utils.l2_norm, obstacles_cd)

        self.data = RobotsData(self.game, self.robots)
        self.prm.create_graph(self.num_landmarks, fix_points=self.data.bonuses + self.data.end_circles)

    @staticmethod
    def point_list_to_segment_list(path: [Point]) -> [Segment_2]:
        if len(path) == 1:
            return [Segment_2(Point_2(*path[0]), Point_2(*path[0]))]
        return [Segment_2(Point_2(*a), Point_2(*b)) for a, b in zip(path[:-1], path[1:])]

    @staticmethod
    def equalise_paths_length(paths: dict) -> None:
        """Duplicate last node of all shortest paths so they would all have the same length"""
        max_len = max([len(paths[robot]) for robot in paths])
        for robot in paths:
            if len(paths[robot]) < max_len:
                paths[robot] += [Segment_2(paths[robot][-1][1], paths[robot][-1][1])] * (max_len - len(paths[robot]))

    def preprocess_turn(self) -> None:
        """Preprocess all information before turn and return 2 list of robots"""
        self.turn += 1
        self.data = RobotsData(self.game, self.robots)

        self.prm.add_team_robots_location(self.data.robots)
        self.prm.remove_opponent_robots_location(self.data.opponent_robots)

    def postprocess_turn(self) -> None:
        self.prm.re_add_opponent_robots_locations()

    def find_best_path(self):
        best_paths = self.prm.find_all_couple_of_paths(self.data.robots, self.game.makespan)
        heuristics = {node: self.heuristic.score(node) for node in best_paths}
        sorted_nodes = sorted(heuristics, key=lambda n: heuristics[n])

        plotter.plot_prm_heat_map(heuristics)

        best_node = sorted_nodes[0]
        paths = {robot: self.point_list_to_segment_list(path) for robot, path in
                 zip(self.robots[:2], best_paths[best_node])}

        paths[self.robots[2]] = [Segment_2(self.robots[2].location, self.robots[2].location)]
        self.equalise_paths_length(paths)

        return paths

    def get_turn_robot_paths(self) -> dict:
        """Apparently I should return a dict of paths..."""
        self.preprocess_turn()

        paths = self.find_best_path()

        self.postprocess_turn()

        logging.debug('################ POST PROCESS #################')
        logging.debug(paths)
        logging.debug('###############################################')

        return paths
