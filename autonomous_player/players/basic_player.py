import logging

from bindings import Segment_2, Point_2, FT
from coMotion.game.comotion_player import CoMotion_Player
import geometry_utils.collision_detection as collision_detection

from autonomous_player.utils import utils
from autonomous_player.algorithms.prm import Prm
from autonomous_player.plotter import plotter
from autonomous_player.heuristic.basic_heuristic import BonusDistanceHeuristic
from autonomous_player.utils.robot_data import RobotsData
from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity, Robot


class BasicPlayer(CoMotion_Player):
    """
    A player that plays random PRM for its robots
    This is meant only as a test/example to how true logic should work.
    """

    def __init__(self, player_id, num_samples=1500, k_nearest=15, heuristic_class=BonusDistanceHeuristic):
        super().__init__(player_id)

        # Store a (static) probabilistic roadmap of the scene
        # Also all robot locations are vertices in the roadmap
        self.prm = None
        self.k_nearest = int(k_nearest)
        self.num_landmarks = int(num_samples)
        # game is inited once attach game is called. I would prefer it here on init...

        self.heuristic_class = heuristic_class
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

    @staticmethod
    def is_comotion_edge_static(edge):
        return edge[0] == edge[1]

    def fix_robots_collision(self, paths: dict) -> bool:
        for robot in paths:
            path = paths[robot]

            for i, edge in enumerate(path):
                if self.is_comotion_edge_static(edge):
                    continue

                team_edges = [paths[team_robot][i] for team_robot in paths]

                if collision_detection.check_intersection_against_robots(team_edges, [FT(1.0)] * len(team_edges)):
                    print('Collision Detected - Reparation fase 1')
                    team_robots_not_moving = [Segment_2(paths[team_robot][i][0], paths[team_robot][i][0]) for team_robot
                                              in paths]
                    # if collision_detection.check_intersection_against_robots(team_robots_not_moving, [FT(1.0)] * len(team_robots_not_moving)):
                    print(
                        f'Collision Detected - Reparation fase 2 - Trimming path {Robot.robot_from_comotion(robot)} at edge {i} - {edge}')
                    static_location = Segment_2(edge[0], edge[0])

                    paths[robot] = path[:i] + [static_location] * (len(path) - i)
                    # else:
                    #     print(f'Collision Detected at {robot=} at segment {i} - {edge} - Reparation fase 1 possible - adding empty segment')
                    #     for other_robot in paths:
                    #         if other_robot != robot:
                    #             static_location = Segment_2(paths[other_robot][i][0], paths[other_robot][i][0])
                    #             paths[other_robot] = paths[other_robot][:i] + [static_location] + paths[other_robot][i:]
                    #         else:
                    #             static_location = Segment_2(paths[robot][i][0], paths[robot][i][0])
                    #             paths[robot] = paths[robot][:i+1] + [static_location] + paths[robot][i+1:]
                    #     print(paths)
                    #     # import ipdb;ipdb.set_trace()

                    return False

        return True

    def fix_team_collision_robots(self, paths):
        path_valid = False
        counter = 0
        while not path_valid:
            path_valid = self.fix_robots_collision(paths)
            counter += 1

    def get_turn_robot_paths(self) -> dict:
        """Apparently I should return a dict of paths..."""
        self.preprocess_turn()

        paths = self.find_best_path()
        self.fix_team_collision_robots(paths)

        self.postprocess_turn()

        return paths
