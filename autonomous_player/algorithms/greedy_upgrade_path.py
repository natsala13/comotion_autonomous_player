import logging
import networkx as nx
from typing import Any

from autonomous_player.utils.robot_data import RobotsData
from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity, l2_norm, Robot, Path


class UpgradePath:
    def __init__(self, prm, bonuses_distances):
        self.prm = prm
        self.bonuses_distances = bonuses_distances

    def add_bonus_to_path(self, all_bonuses: [Bonus], starting_location: Entity, end_bonus: Entity,
                          max_distance: float, bonus_path: [Entity]):
        for bonus in all_bonuses[:3]:
            if bonus in bonus_path:
                continue

            if bonus not in self.prm.graph:
                continue

            # profiler.log('Start loop')
            try:
                distance1, path = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.prm.graph,
                                                                                               tuple(starting_location),
                                                                                               target=tuple(bonus))
            except nx.exception.NetworkXNoPath:
                # Mean there is no path to that robot, lets check the next one...
                continue

            distance2 = self.bonuses_distances[bonus][end_bonus]

            if distance1 + distance2 < max_distance and bonus not in bonus_path:
                return bonus, path

        return None, None

    def pseudo_heuristic_bonuses_along_path(self, end_bonus: Entity,
                                            original_distance: float,
                                            left_distance: float,
                                            data: RobotsData,
                                            distances: dict[Bonus, float]) -> int:
        """Count all bonuses that are in an ellipsoid around start and end point"""
        deltas_from_path = []

        for bonus in data.bonuses:
            distance1 = distances[bonus]
            distance2 = self.bonuses_distances[bonus][end_bonus]

            if distance1 + distance2 <= original_distance + left_distance:
                deltas_from_path += [distance1 + distance2 - original_distance]

        return len(distances)  # TODO: Better heuristic

    def greedy_add_bonuses_up_to_distance(self, starting_location: Entity,
                                          end_bonus: Entity,
                                          original_distance: float,
                                          left_distance: float,
                                          distances: dict[Bonus, float],
                                          robot_graphs: dict[Bonus, [Point]]) -> Path:
        """Scan all bonuses from the closest one and next and add every one of them if possible."""
        self.bonuses_distances[starting_location] = distances

        # profiler = Profiler(tabs=2)

        bonus_path = [end_bonus, starting_location]
        total_path = []
        bonus = True

        path_distance = 0

        logging.info(f'Trying to add bonuses to path from {starting_location} to {end_bonus}...')

        while bonus is not None:
            # profiler.log('start loop')

            sorted_bonuses = [d[0] for d in sorted(distances.items(), key=lambda item: item[1])]
            # profiler.log('sprt bonuses')

            bonus, path = self.add_bonus_to_path(sorted_bonuses,
                                                 starting_location,
                                                 end_bonus,
                                                 original_distance + left_distance - path_distance,
                                                 bonus_path)
            # profiler.log('add bonus to path')

            if bonus:
                bonus_path += [bonus]
                total_path += path
                path_distance += distances[bonus]

                starting_location = bonus
                distances = {b: self.bonuses_distances[bonus][b] for b in self.bonuses_distances[bonus] if
                             b in distances}
                logging.info(f'Added Bonus {bonus} to path!')

            # profiler.log('added bonus to path')

        last_dist, path = nx.algorithms.shortest_paths.weighted.single_source_dijkstra(self.prm.graph,
                                                                                       tuple(bonus_path[-1]),
                                                                                       target=tuple(end_bonus))

        total_path += path
        bonus_path += [end_bonus]
        path_distance += last_dist

        return Path(total_path, length=path_distance, extra_bonuses=len(bonus_path) - 3,
                    endpoint=end_bonus, added_distance=path_distance - original_distance)

    @staticmethod
    def sum_bonuses_from_robots(robots: [Robot], all_fixes: dict[Robot, Path]) -> int:
        return sum([all_fixes[robot].bonuses for robot in robots])

    def best_robots_to_fix(self, robots: [Robot],
                           all_fixes: dict[Robot, Path],
                           maximum_path_length: float) -> [Robot]:
        if not robots:
            return []

        current_robot = robots[0]
        current_path_delta = all_fixes[current_robot].added_distance
        with_robot = []

        if current_path_delta < maximum_path_length:
            with_robot = [current_robot] + self.best_robots_to_fix(robots[1:], all_fixes,
                                                                   maximum_path_length - current_path_delta)

        without_robot = self.best_robots_to_fix(robots[1:], all_fixes, maximum_path_length)

        if self.sum_bonuses_from_robots(without_robot, all_fixes) >= self.sum_bonuses_from_robots(with_robot,
                                                                                                  all_fixes):
            return without_robot
        else:
            return with_robot

    def upgrade_path_for_best_robot(self, match: [Entity],
                                    data: RobotsData,
                                    left_distance: float,
                                    distances: dict[Robot, dict[Entity, float]],
                                    graphs: dict[tuple, dict[Entity, Path]]) -> (Robot, [Point], int):
        if left_distance <= 0:
            return {}

        all_fixes = {robot: self.greedy_add_bonuses_up_to_distance(robot, robot_match,
                                                                   distances[robot][robot_match],
                                                                   left_distance,
                                                                   {b: distances[robot][b] for b in data.bonuses},
                                                                   graphs[robot]) for robot, robot_match in
                     zip(data.robots, match)}

        all_fixes = {robot: all_fixes[robot] for robot in all_fixes if all_fixes[robot].bonuses > 0}
        best_robots = self.best_robots_to_fix(list(all_fixes.keys()), all_fixes, left_distance)

        if all_fixes:
            return {robot: all_fixes[robot] for robot in best_robots}
        else:
            return {}
