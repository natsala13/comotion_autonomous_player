import random

import numpy as np
import networkx as nx
import sklearn.neighbors

from prm import Prm
from bindings import Segment_2
from coMotion.game.comotion_player import CoMotion_Player
import geometry_utils.collision_detection as collision_detection


class BasicPlayer(CoMotion_Player):
    """
    A player that plays random PRM for its robots
    This is meant only as a test/example to how true logic should work.
    """

    def __init__(self, player_id, num_landmarks, k_nearest):
        super().__init__(player_id)

        # Store a (static) probabilistic roadmap of the scene
        # Also all robot locations are vertices in the roadmap
        self.prm = Prm(self.game, self.robots, k_nearest)
        self.k_nearest = int(k_nearest)
        self.num_landmarks = int(num_landmarks)

    def preprocess(self):
        self.prm.create_graph(self.num_landmarks)

    def advance_one_step(self):
        # Advance all robots in one random step, and return the total makespan
        makespan = 0
        steps = {}

        for robot in self.robots:
            neighbors = self.prm[robot.location]  # all possible next steps.
            random_next = random.choice(list(neighbors.keys()))  # choose one randomally

            weight = neighbors[random_next]['weight']  # next random step weight.
            edge = Segment_2(robot.location, random_next)  # corresponding edge.

            # All chosen paths from before plus the new one.
            parallel_edges = [edge] + [steps[other_robot] for other_robot in steps]

            other_team_edges = [Segment_2(oponent_robot.location, oponent_robot.location) for oponent_robot in
                                self.game.other_player.robots]  # oponent player robot new locations.

            total_robot_edges = parallel_edges + other_team_edges
            if collision_detection.check_intersection_against_robots(total_robot_edges,
                                                                     [robot.radius] * len(total_robot_edges)):
                print('Intersection between robots detected...')
                steps[robot] = Segment_2(robot.location, robot.location)  # stay in place
            else:
                steps[robot] = edge
                makespan += weight
                robot.location = random_next  # Typically dangerous, but we return all robots back to place in the end

        return steps, makespan

    def get_turn_robot_paths(self):
        total_makespan = 0
        paths = {robot: [] for robot in self.robots}

        original_locations = {robot: robot.location for robot in self.robots}

        while True:
            steps, makespan = self.advance_one_step()
            if total_makespan + makespan >= self.game.makespan:
                break
            total_makespan += makespan
            for robot in steps:
                paths[robot].append(steps[robot])

        # restore original locations
        for robot in original_locations:
            robot.location = original_locations[robot]

        return paths