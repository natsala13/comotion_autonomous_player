import numpy as np
import matplotlib.pyplot as plt

from coMotion.autonomous_player.algorithms.prm import Prm
from coMotion.autonomous_player.utils.utils import Point, Segment


def normalise_values(values: list, max_range=256):
    delta = max_range / (max(values) - min(values))
    min_value = min(values)
    return [int((val - min_value) * delta) for val in values]


class Plotter:
    @staticmethod
    def plot_prm_graph(prm: Prm, show: bool = True, colors=None):
        plt.figure()
        plt.title(f'prm all sampled {len(prm.sampled_points)} points...')
        print(f'prm all sampled {len(prm.sampled_points)} points...')

        x = [p[0] for p in prm.sampled_points]
        y = [p[1] for p in prm.sampled_points]

        colors = [np.log(c) for c in colors]
        colors = normalise_values(colors)
        plt.scatter(x, y, c=colors, cmap='gist_rainbow')

        # for point in prm.sampled_points:
        #     plt.scatter(point[0], point[1])

        if show:
            plt.colorbar()
            plt.show()

        plt.figure()
        plt.hist(colors, bins=50)
        plt.show()

    @staticmethod
    def plot_dijkestra_path(paths: list, show: bool = True):
        # all_nodes = {item for sublist in paths for item in sublist}
        for node in paths:
            plt.scatter(node[0], node[1], marker='x', color='red')

        if show:
            plt.show()

    @staticmethod
    def plot_prm_heat_map(paths: dict[(Point, Point), float]):
        def drop_redundant_values(values: dict):
            # return {p[0]: np.mean([values[g] for g in values if p[0] == g[0]]) for p in values}
            new_values = {}
            for node in values:
                if node[0] not in new_values:
                    new_values[node[0]] = np.mean([values[g] for g in values if g[0] == node[0]])
            return new_values

        print('Scatter all heuristic paths')
        paths = drop_redundant_values(paths)

        colors = [np.log(c) for c in paths.values()]
        colors = normalise_values(colors)
        alphas = [x + (1-x)/5 for x in normalise_values(list(paths.values()), max_range=1)]

        x = [node[0] for node in paths]
        y = [node[1] for node in paths]
        plt.scatter(x, y, c=colors, alpha=alphas, cmap='gist_rainbow')
        # for node, heuristic in zip(paths, normalised_heuristics):
        #     print(f'point x - {node[0]}, y - {node[1]}, color - {heuristic}')
        #     plt.scatter(node[0], node[1], c=random.uniform(0, 256), cmap='prism')

        plt.colorbar()
        plt.show()


