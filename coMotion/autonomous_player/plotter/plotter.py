import matplotlib.pyplot as plt

from coMotion.autonomous_player.algorithms.prm import Prm


class Plotter:
    @staticmethod
    def plot_prm_graph(prm: Prm):
        plt.figure()
        plt.title(f'prm all sampled {len(prm.sampled_points)} points...')

        for point in prm.sampled_points:
            plt.scatter(point[0], point[1])

        plt.show()
