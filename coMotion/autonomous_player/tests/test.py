"""
Usage:
    experiment.py test [-s] [--filename=test] [--config=CONFIGFILE]
    experiment.py graph [--config=CONFIGFILE] [--samples=NUM]
    experiment.py run [--config=CONFIGFILE] [--samples=NUM]

Options:
    -p --plot                       Present graphs from saved json run files.
    -f --filename=TEST              output filename [default: test]
    -r --results=FILENAME           results filename or part of a name if only one match.
    --samples=NUM                   Number of samples to use. [default: 300]
    --scene=FILENAME                Scene filename.
    --config=CONFIGFILE             Config file path [default: "experiment.json"]
"""
import yaml
from docopt import docopt

from coMotion.read_input import read_scene
from coMotion.autonomous_player.autonomous_game.game import Game
from coMotion.autonomous_player.algorithms.prm import Prm
from coMotion.autonomous_player.plotter.plotter import Plotter
from coMotion.autonomous_player.heuristic.basic_heuristic import BonusDistanceHeuristic
import geometry_utils.collision_detection as collision_detection


def load_config(config_file: str) -> dict:
    if config_file.split('.')[-1] == 'yml':
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    else:
        raise ValueError(f'Invalid config file extension: {config_file.split(".")[-1]}')


def run_test_with_config(config):
    raise NotImplementedError('Run tests not implemented yet...')


def run_and_plot_prm(config, smaples):
    scene_config = read_scene(config['scene'])
    obstacles = scene_config['obstacles']
    radius = scene_config['radius']
    obstacles_cd = collision_detection.Collision_detector(obstacles, [], radius)

    prm = Prm(obstacles, scene_config['blue_robots'], 3, Prm.l2_norm, obstacles_cd)
    prm.create_graph(smaples)

    game = Game.init_from_config(config)
    heuristic = BonusDistanceHeuristic(game.game)
    colors = [heuristic.score([p]) for p in prm.sampled_points]

    plotter = Plotter()
    plotter.plot_prm_graph(prm, show=True, colors=colors)

    # first_node = list(prm)[0]
    # distances, paths = prm.find_all_possible_configuration_up_to_length(first_node)
    # plotter.plot_dijkestra_path(paths)


def single_run(config, smaples=300):
    game = Game.init_from_config(config)
    game.init_game()
    game.play_game()


if __name__ == "__main__":
    args = docopt(__doc__)
    configurations = load_config(args['--config'])

    if args['test']:
        run_test_with_config(configurations)
    if args['graph']:
        run_and_plot_prm(configurations, int(args['--samples']))
    if args['run']:
        # import ipdb;ipdb.set_trace()
        # scene_filenmae = f'coMotion/scenes/{args["--scene"]}'
        single_run(configurations, int(args['--samples']))
