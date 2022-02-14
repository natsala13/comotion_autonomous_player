import yaml
import pytest

from autonomous_player.autonomous_game.game import Game


def load_config(config_file: str) -> dict:
    if config_file.split('.')[-1] == 'yml':
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    else:
        raise ValueError(f'Invalid config file extension: {config_file.split(".")[-1]}')


def test_greedy_2_close_bonuses():
    config = {'scene': '/home/nathan/tau/robotics/final_project/coMotion/coMotion/scenes/two_bonuses_scene.json',
              'players': [{'module': 'autonomous_player.players.focused_player', 'class': 'FocusedPlayer',
                           'args': {'num_samples': 1500, 'k_nearest': 15, 'heuristic_class': 'BonusDistanceHeuristic'}},
                          {'module': 'coMotion.game.comotion_player', 'class': 'CoMotion_RandomPRMPlayer',
                           'args': {'num_landmarks': 1000, 'K': 15}}]}

    game = Game.init_from_config(config)
    game.init_game()
    game.play_game()

    assert game.get_first_player_score() == 20
