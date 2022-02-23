import os
import yaml
import pytest

from autonomous_player.autonomous_game.game import Game


BONUS_SCORE = 10
CIRCLE_SCORE = 25
DIR_PATH = os.path.dirname(os.path.realpath(__file__))


def load_config(config_file: str) -> dict:
    if config_file.split('.')[-1] == 'yml':
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    else:
        raise ValueError(f'Invalid config file extension: {config_file.split(".")[-1]}')


def test_basic_game():
    config = {'scene': f'{DIR_PATH}/test_scenes/example_scene_1_turn.json',
              'players': [{'module': 'autonomous_player.players.focused_player', 'class': 'FocusedPlayer',
                           'args': {'num_samples': 500, 'k_nearest': 15, 'heuristic_class': 'BonusDistanceHeuristic'}},
                          {'module': 'coMotion.game.comotion_player', 'class': 'CoMotion_RandomPRMPlayer',
                           'args': {'num_landmarks': 500, 'K': 15}}]}

    game = Game.init_from_config(config)
    game.init_game()
    game.play_game()

    assert game.get_first_player_score() == BONUS_SCORE + 2 * CIRCLE_SCORE


def test_far_bonus_game():
    config = {'scene': f'{DIR_PATH}/test_scenes/far_bonus_1_robot.json',
              'players': [{'module': 'autonomous_player.players.focused_player', 'class': 'FocusedPlayer',
                           'args': {'num_samples': 500, 'k_nearest': 15, 'heuristic_class': 'BonusDistanceHeuristic'}},
                          {'module': 'coMotion.game.comotion_player', 'class': 'CoMotion_RandomPRMPlayer',
                           'args': {'num_landmarks': 500, 'K': 15}}]}

    game = Game.init_from_config(config)
    game.init_game()
    game.play_game()

    assert game.get_first_player_score() > 10


def test_greedy_2_close_bonuses():
    config = {'scene': f'{DIR_PATH}/test_scenes/two_bonuses_scene.json',
              'players': [{'module': 'autonomous_player.players.focused_player', 'class': 'FocusedPlayer',
                           'args': {'num_samples': 500, 'k_nearest': 15, 'heuristic_class': 'BonusDistanceHeuristic'}},
                          {'module': 'coMotion.game.comotion_player', 'class': 'CoMotion_RandomPRMPlayer',
                           'args': {'num_landmarks': 500, 'K': 15}}]}

    game = Game.init_from_config(config)
    game.init_game()
    game.play_game()

    assert game.get_first_player_score() == 20


def test_winning_opponent():
    config = {'scene': f'{DIR_PATH}/test_scenes/example_4robots.json',
              'players': [{'module': 'autonomous_player.players.focused_player', 'class': 'FocusedPlayer',
                           'args': {'num_samples': 1500, 'k_nearest': 15, 'heuristic_class': 'BonusDistanceHeuristic'}},
                          {'module': 'coMotion.other_players.comotion_rrt_player', 'class': 'CoMotion_RRTPlayer',
                           'args': {'max_iters': 1500, 'ETA': 1, 'cutoff': 3}}]}

    game = Game.init_from_config(config)
    game.init_game()
    game.play_game()

    assert game.get_first_player_score() > 20
