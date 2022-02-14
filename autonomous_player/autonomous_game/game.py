import sys
import time
import importlib
from _io import TextIOWrapper

from coMotion.read_input import read_scene
from coMotion.game_dspgl import CoMotion_DiscoPygal
from coMotion.game.comotion_game import CoMotion_Game
from coMotion.game.comotion_events import COMOTION_EVENT_NAMES
from coMotion.game.comotion_player import CoMotion_Robot, CoMotion_Player


DASH_STR = '------------------------------------'


class Game:
    def __init__(self, red_player: CoMotion_Player, blue_player: CoMotion_Player,
                 scene_config: dict, writer: TextIOWrapper = sys.stdout):
        self.writer = writer

        self.red_player: CoMotion_Player = red_player
        self.blue_player: CoMotion_Player = blue_player
        self.game = CoMotion_Game(scene_config, self.red_player, self.blue_player)

        self.game_display = None

    @classmethod
    def init_player(cls, player_config: dict, player_id: int, robots: list, robot_radius: int):
        module = importlib.import_module(player_config['module'])
        player = getattr(module, player_config['class'])(player_id, **player_config['args'])

        comotion_robots = [CoMotion_Robot(robot, robot_radius, player) for robot in robots]
        player.add_robots(comotion_robots)

        return player

    @classmethod
    def init_from_config(cls, config: dict, writer: TextIOWrapper = sys.stdout):
        scene_config = read_scene(config['scene'])

        red_player = cls.init_player(config['players'][0], 0, scene_config['red_robots'], scene_config['radius'])
        blue_player = cls.init_player(config['players'][1], 1, scene_config['blue_robots'], scene_config['radius'])

        return cls(red_player, blue_player, scene_config, writer)

    def init_game(self, gui=None):
        self.red_player.attach_game(self.game)
        self.blue_player.attach_game(self.game)

        # Preprocess the players
        print(DASH_STR, file=self.writer)
        print("Preprocessing first player...", file=self.writer)
        t0 = time.time()
        self.red_player.preprocess()
        time_taken = time.time() - t0
        print(f"Finished preprocessing first player, {time_taken=} [sec]", file=self.writer)

        print("Preprocessing second player...", file=self.writer)
        t0 = time.time()
        self.blue_player.preprocess()
        time_taken = time.time() - t0
        print(f"Finished preprocessing second player, {time_taken=} [sec]", file=self.writer)

        if gui:
            gui.clear_scene()
            game_dspgl = CoMotion_DiscoPygal(gui)
            game_dspgl.connect_game(self.game)

    def check_winner(self) -> int:
        winner = self.game.check_winner()
        message = f'Player {winner} won!' if winner > 0 else "Game ended with draw!"

        print(message, file=self.writer)
        print(DASH_STR, file=self.writer)

        return winner

    def print_turn_state(self, events):
        print(f'Player Red: score = {self.game.first_player.score}')
        print(f'Player Red: score = {self.game.second_player.score}')

        # Also print the current events
        print(DASH_STR, file=self.writer)

        print(f'### Turn {(self.game.turn - 1) // 2 + 1}, {"Red" if (self.game.turn - 1) % 2 == 0 else "Blue"}',
              file=self.writer)

        event_types = {}  # count for each type how many occurences
        for event_list in events:
            for event in event_list:
                if event.event_type not in event_types:
                    event_types[event.event_type] = 0
                event_types[event.event_type] += 1

        for event in event_types:
            print(f'\t* {COMOTION_EVENT_NAMES[event]} - {event_types[event]}', file=self.writer)

        print(events)

    def step(self, gui=None):
        if self.game.turn == self.game.max_turns * 2:
            return None, None

        paths, events = self.game.play_turn()
        if gui and paths:
            self.game_display.animate_robot_paths(paths, events)
            self.game_display.update_robot_halos(events)

        return paths, events

    def play_game(self):
        game_playing = True
        while game_playing:
            paths, events = self.step()
            if paths:
                self.print_turn_state(events)
            game_playing = paths is not None

        print('################# GAME FINISHED ################')
        self.check_winner()

    def get_first_player_score(self):
        return self.game.first_player.score


