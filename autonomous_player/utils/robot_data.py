from functools import cached_property

from autonomous_player.utils.utils import Point, Segment, Bonus, Goal, Entity


class RobotsData:
    def __init__(self, game, robots):
        self.game = game
        self.my_robots = robots

    @cached_property
    def robots(self):
        return tuple((robot.location.x().to_double(), robot.location.y().to_double()) for robot in self.my_robots)  # self.game.other_player.robots

    @cached_property
    def opponent_robots(self):
        return tuple(
            (robot.location.x().to_double(), robot.location.y().to_double()) for robot in self.game.other_player.robots)

    @cached_property
    def bonuses(self) -> tuple[Bonus]:
        return tuple(Bonus(bonus.location.x().to_double(), bonus.location.y().to_double()) for bonus in self.game.bonuses if
                     not bonus.is_collected)

    @cached_property
    def end_circles(self) -> tuple[Goal]:
        return tuple(Goal(bonus.location.x().to_double(), bonus.location.y().to_double()) for bonus in self.game.goals)
        # If end circle is free ?

    @cached_property
    def all_entities(self) -> tuple[Entity]:
        return self.bonuses + self.end_circles
