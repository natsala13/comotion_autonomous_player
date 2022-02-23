from bindings import Point_2, Segment_2
from typing import Union


class Point:
    def __init__(self, *dimensions: float):
        self.dimensions = dimensions

    def __getitem__(self, item):
        return self.dimensions[item]

    def __iter__(self):
        return self.dimensions.__iter__()

    def __repr__(self):
        return f'{type(self).__name__}{tuple(val for val in self.dimensions)}'

    def __eq__(self, other):
        return tuple(self) == tuple(other)

    def __hash__(self):
        return hash(tuple(self))

    @property
    def comotion_point(self):
        return Point_2(*self.dimensions)  # TODO: include multi dimensional points.


class AbstractEntity(Point):
    pass


class Bonus(AbstractEntity):
    pass


class Goal(AbstractEntity):
    pass


class Robot(Point):
    pass


Entity = Union[Bonus, Goal]


class Segment:
    def __init__(self, src: Point, dst: Point):
        self.src = src if isinstance(src, Point) else Point(*src)
        self.dst = dst if isinstance(dst, Point) else Point(*dst)

    def __getitem__(self, key):
        if key not in [0, 1]:
            raise Exception(f'Key {key} does no exist')

        return self.src if key == 0 else self.dst

    def __iter__(self):
        return (self.src, self.dst).__iter__()

    def __repr__(self):
        return f'{type(self).__name__}({self.src}, {self.dst})'

    @property
    def comotion_segment(self):
        return Segment_2(self.src.comotion_point, self.dst.comotion_point)


class Path(list):
    def __init__(self, points: [Point],
                 length: float = 0,
                 endpoint: Entity = None,
                 extra_bonuses: int = 0,
                 added_distance: float = 0,
                 distance_to_goal: float = 0):
        super(Path, self).__init__(points)

        self.length = length
        self.endpoint = endpoint
        self.bonuses = extra_bonuses
        self.added_distance = added_distance
        self.distance_to_goal = distance_to_goal

    def __repr__(self):
        return f'{type(self).__name__}({self[0]} -> {self.endpoint}, {self.bonuses=}, {self.length=}, {self.added_distance=})'

    def __str__(self):
        return self.__repr__()

    @property
    def comotion_path(self) -> [Segment_2]:
        if len(self) == 1:
            return [Segment_2(Point_2(*self[0]), Point_2(*self[0]))]

        return [Segment_2(Point_2(*a), Point_2(*b)) for a, b in zip(self[:-1], self[1:])]


def l2_norm(p, q):
    return ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5
