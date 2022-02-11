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
        return f'{type(self).__name__}({tuple(val for val in self.dimensions)})'

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


def l2_norm(p, q):
    return ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5
