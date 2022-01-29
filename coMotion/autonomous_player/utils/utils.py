from bindings import Point_2, Segment_2


class Point:
    def __init__(self, *dimensions: float):
        self.dimensions = dimensions

    def __getitem__(self, item):
        return self.dimensions[item]

    def __iter__(self):
        return self.dimensions.__iter__()

    @property
    def comotion_point(self):
        return Point_2(*self.dimensions)  # TODO: include multi dimensional points.


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

    @property
    def comotion_segment(self):
        return Segment_2(self.src.comotion_point, self.dst.comotion_point)


