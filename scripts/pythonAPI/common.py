import enum

class Angle(enum.Enum):
    J1 = '00'
    J2 = '01'
    J3 = '02'
    J4 = '03'
    J5 = '04'
    J6 = '05'


class Coord(enum.Enum):
    X = '00'
    Y = '01'
    Z = '02'
    Rx = '03'
    Ry = '04'
    Rz = '05'