from math import sqrt
from typing import Tuple


def calculate_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)


def calculate_distance_point(P1: Tuple[float, float], P2: Tuple[float, float]) -> float:
    return calculate_distance(P1[0], P1[1], P2[0], P2[1])


def left_side_perpendicular_vector(x: float, y:float) -> Tuple[float, float]:
    return -y, x


def vector_length(x: float, y: float) -> float:
    return sqrt(x**2 + y**2)


def unit_vector(x: float, y: float) -> Tuple[float, float]:
    n = vector_length(x, y)
    return x / n, y / n


def curvature(x_prime, y_prime, x_bis, y_bis):
    from math import sqrt, isclose
    nom = x_prime * y_bis - y_prime * x_bis
    denom = sqrt((x_prime**2 + y_prime**2)**3)

    if isclose(denom, 0):
        return 0    # if both derivatives are ~0, assume that track is linear

    return nom / denom


def curvature_squared(x_prime, y_prime, x_bis, y_bis):

    # TODO: middle part of the nominator was omitted
    nom = x_prime**2 * y_bis**2 - 2 * x_prime * x_bis * y_prime * y_bis + y_prime**2 * x_bis**2
    denom = (x_prime**2 + y_prime**2)**3
    assert denom != 0

    return nom / denom
