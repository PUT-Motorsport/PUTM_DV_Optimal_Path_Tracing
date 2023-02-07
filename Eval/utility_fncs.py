from math import sqrt
from typing import Tuple


def calculate_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)


def calculate_distance_point(P1: Tuple[float, float], P2: Tuple[float, float]) -> float:
    return calculate_distance(P1[0], P1[1], P2[0], P2[1])


def left_side_perpendicular_vector(x: float, y: float) -> Tuple[float, float]:
    return -y, x


def vector_length(x: float, y: float) -> float:
    return sqrt(x**2 + y**2)


def unit_vector(x: float, y: float) -> Tuple[float, float]:
    n = vector_length(x, y)
    return x / n, y / n
