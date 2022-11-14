import numpy as np
from matplotlib import pyplot as plt

"""
Natural cubic spline
The algorithm: 
http://en.wikipedia.org/w/index.php?title=Spline_%28mathematics%29&oldid=288288033#Algorithm_for_computing_natural_cubic_splines
"""

from typing import List


class CubicSpline:
    def __init__(self, _x: np.ndarray, _y: np.ndarray) -> None:
        self.x = _x

        n = len(_x) - 1
        assert(n > 1)

        assert(len(_x) == len(_y))

        self.a = _y
        assert(len(_y) == (n + 1))
        self.b = np.zeros(n)
        self.d = np.zeros(n)

        h = np.zeros(n)
        alpha = np.zeros(n)

        for index in range(len(h)):
            h[index] = _x[index + 1] - _x[index]
        
        alpha[0] = 0

        for index in range(1, len(alpha)):
            alpha[index] = 3 * (self.a[index + 1] - self.a[index]) / h[index] \
                - 3 / h[index - 1] * (self.a[index] - self.a[index - 1])

        self.c = np.empty(n + 1)
        l = np.empty(n + 1)
        mu = np.empty(n + 1)
        z = np.empty(n + 1)
        l[0] = 1
        mu[0] = 0
        z[0] = 0

        for i in range(1, n):
            l[i] = 2 * (_x[i + 1] - _x[i - 1]) - h[i - 1] * mu[i - 1]
            mu[i] = h[i] / l[i]

            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i]

        l[n] = 1
        z[n] = 0
        self.c[n] = 0

        for j in range(n - 1, -1, -1):
            self.c[j] = z[j] - mu[j] * self.c[j + 1]
            self.b[j] = (self.a[j + 1] - self.a[j]) / h[j] - h[j] * (self.c[j + 1] + 2 * self.c[j]) / 3
            self.d[j] = (self.c[j + 1] - self.c[j]) / (3 * h[j])

    def get(self, arg: float) -> float:
        assert self.x[0] <= arg <= self.x[-1]
        # find biggest x that satisfies x <= arg

        _index = [i for i, n in enumerate(self.x) if n >= arg][0] - 1

        if _index < 0:
            _index = 0

        delta_x = arg - self.x[_index]

        # assert(delta_x >= 0)

        return self.a[_index] + self.b[_index] * delta_x + self.c[_index] * delta_x ** 2 + self.d[_index] * delta_x ** 3
    
    def get_range(self, arg_array: np.ndarray) -> np.ndarray:
        spline_index = [i for i, v in enumerate(self.x) if v >= arg_array[0]][0] - 1

        if spline_index < 0:
            spline_index = 0

        result = np.zeros(*arg_array.shape)

        for _index, value in enumerate(arg_array):
            if value > self.x[spline_index + 1]:
                spline_index += 1
                assert(spline_index < len(self.b))

            ref_x = self.x[spline_index]
            delta_x = value - ref_x
            # assert(delta_x >= 0)
            result[_index] = (self.a[spline_index] + self.b[spline_index] * delta_x +
                              self.c[spline_index] * delta_x ** 2 + self.d[spline_index] * delta_x ** 3)

        return result

    def get_derivative_at(self, arg: float) -> float:
        assert self.x[0] <= arg <= self.x[-1]
        # find biggest x that satisfies x <= arg
        _index = [i for i, n in enumerate(self.x) if n >= arg][0] - 1

        if _index < 0:
            _index = 0

        delta_x = arg - self.x[_index]
        assert delta_x >= 0
        result = 3 * self.d[_index] * delta_x ** 2 + 2 * self.c[_index] * delta_x + self.b[_index]
        return result

    def get_second_derivative_at(self, arg: float) -> float:
        assert self.x[0] <= arg <= self.x[-1]
        # find biggest x that satisfies x <= arg
        _index = [i for i, n in enumerate(self.x) if n >= arg][0] - 1

        if _index < 0:
            _index = 0

        delta_x = arg - self.x[_index]
        assert delta_x >= 0

        return 6 * self.d[_index] * arg + 2 * self.c[_index]


if __name__ == '__main__':
    # generate x with random increments

    increments = np.random.rand(30)
    x = np.empty(*increments.shape)

    x[0] = 0

    for i in range(1, len(increments)):
        x[i] = x[i - 1] + increments[i]

    y = np.random.rand(*x.shape) * 4

    plt.scatter(x, y)

    spline = CubicSpline(x.tolist(), y.tolist())

    dX = 0.01
    xNew = np.arange(0, max(x), dX)
    spline_values = np.empty(*xNew.shape)
    derivatives = np.empty_like(spline_values)
    for index, item in enumerate(xNew):
        spline_values[index] = spline.get(item)
        derivatives[index] = spline.get_derivative_at(item)

    forward_diff = np.diff(spline_values) / dX

    plt.plot(xNew, spline_values)
    plt.plot(xNew, derivatives, label='From spline', color='red')
    plt.plot(xNew, np.zeros_like(xNew), color='gray')
    plt.plot(xNew[:-1:], forward_diff, label='Numerical')
    plt.legend()
    plt.title("Confirms that spline's derivatives are ok")
    plt.show()