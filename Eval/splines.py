import numpy as np
from matplotlib import pyplot as plt

"""
Natural cubic spline
The algorithm:
http://en.wikipedia.org/w/index.php?title=Spline_%28mathematics%29&oldid=288288033#Algorithm_for_computing_natural_cubic_splines
"""

from typing import List

class CubicSpline:
    def __init__(self, x: List[float], y: List[float]) -> None:
        self.x = x

        n = len(x) - 1
        assert (n > 1)

        assert (len(x) == len(y))

        self.a = y
        self.b = np.empty(n)
        self.d = np.empty(n)

        h = np.empty(n)
        alpha = np.empty(n)

        for index in range(len(h)):
            h[index] = x[index + 1] - x[index]

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
            l[i] = 2 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1]
            mu[i] = h[i] / l[i]

            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i]

        l[n] = 1
        z[n] = 0
        self.c[n] = 0

        for j in range(n - 1, -1, -1):
            self.c[j] = z[j] - mu[j] * self.c[j + 1]
            self.b[j] = (self.a[j + 1] - self.a[j]) / h[j] - \
                h[j] * (self.c[j + 1] + 2 * self.c[j]) / 3
            self.d[j] = (self.c[j + 1] - self.c[j]) / (3 * h[j])

    def get(self, arg: float) -> float:
        index = [i for i, v in enumerate(self.x) if v >= arg][0] - 1

        index = max(index, 0)
        delta_x = arg - self.x[index]

        # assert(delta_x >= 0)

        return (self.a[index] + self.b[index] * delta_x + self.c[index] * delta_x ** 2 + self.d[index] * delta_x ** 3)

    def get_range(self, arg_array):
        spline_index = [i for i, v in enumerate(
            self.x) if v > arg_array[0]][0] - 1

        spline_index = max(spline_index, 0)
        result = np.zeros(*arg_array.shape)

        for index, value in enumerate(arg_array):
            if value > self.x[spline_index + 1]:
                spline_index += 1
                assert (spline_index < len(self.b))

            ref_x = self.x[spline_index]
            delta_x = value - ref_x
            # assert(delta_x >= 0)
            result[index] = (self.a[spline_index] + self.b[spline_index] * delta_x +
                             self.c[spline_index] * delta_x ** 2 + self.d[spline_index] * delta_x ** 3)

        return result


if __name__ == '__main__':
    # # generate x with random increments
    #
    # increments = np.random.rand(30)
    # x = np.empty(*increments.shape)
    #
    # x[0] = 0
    #
    # for i in range(1, len(increments)):
    #     x[i] = x[i - 1] + increments[i]
    #
    # y = np.random.rand(*x.shape) * 4
    #
    # plt.scatter(x, y)
    #
    # spline = CubicSpline(x.tolist(), y.tolist())
    #
    xNew = np.arange(0, 1.0, 0.01)
    # spline_values = np.empty(*xNew.shape)
    #
    # for index, item in enumerate(xNew):
    #     spline_values[index] = spline.get(item)
    #
    # spline_val = spline.get_range(xNew)
    #
    # plt.plot(xNew, spline_values)
    # plt.plot(xNew, spline_val)
    # plt.show()
    x = np.linspace(0, 1, 5)
    y = [0.51716988, 0.16154433, 0.16876195, 0.4129143 , 0.88514508]
    spl = CubicSpline(x, y)
    print(f"a: {spl.a} \nb: {spl.b} \nc: {spl.c} \nd: {spl.d}")
    plt.plot(np.arange(0.0, 1.0, 0.01), [0.51717, 0.499577, 0.482016, 0.46452, 0.447121, 0.429852, 0.412744, 0.395831, 0.379145, 0.362718, 0.346582, 0.33077, 0.315314, 0.300247, 0.285601, 0.271408, 0.257702, 0.244513, 0.231875, 0.21982, 0.20838, 0.197588, 0.187476, 0.178076, 0.169422, 0.161544, 0.154467, 0.148178, 0.142652, 0.137869, 0.133805, 0.130437, 0.127744, 0.125702, 0.124288, 0.123482, 0.123258, 0.123596, 0.124472, 0.125865, 0.12775, 0.130106, 0.13291, 0.13614, 0.139772, 0.143785, 0.148156, 0.152861, 0.157879, 0.163187, 0.168762, 0.174587, 0.180667, 0.18701, 0.193627, 0.200528, 0.207721, 0.215217, 0.223026, 0.231156, 0.239618, 0.248422, 0.257576, 0.267091, 0.276976, 0.287241, 0.297896, 0.30895, 0.320413, 0.332294, 0.344604, 0.357351, 0.370547, 0.384199, 0.398318, 0.412914, 0.427992, 0.443536, 0.459527, 0.475946, 0.492774, 0.509991, 0.527577, 0.545513, 0.56378, 0.582358, 0.601228, 0.62037, 0.639765, 0.659393, 0.679235, 0.699272, 0.719484, 0.739851, 0.760354, 0.780975, 0.801692, 0.822487, 0.843341, 0.864233])
    plt.scatter(x, y)
    plt.plot(np.arange(0.0, 1.0, 0.01), spl.get_range(xNew))
    plt.show()