import numpy as np
from matplotlib import pyplot as plt

'''
Designing and testing the minimal curvature algorithms
'''

def minimal_curvature_optimize(x, y, left_width, right_width):
    assert(len(x) == len(y) == len(left_width) == len(right_width))
    print(len(x))
    plt.scatter(x, y, color='red')
    plt.show()


if __name__ == '__main__':
    import centerline_search
    minimal_curvature_optimize(*centerline_search.get_centerline())