import numpy as np


def normalize(d, index):
    d[index] = (d[index] + np.pi) % (2 * np.pi) - np.pi

    # d[index] %= 2 * np.pi
    # mask = np.abs(d[index]) > np.pi
    # d[index, mask] -= (np.pi * 2)
