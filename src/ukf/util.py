# coding=utf-8
import numpy as np


def normalize(d, index):
    d[index] = (d[index] + np.pi) % (2 * np.pi) - np.pi


def angle_diff(angle1, angle2):
    diff = (angle2 - angle1 + np.pi) % (2 * np.pi) - np.pi

    diff[diff < -np.pi] += 2 * np.pi

    return diff

    # d[index] %= 2 * np.pi
    # mask = np.abs(d[index]) > np.pi
    # d[index, mask] -= (np.pi * 2)
