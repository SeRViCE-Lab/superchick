import sys
import time
import random
import threading

import os
from os.path import join
pwd = os.getcwd()
sys.path.append(join(pwd, '../'))

import numpy as np
import matplotlib.pyplot as plt
from utils import HeadTrajPlotter, display_chart
import matplotlib.gridspec as gridspec



fig = plt.figure()
# gs = gridspec.GridSpec(2, 3)
gs = gridspec.GridSpec(1,1) # rows cols


def demo_headtraj_plotter():
    i, j = 0, 0
    while True:
        i += random.randint(-10, 10)
        j += random.randint(-10, 10)
        data = [i, j, i + j, i - j]
        headtraj_plotter.update(data)
        time.sleep(1)


headtraj_plotter = HeadTrajPlotter(fig, gs[0]) # subplot in gridspec
display_chart(demo_headtraj_plotter)


plt.show()
