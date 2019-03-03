#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import sys
import seaborn as sns
from matplotlib.animation import FuncAnimation

class SimpleBot():
    def __init__(self, id = 0, max_vel = 0.8, max_acc = 0.4, radius = 0.3, color = "red", odom = np.array((0.0, 0.0, 0.0, 0.0))):
        #odom := (pose(x,y), velocity(x, y))
        self.id = id
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.radius = radius
        self.color = color
        self.odom = np.copy(odom)

    def move(self, cmd, dt):
        self.odom[0:2] = self.odom[0:2] + cmd * dt #position
        self.odom[2:4] = cmd #velocity

    def plot(self, ax):
        rand = np.random.randn(100)
        #ax.scatter(self.odom[0], self.odom[1], color=self.color, marker = "o", s=self.radius*1000)
        ax.add_patch(plt.Circle(self.odom[0:2], self.radius, facecolor=self.color, alpha=.7))
