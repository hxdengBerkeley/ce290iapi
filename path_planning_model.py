#!/usr/bin/env bash
import skfmm
import numpy as np
import time
import matplotlib.pyplot as plt
import os

class drone_path_planning_model():
    def __init__(self, n, centroid_x, centroid_y, radius):
        self.n = n
        self.word_size = 20.
        self.radius = radius
        self.centroid_x = centroid_x
        self.centroid_y = centroid_y
        self.path = []

    def planning(self):
        n = self.n
        word_size = self.word_size
        radius = self.radius
        center_x = self.centroid_x / word_size * n
        center_y = self.centroid_y / word_size * n
        grid_world = np.ones((n + 1, n + 1))
        mask = np.full(np.shape(grid_world), False, dtype=bool)
        for i in range(n + 1):
            for j in range(n + 1):
                if np.sqrt((i - center_x) ** 2 + (j - center_y) ** 2) <= radius / word_size * n:
                    mask[i, j] = True
        grid_world_A = np.ma.MaskedArray(np.ones((n + 1, n + 1)), mask)
        grid_world_A[0, 0] = 0
        grid_world_B = np.ma.MaskedArray(np.ones((n + 1, n + 1)), mask)
        grid_world_B[n, n] = 0
        self.dist_map_A = skfmm.travel_time(grid_world_A, np.ones_like(grid_world), dx=word_size / n)
        self.shortest_distance = self.dist_map_A[n, n]
        self.dist_map_B = skfmm.travel_time(grid_world_B, np.ones_like(grid_world), dx=word_size / n)
        self.shortest_path()

    def shortest_path(self):
        path = []
        n = self.n
        i = 0
        j = 0
        dist_map = self.dist_map_A + self.dist_map_B
        while not (i == n and j == n):
            path.append([(i / self.n * self.word_size), (j / self.n * self.word_size)])
            dist = 400.
            next_point = [-1, -1]
            for next_i, next_j in [[i + 1, j], [i + 1, j + 1], [i, j + 1]]:
                if next_i <= n and next_j <= n and dist_map[next_i][next_j] < dist:
                    dist = dist_map[next_i][next_j]
                    next_point[0] = next_i
                    next_point[1] = next_j
            if dist == 400.:
                print("Error Input")
                break
            i = next_point[0]
            j = next_point[1]
        path.append([self.word_size, self.word_size])
        self.path = path

    def shortest_distance(self):
        return self.shortest_distance

    def visualize_path(self):
        data = np.array(self.path)
        storm_plt = plt.Circle((self.centroid_x, self.centroid_y), self.radius, color='grey')
        path_plt = plt.plot(*data.T, color='red')
        ax = plt.gca()
        ax.add_patch(storm_plt)
        ax.legend(path_plt, ["Shortest Path"])
        ax.text(self.centroid_x, self.centroid_y, 'Storm')
        ax.text(0, 0, 'Hospital A')
        ax.text(self.word_size, self.word_size, 'Hospital B')
        plt.show()

    def save_figure(self, img_path):
        data = np.array(self.path)
        storm_plt = plt.Circle((self.centroid_x, self.centroid_y), self.radius, color='grey')
        path_plt = plt.plot(*data.T, color='red')
        ax = plt.gca()
        ax.add_patch(storm_plt)
        ax.legend(path_plt, ["Shortest Path"])
        ax.text(self.centroid_x, self.centroid_y, 'Storm')
        ax.text(0, 0, 'Hospital A')
        ax.text(self.word_size, self.word_size, 'Hospital B')
        if os.path.isfile(img_path):
            os.remove(img_path)
        plt.savefig(img_path)
        plt.close('all')