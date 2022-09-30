from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

from BagFileParser import BagFileParser

def create_points(msgs):
    points = []
    for msg in msgs:
        timestamp = msg[0]
        vector = msg[1].vector
        points.append([vector.x, vector.y, vector.z])
    return points

def create_values(msgs):
    values = []
    for msg in msgs:
        timestamp = msg[0]
        value = msg[1].data
        values.append(value)
    return values

def ith_column(list, i):
    return [element[i] for element in list]

if __name__ == "__main__":

        bag_file = 'rosbag2_2022_09_16-10_30_49/rosbag2_2022_09_16-10_30_49_0.db3'

        values = create_values(BagFileParser(bag_file).get_messages('/blueye/dummy_value'))
        points = create_points(BagFileParser(bag_file).get_messages("/blueye/dummy_position"))
        X = ith_column(points, 0)
        Y = ith_column(points, 1)
        Z = ith_column(points, 2)

        min_len = min(len(values), len(X), len(Y), len(Z))
        values = values[:min_len]
        X = X[:min_len]
        Y = Y[:min_len]
        Z = Z[:min_len]

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(X, Z, Y, c=values, cmap='RdYlGn')
        ax.set_yticks(np.arange(0, 50, 5))    
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')
        plt.show()
