import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import matplotlib.pyplot as plt
import pandas as pd

import plotly.express as px



class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]


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

        bag_file = 'rosbag2_2022_09_15-14_03_24/rosbag2_2022_09_15-14_03_24_0.db3'

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

        dict = {'x': X, 'y': Y, 'z': Z, 'v': values}
        df = pd.DataFrame(data=dict)

        fig = px.scatter_3d(df, x='x', y='y', z='z',
              color='v')
        fig.show()
        # print(points)
        # print(values)

        