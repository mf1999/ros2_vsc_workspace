import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import plotly.express as px

from BagFileParser import BagFileParser

if __name__ == "__main__":

        bag_file = 'rosbag2_2022_09_19-15_56_52/rosbag2_2022_09_19-15_56_52_0.db3'  
        parser = BagFileParser(bag_file)
        camera_topic = BagFileParser(bag_file).get_messages('/blueye/camera_img_raw_drop')
        biofouling_topic = BagFileParser(bag_file).get_messages('/blueye/biofouling_ratio')

        camera_timestamps = [msg[0] for msg in camera_topic]
        biofouling_timestamps = [msg[0] for msg in biofouling_topic]
        
        fig = plt.figure()
        ax1 = fig.add_subplot(111)

        ax1.scatter(camera_timestamps, [0]*len(camera_timestamps))
        ax1.scatter(biofouling_timestamps, [1]*len(biofouling_timestamps))
        plt.show()