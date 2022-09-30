import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Pose

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import plotly.express as px

from BagFileParser import BagFileParser
from numpy import arctan2, arcsin

if __name__ == "__main__":

        bag_file = 'rosbag2_2022_09_20-14_55_29/rosbag2_2022_09_20-14_55_29_0.db3' 
        parser = BagFileParser(bag_file)
        pose_topic = BagFileParser(bag_file).get_messages('/blueye/pose')
        pose_timestamps = [msg[0] for msg in pose_topic]
        pose_data = [msg[1] for msg in pose_topic]
        
        depth = [elem.position.z for elem in pose_data]
        
        yaws = []
        pitches = []
        rolls = []

        for q in [elem.orientation for elem in pose_data]:
                yaw = arctan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
                pitch = arcsin(-2.0*(q.x*q.z - q.w*q.y))
                roll = arctan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
                yaws.append(yaw)
                pitches.append(pitch)
                rolls.append(roll)

        x = [elem.orientation.x for elem in pose_data]
        y = [elem.orientation.y for elem in pose_data]
        z = [elem.orientation.z for elem in pose_data]
        w = [elem.orientation.w for elem in pose_data]
        
        fig = plt.figure()
        ax1 = fig.add_subplot(231)
        ax2 = fig.add_subplot(232)
        ax3 = fig.add_subplot(233)
        ax4 = fig.add_subplot(234)
        # ax5 = fig.add_subplot(235)


        ax1.scatter(pose_timestamps, depth)
        ax2.scatter(pose_timestamps, yaws)
        ax3.scatter(pose_timestamps, pitches)
        ax4.scatter(pose_timestamps, rolls)
        # ax5.scatter(pose_timestamps, w)
        plt.show()