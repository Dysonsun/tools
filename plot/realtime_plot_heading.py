#creat by sundong 2020/9/12
#!/usr/bin/env python

from sensor_driver_msgs.msg import GpswithHeading
import rospy
import matplotlib.pyplot as plt
from math import *
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import re
import time
import math
import sys
import threading



class Plotter(object):

    def __init__(self, ax1, ax2):
    
        
        global e0, n0
        e0 = 0
        n0 = 0
        self.count = 0
        self.ax1 = ax1
        self.ax2 = ax2
        self.updategraph = False
        self.gps_lat = []
        self.gps_lon = []
        self.imu_lat = []
        self.imu_lon = []
        self.fusion_lon = []
        self.fusion_lat = []
        
        self.imu_heading = []
        self.fusion_heading = []

        self.imu_time = []
        self.gps_time = []
        self.fusion_time = []

        self.lock = threading.Lock()

    def mouse_callback(self, event):
        axtemp=event.inaxes
        x_min, x_max = axtemp.get_xlim()
        y_min, y_max = axtemp.get_ylim()
        fanwei_x = (x_max - x_min) / 10
        fanwei_y = (y_max - y_min) / 10
        if event.button == 'up':
            axtemp.set(xlim=(x_min + fanwei_x, x_max - fanwei_x))
            axtemp.set(ylim=(y_min + fanwei_y, y_max - fanwei_y))        
        elif event.button == 'down':
            axtemp.set(xlim=(x_min - fanwei_x, x_max + fanwei_x))
            axtemp.set(ylim=(y_min - fanwei_y, y_max + fanwei_y))
        # 

    def press(self, event):
        """Keyboard events during plotting"""
        if event.key == 'q' or event.key == 'Q':
            plt.close('all')
            self.closed = True

        if event.key == 'x' or event.key == 'X':
            self.updategraph = True

        if event.key == 'a' or event.key == 'A':
            fig = plt.gcf()
            fig.gca().autoscale()
            fig.canvas.draw()

        if event.key == 'n' or event.key == 'N':
            with self.lock:
                self.reset()
            self.updategraph = True

    def positionGPStoMeters(self, longitude, latitude):
        WGS84_ECCENTRICITY = 0.0818192
        WGS84_EQUATORIAL_RADIUS = 6378.137
        k0 = 0.9996

        Zone = (int)(longitude / 6) + 1
        lonBase = Zone * 6 - 3

        vPhi = (float)(1 / sqrt(1 - pow(WGS84_ECCENTRICITY * sin(latitude * pi / 180.0), 2)))
        A = (float)((longitude - lonBase) * pi / 180.0 * cos(latitude * pi / 180.0))
        sPhi = (float)((1 - pow(WGS84_ECCENTRICITY, 2) / 4.0 - 3 * pow(WGS84_ECCENTRICITY, 4) / 64.0
                        - 5 * pow(WGS84_ECCENTRICITY, 6) / 256.0) * latitude * pi / 180.0
                    - (3 * pow(WGS84_ECCENTRICITY, 2) / 8.0 + 3 * pow(WGS84_ECCENTRICITY, 4) / 32.0
                        + 45 * pow(WGS84_ECCENTRICITY, 6) / 1024.0) * sin(2 * latitude * pi / 180.0)
                    + (15 * pow(WGS84_ECCENTRICITY, 4) / 256.0 + 45 * pow(WGS84_ECCENTRICITY, 6) / 256.0)
                    * sin(4 * latitude * pi / 180.0)
                    - (35 * pow(WGS84_ECCENTRICITY, 6) / 3072.0) * sin(6 * latitude * pi / 180.0))
        T = (float)(pow(tan(latitude * pi / 180.0), 2))
        C = (float)((pow(WGS84_ECCENTRICITY, 2) / (1 - pow(WGS84_ECCENTRICITY, 2)))
                    * pow(cos(latitude * pi / 180.0), 2))

        pose_x = (float)((k0 * WGS84_EQUATORIAL_RADIUS * vPhi * (A + (1 - T + C) * pow(A, 3) / 6.0
                                                                + (5 - 18 * T + pow(T, 2)) * pow(A, 5) / 120.0)) * 1000)
        pose_y = (float)((k0 * WGS84_EQUATORIAL_RADIUS * (sPhi + vPhi * tan(latitude * pi / 180.0) * (pow(A, 2) / 2
                                                                                                    + (
                                                                                                                5 - T + 9 * C + 4 * C * C) * pow(
                    A, 4) / 24.0 + (61 - 58 * T + T * T) * pow(A, 6) / 720.0))) * 1000)
        global e0, n0
        if (0 == e0 and 0 == n0):
            e0 = int(pose_x)
            n0 = int(pose_y)

        pose_x -= e0
        pose_y -= n0

        return pose_x, pose_y
    def reset(self):
        """Reset"""
        del self.gps_lat[:]
        del self.gps_lon[:]

        del self.imu_lat[:]
        del self.imu_lon[:]

        del self.fusion_lat[:]
        del self.fusion_lon[:]

        del self.imu_heading[:]
        del self.fusion_headin[:]

        del self.imu_time[:]
        del self.gps_time[:]  
        self.count = 0
        self.ax1.cla()

    def callback_fusion(self, data):
        self.fusion_time.append(data.header.stamp.secs + data.header.stamp.nsecs/1000000000.0)
        self.fusion_heading.append(data.heading) 
        a, b =self.positionGPStoMeters( data.gps.longitude, data.gps.latitude)
        self.fusion_lon.append(a)
        self.fusion_lat.append(b)

    def callback_gps(self, data):
        self.gps_time.append(data.header.stamp.secs + data.header.stamp.nsecs/1000000000.0) 
        #gps_heading.append(data.heading) 
        a, b =self.positionGPStoMeters( data.gps.longitude, data.gps.latitude)
        self.gps_lon.append(a)
        self.gps_lat.append(b)

    def callback_imu(self, data):
        self.imu_time.append(data.header.stamp.secs + data.header.stamp.nsecs/1000000000.0) 
        self.imu_heading.append(data.heading) 
        a, b =self.positionGPStoMeters( data.gps.longitude, data.gps.latitude)
        self.imu_lon.append(a)
        self.imu_lat.append(b)

    def xyplot(self):
        self.ax1.plot(self.fusion_lat, self.fusion_lon,
            color='red', linestyle='-', label = "Fusion")
        self.ax1.plot(self.gps_lat, self.gps_lon,
            color='black', linestyle='-', label = "GPS")
        self.ax1.plot(self.imu_lat, self.imu_lon,
            color='blue', linestyle='-', label = "IMU")
    def headingplot(self):
        self.ax2.plot(self.fusion_time, self.fusion_heading,
            color='red', linestyle='-', label = 'Fusion')
        self.ax2.plot(self.imu_time, self.imu_heading,
            color='blue', linestyle='-', label = 'IMU')
    

        

         
        
def main(argv):

    print """
    Keyboard Shortcut:
        [q]: Quit Tool
        [s]: Save Figure
        [a]: Auto-adjust x, y axis to display entire plot
        [x]: Update Figure
        [h][r]: Go back Home, Display all Trajectory
        [f]: Toggle Full Screen
        [n]: Reset all Plots
        [b]: Unsubscribe Topics

    """

    rospy.init_node('plot_pose', anonymous=True)
    fig = plt.figure()

    ax1 = plt.subplot(2, 1, 1)
    ax1.set_title("Position")
    ax2 = plt.subplot(2, 1, 2)
    ax2.set_title("Heading")
    plt.ion()
    plt.pause(0.0001)
    plt.show()
    plotter = Plotter(ax1, ax2)

    fig.canvas.mpl_connect('scroll_event', plotter.mouse_callback)
    fig.canvas.mpl_connect('button_press_event', plotter.mouse_callback)
    fig.canvas.mpl_connect('key_press_event', plotter.press)

    gps_sub = rospy.Subscriber('GPSmsg', GpswithHeading, 
                                    plotter.callback_gps)
    fusion_sub = rospy.Subscriber('sensor_fusion_output', 
                            GpswithHeading, plotter.callback_fusion)
    imu_sub = rospy.Subscriber('gpsdata', 
                            GpswithHeading, plotter.callback_imu)

    while not rospy.is_shutdown():
        try:
            ax1.draw_artist(ax1.patch)
            ax2.draw_artist(ax2.patch)

            with plotter.lock:
                plotter.xyplot()
                plotter.headingplot()
                fig.canvas.draw_idle()
            if plotter.count == 0:
                ax1.legend()
                ax2.legend()
            fig.canvas.blit(ax1.bbox)
            fig.canvas.blit(ax2.bbox)
            fig.canvas.flush_events()
            plotter.count += 1
        except ValueError as e:
            print(e)

    rospy.spin()

if __name__ == '__main__':
    print("//////start to plot pose///////")
    main(sys.argv)

