#creat by sundong 2020/9/12
#!/usr/bin/env python

#from sensor_driver_msgs.msg import GpswithHeading
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


class RealTimePlotPose(object):

    def __init__(self):

        
        global e0, n0
        e0 = 0
        n0 = 0
        self.count = 0
        global gps_lat
        gps_lat = []
        global gps_lon
        gps_lon = []
        global sensor_fusion_lon
        sensor_fusion_lon = []
        global sensor_fusion_lat
        sensor_fusion_lat = []
        global fig
        fig = plt.figure()
        fig.canvas.mpl_connect('scroll_event', self.mouse_callback)
        fig.canvas.mpl_connect('button_press_event', self.mouse_callback)
        plt.axis('equal')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        plt.xlabel('East')
        plt.ylabel('North')
        plt.legend()
        plt.title("Plot Pose")
        fig.show()
        

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
        fig.canvas.draw_idle()



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


    def Gpsdatacallback(self, gps_data_):
        #gps_data = gps_data_
        a, b =self.positionGPStoMeters( gps_data_.gps.longitude, gps_data_.gps.latitude)
        gps_lon.append(a)
        gps_lat.append(b)
        
    def Sensor_fusiondatacallback(self, sensor_fusion_data_):
        #gps_data = gps_data_
        a, b =self.positionGPStoMeters( sensor_fusion_data_.gps.longitude, sensor_fusion_data_.gps.latitude)
        sensor_fusion_lon.append(a)
        sensor_fusion_lat.append(b) 
    
    def run(self):
        rospy.init_node('plot_pose', anonymous=True)
        rospy.Subscriber('GPSmsg', GpswithHeading, self.Gpsdatacallback)
        rospy.Subscriber('sensor_fusion_output', GpswithHeading, self.Sensor_fusiondatacallback)        
        
        while(not rospy.is_shutdown()):
            time.sleep(0.1)
            try:
                plt.plot(gps_lon, gps_lat, color='black', linestyle='-', linewidth=2.0, label='GPS')
                plt.plot(sensor_fusion_lon, sensor_fusion_lat, color='red', linestyle='-', linewidth=2.0, label='Sensor_fusion')
                
                if self.count == 0:
                    plt.legend()
                self.count += 1
                fig.canvas.draw()
            except ValueError as e:
                print(e)
                print("len of gps lat:", len(gps_lat),"len of gps lat:", len(gps_lon))
                
            
        rospy.spin()

if __name__ == '__main__':
    print("//////start to plot pose///////")
    rtp = RealTimePlotPose()
    rtp.run()

