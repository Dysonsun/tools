#!/usr/bin/env python
 # -*- coding: utf-8 -*

from math import *
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import re
import math

pwd = os.getcwd()
filename = "/all_2020-10-28-08-45-17_0"
path = pwd+filename

def call_back(event):
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
    fig.canvas.draw_idle()  # 绘图动作实时反映在图像上

e0 = 0
n0 = 0

def positionGPStoMeters(longitude, latitude):
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

def Least_squares(x,y):
    x_sum = 0
    y_sum = 0
    for t in range(len(x)):
        x_sum = x_sum + x[t]
        y_sum = y_sum + y[t]
    x_ = x_sum/len(x)
    y_ = y_sum/len(y)
    m = np.zeros(1)
    n = np.zeros(1)
    k = np.zeros(1)
    p = np.zeros(1)
    for i in np.arange(len(x)):
        k = (x[i]-x_)* (y[i]-y_)
        m += k
        p = np.square( x[i]-x_ )
        n = n + p
    a = m/n
    b = y_ - a* x_
    return a,b
    
pathdata = pd.read_csv(path+"/GPSmsg.csv")
pathtimes = pathdata[".gps.header.stamp.secs"]
pathlat = pathdata[".gps.latitude"]
pathlon = pathdata[".gps.longitude"]

pathdata = pd.read_csv(path+"/gpsdata.csv")
imuheading = pathdata[".heading"]

path_lat = []
path_lon = []
path_lat_fix = []
heading_sum = 0

for i in range(len(pathtimes)):
    a, b = positionGPStoMeters(pathlon[i], pathlat[i])
    path_lon.append(a)
    path_lat.append(b)
for i in range(len(imuheading)):
    heading_sum = heading_sum + imuheading[i]
heading_avg = heading_sum/len(imuheading)
 
k, m = Least_squares(path_lon, path_lat)
print(k, m)

angle = float(k * 180/math.pi)
if path_lon[100] > 0:
    angle = -90 + angle
else:
    angle = 90 + angle
print(heading_avg, angle)
for j in range(len(path_lon)):
    y = k*path_lon[j] + m
    path_lat_fix.append(y)



fig = plt.figure()
fig.canvas.mpl_connect('scroll_event', call_back)
fig.canvas.mpl_connect('button_press_event', call_back)
plt.axis('equal')
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)

plt.plot(path_lon, path_lat, color='red', linewidth=1, label='GPS')
plt.plot(path_lon, path_lat_fix, color='black', linewidth=1, label='GPS_Calibrate')

font2 = {'family' : 'Times New Roman',
'weight' : 'normal',
'size' : 20,
}

plt.xlabel('South(m)', font2)
plt.ylabel('North(m)',font2)
plt.legend()
plt.title(" ")
plt.show()
