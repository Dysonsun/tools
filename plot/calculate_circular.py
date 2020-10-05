#!/usr/bin/env python
 # -*- coding: utf-8 -*

from math import *
import cmath
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import re

pwd = os.getcwd()
filename = "/success50easy2"
path = pwd+filename

class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

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

def Calculate_cicular(p1, p2, p3):
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    x3, y3 = p3.x, p3.y
    e = 2 * (x2 - x1)
    f = 2 * (y2 - y1)
    g = x2*x2 - x1*x1 + y2*y2 - y1*y1
    a = 2 * (x3 - x2)
    b = 2 * (y3 - y2)
    c = x3*x3 - x2*x2 + y3*y3 - y2*y2
    if e*b - a*f == 0 or a*f - b*e == 0:
        return 999, 999, 999
    X = (g*b - c*f) / (e*b - a*f)
    Y = (a*g - c*e) / (a*f - b*e)
    R = ((X-x1)*(X-x1)+(Y-y1)*(Y-y1)) ** 0.5
    return X, Y, R



pathdata = pd.read_csv(path+"/easy_path.csv")
pathtimes = pathdata[".header.stamp.secs"]
pathlat = pathdata[".gps.latitude"]
pathlon = pathdata[".gps.longitude"]

path_lat = []
path_lon = []


    a, b = positionGPStoMeters(pathlon[i], pathlat[i])
    path_lon.append(a)
    path_lat.append(b)


fig = plt.figure()
fig.canvas.mpl_connect('scroll_event', call_back)
fig.canvas.mpl_connect('button_press_event', call_back)
#plt.axis('equal')
#plt.xticks(fontsize=20)
#plt.yticks(fontsize=20)
pathGraph = fig.add_subplot(211)
pathGraph.axis([-20, 200, -250, 50])
pointGraph = fig.add_subplot(212)
pointGraph.axis([-20, 200, 0, 120])

pathGraph.plot(path_lon, path_lat, color='black', linewidth=2.0, label='Path')


p1 = Point(0, 0)
p2 = Point(0, 0)
p3 = Point(0, 0)
circular_x = []
circular_min = []
for i in range(1500, len(pathtimes) - 1000, 10):
    p1.x, p1.y = (path_lon[i-3]+path_lon[i]+path_lon[i+3])/3, (path_lat[i-3]+path_lat[i]+path_lat[i+3])/3
    p2.x, p2.y = (path_lon[i+7]+path_lon[i+10]+path_lon[i+13])/3, (path_lat[i+7]+path_lat[i+10]+path_lat[i+13])/3
    p3.x, p3.y = (path_lon[i+17]+path_lon[i+20]+path_lon[i+23])/3, (path_lat[i+17]+path_lat[i+20]+path_lat[i+23])/3
    circule_x, circule_y, circule_r = Calculate_cicular(p1, p2, p3)
    if(circule_r > 10 and circule_r < 100 and ((p2.x-p1.x)**2 + (p2.y - p1.y)** 2) > 1 ):
        print(i, circule_x, circule_y, circule_r)
        circular_min.append(circule_r)
        circular_x.append(path_lon[i+10])
        pathGraph.scatter(path_lon[i+10], path_lat[i+10], s = 40, color = 'red', marker = '.' )

pointGraph.scatter(circular_x, circular_min, s = 40, color = 'red', marker = '.' )


font2 = {'family' : 'Times New Roman',
'weight' : 'normal',
'size' : 20,
}

#plt.xlabel('East', font2)
#plt.ylabel('North',font2)
plt.legend()
plt.title(" ")
plt.show()
