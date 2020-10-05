#!/usr/bin/env python
 # -*- coding: utf-8 -*

from math import *
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import re

pwd = os.getcwd()
filename = "/success50easy"
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


imudata = pd.read_csv(path+"/gpsdata.csv")
imutimes = imudata[".header.stamp.secs"]
imulat = imudata[".gps.latitude"]
imulon = imudata[".gps.longitude"]

imu_lat = []
imu_lon = []

pathdata = pd.read_csv(path+"/easy_path.csv")
pathtimes = pathdata[".header.stamp.secs"]
pathlat = pathdata[".gps.latitude"]
pathlon = pathdata[".gps.longitude"]

path_lat = []
path_lon = []

vboxdata = pd.read_csv(path+"/vboxdata.csv")
vboxtimes = vboxdata["Time"]
vboxlat = vboxdata["Latitude"]
vboxlon = vboxdata["Longitude"]
vbox_lat = []
vbox_lon = []
vboxlat_ = []
vboxlon_ = []

for i in range(len(vboxlat)):
    lat_split = re.split(u"\xc2°| N", vboxlat[i])[:2]
    if len(lat_split) == 2:
       x = [float(j) for j in lat_split]
       vboxlat_.append(x[0] + x[1] / 60)

for i in range(len(vboxlon)):
    lon_split = re.split(u"\xc2°| W", vboxlon[i])[:2]
    if len(lon_split) == 2:
       x = [float(j) for j in lon_split]
       vboxlon_.append(x[0] + x[1] / 60)


for i in range(len(imutimes)):
    while i % 5 == 0:
        a, b = positionGPStoMeters(imulon[i], imulat[i])
        imu_lon.append(a)
        imu_lat.append(b)
        break

for i in range(len(pathtimes)):
    a, b = positionGPStoMeters(pathlon[i], pathlat[i])
    path_lon.append(a)
    path_lat.append(b)

for i in range(len(vboxtimes)):
    a, b = positionGPStoMeters(vboxlon_[i], vboxlat_[i])
    vbox_lon.append(a-1.33) #
    vbox_lat.append(b+1.58) #

fig = plt.figure()
fig.canvas.mpl_connect('scroll_event', call_back)
fig.canvas.mpl_connect('button_press_event', call_back)
plt.axis('equal')
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)

plt.plot(imu_lon, imu_lat, color='black', linestyle=':', linewidth=2.0, label='SimPark982')
#plt.plot(path_lon, path_lat, color='black', linewidth=2.0, label='Path')
plt.plot(vbox_lon, vbox_lat, color='black', linestyle='-', linewidth=2.0, label='VBOX')

font2 = {'family' : 'Times New Roman',
'weight' : 'normal',
'size' : 20,
}

plt.xlabel('East', font2)
plt.ylabel('North',font2)
plt.legend()
plt.title(" ")
plt.show()
