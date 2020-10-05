#!/bin/zsh

bagname="2020-07-17-18-07-46tang.bag"
RelativePath="2020-07-17-18-04-39su"
rostopic echo -b $bagname -p /gpsdata > $RelativePath/gpsdata_ori.txt
rostopic echo -b $bagname -p /gpsdata_add > $RelativePath/gpsdata_add_ori.txt
rostopic echo -b $bagname -p /imudata > $RelativePath/imudata_ori.txt
rostopic echo -b $bagname -p /insvelocity > $RelativePath/insvelocity_ori.txt
rostopic echo -b $bagname -p /GPSmsg > $RelativePath/GPSmsg_ori.txt
rostopic echo -b $bagname -p /lidar_preciseodometry_to_earth > $RelativePath/lidar_preciseodometry_to_earth_ori.txt
rostopic echo -b $bagname -p /gps_by_lidar_odometry > $RelativePath/gps_by_lidar_odometry_ori.txt
rostopic echo -b $bagname -p /sensor_fusion_output > $RelativePath/sensor_fusion_output_ori.txt
rostopic echo -b $bagname -p /lidar_odometry_for_mapping > $RelativePath/lidar_odometry_for_mapping_ori.txt
rostopic echo -b $bagname -p /lidar_odometry_to_earth > $RelativePath/lidar_odometry_to_earth_ori.txt
rostopic echo -b $bagname -p /GPSmsg_fix > $RelativePath/GPSmsg_fix_ori.txt
cut -f3,10,11,12,24,25 -d',' $RelativePath/GPSmsg_ori.txt > $RelativePath/GPSmsg.txt
cut -f6,10,11 -d',' $RelativePath/GPSmsg_fix_ori.txt > $RelativePath/GPSmsg_fix.txt
cut -f3,10,11,12,23,24,25 -d',' $RelativePath/gpsdata_ori.txt > $RelativePath/gpsdata.txt
cut -f3,10,11,12,23,24,25 -d',' $RelativePath/gpsdata_add_ori.txt > $RelativePath/gpsdata_add.txt
cut -f3,99,100 -d',' $RelativePath/lidar_preciseodometry_to_earth_ori.txt > $RelativePath/lidar_preciseodometry_to_earth.txt
cut -f3,99,100,101,112 -d',' $RelativePath/gps_by_lidar_odometry_ori.txt > $RelativePath/gps_by_lidar_odometry.txt
cut -f3,12,13,14,15,99,100,101 -d',' $RelativePath/lidar_odometry_to_earth_ori.txt > $RelativePath/lidar_odometry_to_earth.txt
cut -f3,5,6,7,8,9,10 -d',' $RelativePath/insvelocity_ori.txt > $RelativePath/insvelocity.txt
cut -f3,30,31,32 -d',' $RelativePath/imudata_ori.txt > $RelativePath/imudata.txt
cut -f6,10,11,12,23,24,25,26 -d',' $RelativePath/sensor_fusion_output_ori.txt > $RelativePath/sensor_fusion_output.txt

sed -i '1,2d' $RelativePath/GPSmsg.txt
sed -i '1,2d' $RelativePath/GPSmsg_fix.txt
sed -i '1,2d' $RelativePath/gpsdata.txt
sed -i '1,2d' $RelativePath/gpsdata_add.txt
sed -i '1,2d' $RelativePath/imudata.txt
sed -i '1,2d' $RelativePath/lidar_preciseodometry_to_earth.txt
sed -i '1,2d' $RelativePath/gps_by_lidar_odometry.txt
sed -i '1,2d' $RelativePath/sensor_fusion_output.txt
sed -i '1,2d' $RelativePath/lidar_odometry_for_mapping.txt
sed -i '1,2d' $RelativePath/lidar_odometry_to_earth.txt
