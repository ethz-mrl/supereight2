
import numpy as np
import cv2
import csv
import os
import sys
import struct

import rosbag, rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image, Imu, PointCloud2
from cv_bridge import CvBridge, CvBridgeError


"""        
    PointField[]            
                # This message holds the description of one point entry in the
                # PointCloud2 message format.
                uint8 INT8    = 1
                uint8 UINT8   = 2
                uint8 INT16   = 3
                uint8 UINT16  = 4
                uint8 INT32   = 5
                uint8 UINT32  = 6
                uint8 FLOAT32 = 7
                uint8 FLOAT64 = 8
                
                string name      # Name of field
                uint32 offset    # Offset from start of point struct
                uint8  datatype  # Datatype enumeration, see above
                uint32 count     # How many elements in the field
                
        Elios3 Ouster lidar format
        name: "x"
            offset: 0
            datatype: 7
            count: 1, 
        name: "y"
            offset: 4
            datatype: 7
            count: 1, 
        name: "z"
            offset: 8
            datatype: 7
            count: 1, 
        name: "intensity"
            offset: 16
            datatype: 7
            count: 1, 
        name: "t"
            offset: 20
            datatype: 6
            count: 1, 
        name: "reflectivity"
            offset: 24
            datatype: 4
            count: 1, 
        name: "ring"
            offset: 26
            datatype: 4
            count: 1, 
        name: "ambient"
            offset: 28
            datatype: 4
            count: 1, 
        name: "range"
            offset: 32
            datatype: 6
            count: 1

         => results in format = 'fffxxxxfIHHHxxIxxxxxxxxxx'

"""


# make sure that command line argument (filename given)
number_of_bags = len(sys.argv) - 1
if not number_of_bags > 0:
    sys.exit("No bag files provided.")
else:
    bagfiles = sys.argv[1:]
    print("Converting the following bags: ")
    for bag_file in bagfiles:
        print(bag_file)


# Now Iterate bag files and process
for bag_file in bagfiles:
    print("Processing " + bag_file)

    # Open bag file
    bag_data = rosbag.Bag(bag_file, 'r')
    bridge = CvBridge()

    # Create folder if not existing
    folder_name = bag_file[:-4]
    if not os.path.exists(folder_name):
        os.mkdir(folder_name)
    else:  # clear folder
        os.system('rm -rf ' + folder_name)
        os.mkdir(folder_name)

    # Setup folders for images
    os.mkdir(folder_name + '/cam0/')
    os.mkdir(folder_name + '/cam0/data/')
    os.mkdir(folder_name + '/cam1/')
    os.mkdir(folder_name + '/cam1/data/')
    os.mkdir(folder_name + '/cam2/')
    os.mkdir(folder_name + '/cam2/data/')
    # Setup folder for imu
    os.mkdir(folder_name + '/imu0/')
    # Setup folder for lidar
    os.mkdir(folder_name + '/lidar0/')
    # lidar byte format
    lidar_msg_format = 'fffxxxxfIHHHxxIxxxxxxxxxxxx'
    num_channels = 32
    num_hpoints = 1024

    # Prepare CSV Files
    cam0_csvFile = open(folder_name + "/cam0/data.csv", 'w')
    cam0_csvWriter = csv.writer(cam0_csvFile)
    cam0_csvWriter.writerow(['#timestamp [ns]', 'filename'])

    cam1_csvFile = open(folder_name + "/cam1/data.csv", 'w')
    cam1_csvWriter = csv.writer(cam1_csvFile)
    cam1_csvWriter.writerow(['#timestamp [ns]', 'filename'])

    cam2_csvFile = open(folder_name + "/cam2/data.csv", 'w')
    cam2_csvWriter = csv.writer(cam2_csvFile)
    cam2_csvWriter.writerow(['#timestamp [ns]', 'filename'])

    imu_csvFile = open(folder_name + "/imu0/data.csv", 'w')
    imu_csvWriter = csv.writer(imu_csvFile)
    imu_csvWriter.writerow(
        ['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]',
         'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])

    lidar_csvFile = open(folder_name + "/lidar0/data.csv", 'w')
    lidar_csvWriter = csv.writer(lidar_csvFile)
    lidar_csvWriter.writerow(
        ['#timestamp [ns]', 'x', 'y', 'z', 'Intensity', 'ring'])

    cnt_msg = 0
    prev_msg_timestamp = 0
    for topic, msg, t in bag_data:
        cnt_msg += 1

        # Cam0
        if topic == '/camera_0/image_raw':
            timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            cv2.imwrite(folder_name + '/cam0/data/' + str(timestamp) + ".png", cv_img)
            cam0_csvWriter.writerow([str(timestamp), str(timestamp) + ".png"])

        # Cam1
        if topic == '/camera_1/image_raw':
            timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            cv2.imwrite(folder_name + '/cam1/data/' + str(timestamp) + ".png", cv_img)
            cam1_csvWriter.writerow([str(timestamp), str(timestamp) + ".png"])

        # Cam2
        if topic == '/camera_2/image_raw':
            timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            cv2.imwrite(folder_name + '/cam2/data/' + str(timestamp) + ".png", cv_img)
            cam2_csvWriter.writerow([str(timestamp), str(timestamp) + ".png"])

        # IMU
        if topic == '/sensors/imu':
            timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
            acc_x = msg.linear_acceleration.x
            acc_y = msg.linear_acceleration.y
            acc_z = msg.linear_acceleration.z
            gyr_x = msg.angular_velocity.x
            gyr_y = msg.angular_velocity.y
            gyr_z = msg.angular_velocity.z
            imu_csvWriter.writerow([str(timestamp), gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z])

        # Lidar
        if topic == '/aft_output/lidar_scan':
            # ts_scan_start = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

            # access uint8_t[] data array and process according to PointField[] specification

            # print(msg.height)
            # print(msg.width)
            # print(msg.fields)

            test_iter = struct.iter_unpack(lidar_msg_format, msg.data)

            lidar_ts = np.zeros([num_channels*num_hpoints])
            lidar_x = np.zeros([num_channels*num_hpoints])
            lidar_y = np.zeros([num_channels*num_hpoints])
            lidar_z = np.zeros([num_channels*num_hpoints])
            lidar_intensity = np.zeros([num_channels*num_hpoints])
            lidar_ring = np.zeros([num_channels*num_hpoints])
            count = 0
            row_cnt = 0
            for packet in test_iter:
                # packet will be a tuple (x,y,z, intensity, timestamp, reflectivity, ring, ambient, range)
                time_since = packet[4]
                sec_i = msg.header.stamp.secs
                nsec_i = msg.header.stamp.nsecs
                timestamp = rospy.Time(sec_i, nsec_i + time_since)

                lidar_ts[count] = timestamp.to_nsec()
                lidar_x[count] = packet[0]
                lidar_y[count] = packet[1]
                lidar_z[count] = packet[2]
                lidar_intensity[count] = packet[3]
                lidar_ring[count] = packet[6]
                
                if time_since == 0:
                    row_cnt += 1
                count += 1

            prev_msg_timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_time()
            print(f"Detected {count} lidar points in {row_cnt} channels.")

            if row_cnt == num_channels:
                sidx = np.argsort(lidar_ts)
                slidar_ts = lidar_ts[sidx]
                slidar_x = lidar_x[sidx]
                slidar_y = lidar_y[sidx]
                slidar_z = lidar_z[sidx]
                slidar_intensity = lidar_intensity[sidx]
                slidar_ring = lidar_ring[sidx]

                for i in range(0,count):
                    x = slidar_x[i]
                    y = slidar_y[i]
                    z = slidar_z[i]
                    if x !=0 and y!=0 and z !=0:
                        ts = slidar_ts[i]
                        intensity = slidar_intensity[i]
                        ring = slidar_ring[i]
                        lidar_csvWriter.writerow([str(ts), x, y, z, intensity, ring])

    # Clsoe Csv files
    cam0_csvFile.close()
    cam1_csvFile.close()
    cam2_csvFile.close()
    imu_csvFile.close()
    lidar_csvFile.close()
    print("Finished Processing " + bag_file)

print("Finished Processing all bag files")

