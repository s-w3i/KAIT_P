#!/usr/bin/env python3
"""
Script that reads ROS2 messages from MCAP bags using the rosbag2_py API
and saves messages from /cmd_vel, /imu, /scan, and /odom into CSV files
within a new folder inside BAG_DIR. For the /scan topic, only the ranges are recorded.
"""

import csv
import os
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def extract_messages_from_bag(bag_folder: str, output_csv_prefix: str):
    # Check if the bag folder exists
    if not os.path.exists(bag_folder):
        print(f"Bag folder {bag_folder} does not exist.")
        return

    # Initialize CSV writers for each topic
    cmd_vel_csv = open(f"{output_csv_prefix}_cmd_vel.csv", 'w', newline='')
    imu_csv = open(f"{output_csv_prefix}_imu.csv", 'w', newline='')
    odom_csv = open(f"{output_csv_prefix}_odom.csv", 'w', newline='')

    cmd_vel_writer = csv.writer(cmd_vel_csv)
    imu_writer = csv.writer(imu_csv)
    odom_writer = csv.writer(odom_csv)

    # Write headers for each CSV file
    cmd_vel_writer.writerow(['timestamp', 'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'])
    imu_writer.writerow(['timestamp', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                         'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                         'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'])
    odom_writer.writerow(['timestamp', 'position_x', 'position_y', 'position_z',
                          'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                          'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
                          'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z'])

    # For /scan, we initialize the CSV writer after receiving the first message
    scan_csv = None
    scan_writer = None

    # Open the bag and read messages
    for topic, msg, timestamp in read_messages_from_bag(bag_folder):
        timestamp_sec = timestamp / 1e9  # Convert nanoseconds to seconds

        if topic == '/cmd_vel':
            linear = msg.linear
            angular = msg.angular
            cmd_vel_writer.writerow([
                timestamp_sec,
                linear.x, linear.y, linear.z,
                angular.x, angular.y, angular.z
            ])
        elif topic == '/imu':
            orientation = msg.orientation
            angular_velocity = msg.angular_velocity
            linear_acceleration = msg.linear_acceleration
            imu_writer.writerow([
                timestamp_sec,
                orientation.x, orientation.y, orientation.z, orientation.w,
                angular_velocity.x, angular_velocity.y, angular_velocity.z,
                linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
            ])
        elif topic == '/scan':
            if scan_writer is None:
                # Initialize the CSV writer and write the header
                num_ranges = len(msg.ranges)
                scan_csv = open(f"{output_csv_prefix}_scan.csv", 'w', newline='')
                scan_writer = csv.writer(scan_csv)
                scan_writer.writerow(['timestamp'] + [f'range_{i}' for i in range(num_ranges)])
            # Record only the ranges for /scan
            ranges = msg.ranges  # This is a list of floats
            scan_writer.writerow([timestamp_sec] + list(ranges))
        elif topic == '/odom':
            pose = msg.pose.pose
            twist = msg.twist.twist
            odom_writer.writerow([
                timestamp_sec,
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z
            ])

    # Close all CSV files
    cmd_vel_csv.close()
    imu_csv.close()
    odom_csv.close()
    if scan_csv:
        scan_csv.close()

    print(f"Processed {bag_folder} and saved CSV files into {os.path.dirname(output_csv_prefix)}.")

def read_messages_from_bag(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a mapping from topic names to message types
    type_map = {topic.name: topic.type for topic in topic_types}

    # Get message classes for topics
    msg_classes = {}
    for topic_name, topic_type in type_map.items():
        msg_class = get_message(topic_type)
        if msg_class is not None:
            msg_classes[topic_name] = msg_class
        else:
            print(f"Could not get message class for topic {topic_name} with type {topic_type}")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in msg_classes:
            msg = deserialize_message(data, msg_classes[topic])
            yield topic, msg, timestamp
    del reader

def main():
    rclpy.init(args=None)

    # Directory where bag files are stored
    BAG_DIR = 'data-930'  # Adjust this if you have a different directory

    # New directory inside BAG_DIR to store CSV files
    CSV_DIR = os.path.join(BAG_DIR, 'extracted_data')

    # Create the CSV_DIR if it doesn't exist
    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)

    # Base name for bag folders
    bag_base_name = 'cmd_vel_bag_'

    for i in range(1, 16):
        # Construct the path to the bag folder
        bag_folder = os.path.join(BAG_DIR, f"{bag_base_name}{i}")
        # Construct the output CSV prefix inside the CSV_DIR
        output_csv_prefix = os.path.join(CSV_DIR, f"{bag_base_name}{i}")
        extract_messages_from_bag(bag_folder, output_csv_prefix)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
