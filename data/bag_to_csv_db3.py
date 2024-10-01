#!/usr/bin/env python3
"""
Script to convert ROS2 .db3 bag files to CSV format for specified topics.

Extracted Topics:
- /cmd_vel
- /imu
- /scan (only ranges)
- /odom

CSV Output:
Each topic will have its own CSV file stored in the 'extracted_data' directory within 'bag_files'.
"""

import csv
import os
import sys
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def extract_messages_from_bag(bag_file: str, csv_dir: str):
    """
    Extracts messages from specified topics in a .db3 bag file and writes them to CSV files.

    Parameters:
    - bag_file: Path to the .db3 bag folder.
    - csv_dir: Directory where the CSV files will be saved.
    """

    # Initialize rosbag2_py SequentialReader
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Failed to open bag file {bag_file}: {e}")
        return

    # Get all topics and types
    topics_and_types = reader.get_all_topics_and_types()
    topic_type_map = {topic.name: topic.type for topic in topics_and_types}

    # Define the topics to extract
    desired_topics = ['/cmd_vel', '/imu', '/scan', '/odom']

    # Verify that desired topics exist in the bag
    for topic in desired_topics:
        if topic not in topic_type_map:
            print(f"Warning: Topic {topic} not found in bag {bag_file}. Skipping.")
    
    # Get message classes for desired topics
    msg_classes = {}
    for topic in desired_topics:
        if topic in topic_type_map:
            msg_type = topic_type_map[topic]
            msg_class = get_message(msg_type)
            if msg_class is None:
                print(f"Warning: Message class for type {msg_type} not found. Topic {topic} will be skipped.")
            else:
                msg_classes[topic] = msg_class

    # Initialize CSV writers
    csv_writers = {}
    csv_files = {}

    # Create CSV files with headers
    for topic, msg_class in msg_classes.items():
        if topic == '/cmd_vel':
            csv_path = os.path.join(csv_dir, 'cmd_vel.csv')
            csv_file = open(csv_path, 'w', newline='')
            writer = csv.writer(csv_file)
            writer.writerow(['timestamp', 'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'])
            csv_writers[topic] = writer
            csv_files[topic] = csv_file

        elif topic == '/imu':
            csv_path = os.path.join(csv_dir, 'imu.csv')
            csv_file = open(csv_path, 'w', newline='')
            writer = csv.writer(csv_file)
            writer.writerow([
                'timestamp',
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'
            ])
            csv_writers[topic] = writer
            csv_files[topic] = csv_file

        elif topic == '/scan':
            csv_path = os.path.join(csv_dir, 'scan.csv')
            csv_file = open(csv_path, 'w', newline='')
            writer = csv.writer(csv_file)
            # Header will include timestamp and ranges as range_0, range_1, ..., range_N
            # We'll define header after reading the first message to know the number of ranges
            csv_writers[topic] = writer
            csv_files[topic] = csv_file
            # Initialize a flag to indicate header not written yet
            csv_writers[topic].header_written = False

        elif topic == '/odom':
            csv_path = os.path.join(csv_dir, 'odom.csv')
            csv_file = open(csv_path, 'w', newline='')
            writer = csv.writer(csv_file)
            writer.writerow([
                'timestamp',
                'position_x', 'position_y', 'position_z',
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
                'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z'
            ])
            csv_writers[topic] = writer
            csv_files[topic] = csv_file

    # Iterate through the bag and write to CSV
    try:
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            if topic in msg_classes:
                msg = deserialize_message(data, msg_classes[topic])

                if topic == '/cmd_vel':
                    linear = msg.linear
                    angular = msg.angular
                    csv_writers[topic].writerow([
                        timestamp / 1e9,  # Convert nanoseconds to seconds
                        linear.x, linear.y, linear.z,
                        angular.x, angular.y, angular.z
                    ])

                elif topic == '/imu':
                    orientation = msg.orientation
                    angular_velocity = msg.angular_velocity
                    linear_acceleration = msg.linear_acceleration
                    csv_writers[topic].writerow([
                        timestamp / 1e9,
                        orientation.x, orientation.y, orientation.z, orientation.w,
                        angular_velocity.x, angular_velocity.y, angular_velocity.z,
                        linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
                    ])

                elif topic == '/scan':
                    # Only record ranges
                    ranges = msg.ranges  # List of float
                    if not csv_writers[topic].header_written:
                        header = ['timestamp'] + [f'range_{i}' for i in range(len(ranges))]
                        csv_writers[topic].writerow(header)
                        csv_writers[topic].header_written = True
                    row = [timestamp / 1e9] + ranges
                    csv_writers[topic].writerow(row)

                elif topic == '/odom':
                    pose = msg.pose.pose
                    twist = msg.twist.twist
                    csv_writers[topic].writerow([
                        timestamp / 1e9,
                        pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                        twist.linear.x, twist.linear.y, twist.linear.z,
                        twist.angular.x, twist.angular.y, twist.angular.z
                    ])
    except Exception as e:
        print(f"Error while processing bag {bag_file}: {e}")
    finally:
        # Close all CSV files
        for topic, csv_file in csv_files.items():
            csv_file.close()

    print(f"Finished processing bag {bag_file}. CSV files saved to {csv_dir}.")

def main():
    rclpy.init(args=None)

    # Directory where bag files are stored
    BAG_DIR = 'bag_files'  # Adjust this path if different

    # Directory to store CSV files
    CSV_DIR = os.path.join(BAG_DIR, 'extracted_data')

    # Create the CSV_DIR if it doesn't exist
    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)

    # Base name for bag folders
    BAG_BASE_NAME = 'cmd_vel_bag_'

    # Number of iterations (bag files)
    NUM_ITERATIONS = 15

    for i in range(1, NUM_ITERATIONS + 1):
        bag_folder = os.path.join(BAG_DIR, f"{BAG_BASE_NAME}{i}")

        if not os.path.exists(bag_folder):
            print(f"Bag folder {bag_folder} does not exist. Skipping.")
            continue

        # Path to the .db3 bag file
        # Assuming each bag folder contains a single .db3 file named as cmd_vel_bag_<i>.db3
        bag_db3_path = os.path.join(bag_folder, f"{BAG_BASE_NAME}{i}.db3")

        if not os.path.exists(bag_db3_path):
            print(f"Bag file {bag_db3_path} does not exist. Skipping.")
            continue

        # Extract messages and save to CSV
        extract_messages_from_bag(bag_db3_path, CSV_DIR)

    rclpy.shutdown()
    print("All bags processed and CSV files generated.")

if __name__ == "__main__":
    main()
