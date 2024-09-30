#!/usr/bin/env python3

import subprocess
import time
import os
import signal

# Number of iterations
NUM_ITERATIONS = 50

# Base name for bag files
BAG_BASE_NAME = "cmd_vel_bag"

# Directory to store bag files
BAG_DIR = 'square_path_normal_speed'

# Create the bag files directory if it doesn't exist
if not os.path.exists(BAG_DIR):
    os.makedirs(BAG_DIR)

for i in range(1, NUM_ITERATIONS + 1):
    print(f"Starting iteration {i} of {NUM_ITERATIONS}")

    try:
        # Start the first launch file: sim.nav.launch.py
        sim_cmd = [
            "ros2", "launch", "gazebo_garden_simulation_example", "sim.nav.launch.py"
        ]
        sim_process = subprocess.Popen(sim_cmd, preexec_fn=os.setsid)

        # Wait for the simulation to fully start
        time.sleep(8)  # Adjust the sleep duration as needed

        # Start the second launch file: nav.launch.py
        nav_cmd = [
            "ros2", "launch", "gazebo_garden_simulation_example", "nav.launch.py"
        ]
        nav_process = subprocess.Popen(nav_cmd, preexec_fn=os.setsid)

        # Wait for the navigation stack to fully start
        time.sleep(8)  # Adjust the sleep duration as needed

        # Start the waypoint follower and rosbag recording
        # Each time, save the bag file with a unique name in the specified directory
        BAG_NAME = os.path.join(BAG_DIR, f"{BAG_BASE_NAME}_{i}")
        waypoint_cmd = [
            "ros2",
            "launch",
            "waypoint_follower_node",
            "waypoint_follower_with_bag.launch.py",
            f"bag_name:={BAG_NAME}"
        ]
        waypoint_process = subprocess.Popen(waypoint_cmd)

        # Wait for the waypoint follower process to complete
        waypoint_process.wait()

    except Exception as e:
        print(f"An error occurred during iteration {i}: {e}")

    finally:
        # After waypoint follower finishes, stop the nav and sim processes
        # Kill the entire process group for nav_process
        os.killpg(os.getpgid(nav_process.pid), signal.SIGINT)
        nav_process.wait()

        # Kill the entire process group for sim_process
        os.killpg(os.getpgid(sim_process.pid), signal.SIGINT)
        sim_process.wait()

    # Wait a few seconds before the next iteration
    time.sleep(15)  # Adjust as needed

    print(f"Iteration {i} completed.")

print("Data collection completed.")
