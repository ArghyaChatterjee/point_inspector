import numpy as np
import struct
import os

def read_bin_file(file_path):
    # Assuming each point is represented as (x, y, z, r, g, b) in float32 and uint8 format
    points = np.fromfile(file_path, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    num_points = len(points)
    print(f"File: {file_path} | Number of points: {num_points}")
    print("First 2000 points:")
    for i in range(min(2000, num_points)):
        point = points[i]
        print(f"Point {i}: x={point['x']}, y={point['y']}, z={point['z']}, r={point['r']}, g={point['g']}, b={point['b']}")

# Specify the path to the .bin file
file_path = '/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/20241003_ZED_Mini_Recording_IHMC_robot_lab_room_exploration_res_1920x1080/pointcloud_kitti/data/left_000000.bin'
read_bin_file(file_path)



# def count_points_in_bin_file(file_path):
#     # Define the data type for each point: (x, y, z, rgb) where rgb is a packed float value
#     # Each point has (3 floats for x, y, z) + (1 float for rgb) = 16 bytes
#     point_dtype = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'f4')])

#     # Read the binary file
#     with open(file_path, 'rb') as f:
#         data = f.read()
    
#     # Calculate the number of points
#     num_points = len(data) // point_dtype.itemsize

#     # Load the binary data into a structured numpy array
#     points = np.frombuffer(data, dtype=point_dtype)

#     # Print the total number of points
#     print(f"File: {file_path} | Number of points: {num_points}")

#     # Optional: Print first few points to verify the data
#     print("First 5 points:")
#     for i in range(min(5, num_points)):
#         x, y, z, rgb = points[i]
#         print(f"Point {i}: x={x}, y={y}, z={z}, rgb={rgb}")

# # Specify the path to the .bin file
# file_path = '/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/20241003_ZED_Mini_Recording_IHMC_robot_lab_room_exploration_res_1920x1080/pointcloud_kitti_unfiltered/data/left_000000.bin'
# count_points_in_bin_file(file_path)

