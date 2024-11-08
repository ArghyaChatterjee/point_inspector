import os
import numpy as np

def count_points_in_bin_files(directory):
    bin_files = [f for f in os.listdir(directory) if f.endswith('.bin')]
    total_points = 0
    
    for bin_file in bin_files:
        file_path = os.path.join(directory, bin_file)
        # Assuming each point is represented as (x, y, z, r, g, b) in float32 and uint8 format
        points = np.fromfile(file_path, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        num_points = len(points)
        print(f"{bin_file}: {num_points} points")
        total_points += num_points

    print(f"Total points across all .bin files: {total_points}")

# Specify the directory containing the .bin files
directory = '/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/20241003_ZED_Mini_Recording_IHMC_robot_lab_room_exploration_res_1920x1080/pointcloud_kitti/data/'
count_points_in_bin_files(directory)
