import numpy as np
import open3d as o3d

def read_ply_file(file_path):
    # Load the point cloud from the .ply file
    pcd = o3d.io.read_point_cloud(file_path)
    
    # Convert the point cloud data to a numpy array
    points = np.asarray(pcd.points)
    has_colors = pcd.has_colors()
    colors = np.asarray(pcd.colors) if has_colors else None
    
    num_points = len(points)
    print(f"File: {file_path} | Number of points: {num_points}")
    print("First 2000 points:")

    for i in range(min(2000, num_points)):
        x, y, z = points[i]
        if has_colors:
            r, g, b = (colors[i] * 255).astype(int)  # Scale from [0, 1] to [0, 255]
            print(f"Point {i}: x={x:.4f}, y={y:.4f}, z={z:.4f}, r={r}, g={g}, b={b}")
        else:
            print(f"Point {i}: x={x:.4f}, y={y:.4f}, z={z:.4f}")

# Specify the path to the .ply file
file_path = '/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/20241003_ZED_Mini_Recording_IHMC_robot_lab_room_exploration_res_1920x1080/pointcloud_ply/left_000000.ply'
read_ply_file(file_path)

