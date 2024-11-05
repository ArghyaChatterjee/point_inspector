import struct
import numpy as np

# Path to a sample .bin file
bin_file_path = '/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/20241003_ZED_Mini_Recording_IHMC_robot_lab_room_exploration_res_1920x1080/pointcloud_kitti/data/left_000000.bin'

def inspect_bin_file(file_path):
    points = []
    with open(file_path, 'rb') as f:
        while True:
            data = f.read(15)  # Each point is 15 bytes
            if len(data) < 15:
                break
            
            # Unpack x, y, z (3 floats) and rgb (1 float)
            x, y, z, rgb = struct.unpack('fffI', data[:12] + struct.pack('I', int.from_bytes(data[12:], 'little')))
            r = (rgb >> 16) & 0xFF
            g = (rgb >> 8) & 0xFF
            b = rgb & 0xFF
            points.append((x, y, z, r, g, b))
    
    print(f"Total points: {len(points)}")
    print("Sample points:")
    for i, point in enumerate(points[:5]):  # Print first 5 points
        print(f"Point {i}: x={point[0]}, y={point[1]}, z={point[2]}, r={point[3]}, g={point[4]}, b={point[5]}")

inspect_bin_file(bin_file_path)
