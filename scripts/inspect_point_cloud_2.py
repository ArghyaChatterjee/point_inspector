import struct

# Path to the .bin file
bin_file_path = '/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/20241003_ZED_Mini_Recording_IHMC_robot_lab_room_exploration_res_1920x1080/pointcloud_kitti/data/left_000000.bin'

def inspect_bin_file(file_path):
    points = []
    with open(file_path, 'rb') as f:
        while True:
            data = f.read(16)  # Expecting 4 floats (x, y, z, rgba)
            if len(data) < 16:
                break
            
            # Read data in little-endian format
            x, y, z, rgba = struct.unpack('<ffff', data)  
            rgba_int = int(rgba) if rgba.is_integer() else struct.unpack('<I', struct.pack('<f', rgba))[0]
            
            # Extract RGB values from rgba_int
            r = (rgba_int >> 16) & 0xFF
            g = (rgba_int >> 8) & 0xFF
            b = rgba_int & 0xFF
            
            points.append((x, y, z, r, g, b))
            if r != 0 or g != 0 or b != 0:
                print(f"Non-zero RGB found at point {len(points)-1}: x={x}, y={y}, z={z}, r={r}, g={g}, b={b}")

    print(f"Total points: {len(points)}")
    print("Sample points:")
    for i, point in enumerate(points[:5]):  # Print first 5 points
        print(f"Point {i}: x={point[0]}, y={point[1]}, z={point[2]}, r={point[3]}, g={point[4]}, b={point[5]}")

inspect_bin_file(bin_file_path)
