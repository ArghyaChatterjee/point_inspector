import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.time import Time
from sensor_msgs_py import point_cloud2  # Used to create the PointCloud2 data
import rclpy
import os

def load_bin_data(pcl_file_path):
    # Read the binary point cloud data from file
    point_cloud = np.fromfile(pcl_file_path, dtype=np.float32).reshape(-1, 4)
    points = [
        [p[0], p[1], p[2], int(p[3] * 255) % 256, int(p[3] * 128) % 256, int(p[3] * 64) % 256]
        for p in point_cloud
    ]
    return points

def create_pointcloud_message(points, width, height, timestamp):
    # Create header
    header = Header()
    header.stamp = Time(seconds=timestamp // int(1e9), nanoseconds=timestamp % int(1e9)).to_msg()
    header.frame_id = 'left_camera_link'

    # Define point fields for x, y, z, r, g, b
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
        PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
        PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
    ]

    # Create the PointCloud2 message
    pointcloud_msg = point_cloud2.create_cloud(header, fields, points)

    # Set additional metadata
    pointcloud_msg.height = height
    pointcloud_msg.width = width
    pointcloud_msg.is_dense = True  # Assuming no invalid points
    pointcloud_msg.point_step = 15  # Each point is 15 bytes
    pointcloud_msg.row_step = pointcloud_msg.point_step * width

    return pointcloud_msg

def check_pointcloud_message(pointcloud_msg):
    # Calculate expected data size
    expected_data_size = pointcloud_msg.width * pointcloud_msg.height * pointcloud_msg.point_step
    actual_data_size = len(pointcloud_msg.data)

    print(f"Expected data size: {expected_data_size} bytes")
    print(f"Actual data size: {actual_data_size} bytes")

    # Check point attributes
    print(f"Width (number of points): {pointcloud_msg.width}")
    print(f"Height: {pointcloud_msg.height}")
    print(f"Point step (size of each point in bytes): {pointcloud_msg.point_step}")
    print(f"Row step (size of one row in bytes): {pointcloud_msg.row_step}")

    # Print fields for verification
    for field in pointcloud_msg.fields:
        print(f"Field: {field.name}, Offset: {field.offset}, Datatype: {field.datatype}, Count: {field.count}")

    # Check a sample of points in data
    print("Sample points from data (first 5 points):")
    point_step = pointcloud_msg.point_step
    for i in range(5):
        start_index = i * point_step
        print(pointcloud_msg.data[start_index:start_index + point_step])

def main(pcl_file_path):
    rclpy.init()

    # Load the binary data from the .bin file
    points = load_bin_data(pcl_file_path)

    # Create a PointCloud2 message
    pointcloud_msg = create_pointcloud_message(points, width=len(points), height=1, timestamp=1727984561992000000)

    # Run the diagnostics
    check_pointcloud_message(pointcloud_msg)

    rclpy.shutdown()

if __name__ == '__main__':
    # Provide the path to your .bin file here
    pcl_file_path = '/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_other_format/output/20241003_ZED_Mini_Recording_IHMC_robot_lab_room_exploration_res_1920x1080/pointcloud_kitti/data/left_000000.bin'
    main(pcl_file_path)
