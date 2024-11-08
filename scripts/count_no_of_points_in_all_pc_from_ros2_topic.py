import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import deserialize_message

def count_points_in_rosbag(bag_file_path, topic_name):
    rclpy.init()
    reader = SequentialReader()
    
    storage_options = StorageOptions(uri=bag_file_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "cdr")
    reader.open(storage_options, converter_options)

    topic_found = False
    total_points = 0

    # Loop through all messages in the bag file
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()

        # Process only the messages from the specified point cloud topic
        if topic == topic_name:
            topic_found = True
            
            # Deserialize the data into a PointCloud2 message
            point_cloud_msg = deserialize_message(data, PointCloud2)

            # Count the points in the PointCloud2 message
            num_points = sum(1 for _ in point_cloud2.read_points(point_cloud_msg))
            print(f"Timestamp: {timestamp}, {topic}: {num_points} points")
            total_points += num_points

    if topic_found:
        print(f"Total points across all messages in topic '{topic_name}': {total_points}")
    else:
        print(f"Topic '{topic_name}' not found in the bag file.")

    rclpy.shutdown()


# Specify the path to the bag file and the topic name
bag_file_path = '/home/arghya/centerpose_training/scripts/perception-data-logger/convert_svo2_to_ros2bags/scripts/zed_pose_data_new2.bag'
topic_name = '/zed/pointcloud'
count_points_in_rosbag(bag_file_path, topic_name)
