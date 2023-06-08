import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def callback(data):
    # Access point cloud data
    gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)

    # Convert point cloud to numpy array
    points = np.array(list(gen))

    # Define grid parameters
    grid_size = 1.0  # grid cell size in meters
    grid_width = int((points[:, 0].max() - points[:, 0].min()) / grid_size)
    grid_height = int((points[:, 1].max() - points[:, 1].min()) / grid_size)

    # Initialize occupancy grid
    grid = np.zeros((grid_height, grid_width), dtype=np.int8)

    # Populate occupancy grid
    for point in points:
        x, y, _ = point
        grid_x = int((x - points[:, 0].min()) / grid_size)
        grid_y = int((y - points[:, 1].min()) / grid_size)
        grid[grid_y, grid_x] = 100  # Occupied

    # Convert occupancy grid to OccupancyGrid message
    occupancy_grid_msg = OccupancyGrid()
    occupancy_grid_msg.data = grid.flatten().tolist()
    occupancy_grid_msg.info.resolution = grid_size
    occupancy_grid_msg.info.width = grid_width
    occupancy_grid_msg.info.height = grid_height

    # Publish OccupancyGrid message
    grid_pub.publish(occupancy_grid_msg)

# Initialize ROS node
rospy.init_node('pointcloud_to_grid')

# Create publisher
grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

# Create subscriber
rospy.Subscriber('/velodyne_points', PointCloud2, callback)

# Spin
rospy.spin()
