#!/usr/bin/env python

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from pcl import PointCloud
import pcl_conversions
from pcl_msgs.msg import PointCloud2 as PCL2

class MapBuilder:
    def __init__(self):
        rospy.init_node('map_builder_node')
        self.pcl_sub = rospy.Subscriber('velodyne_points', PointCloud2, self.point_cloud_callback)
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1)
    
    def point_cloud_callback(self, msg):
        pcl_cloud = pcl.PointCloud()
        pcl_conversions.fromPCL(msg, pcl_cloud)
        
        # Perform your preprocessing, registration, and mapping steps using PCL
        # ...

        # Convert the point cloud back to sensor_msgs/PointCloud2 format
        pcl2_cloud = PCL2()
        pcl_conversions.fromPCL(pcl_cloud, pcl2_cloud)
        pcl2_cloud.header = msg.header

        # Set the map resolution, width, and height appropriately
        map_resolution = 0.1  # Set the desired resolution in meters
        map_width = 1000  # Set the desired width in cells
        map_height = 1000  # Set the desired height in cells
        occupancy_grid_data = [0] * (map_width * map_height)

        # Set the origin of the map in the global coordinate frame
        map_origin_x = 0.0  # Set the x-coordinate of the map's origin in the global frame
        map_origin_y = 0.0  # Set the y-coordinate of the map's origin in the global frame

        # Set the threshold height for considering a point as occupied
        threshold_height = 1.0  # Set the desired threshold height in meters

        # Determine which cells are occupied or unoccupied based on a threshold
        for point in pcl_cloud.points:
            # Convert point coordinates to grid indices
            grid_x = int((point.x - map_origin_x) / map_resolution)
            grid_y = int((point.y - map_origin_y) / map_resolution)

            # Check if the grid indices are within the map boundaries
            if 0 <= grid_x < map_width and 0 <= grid_y < map_height:
                # Set the cell as occupied if the point meets the threshold condition
                if point.z > threshold_height:
                    occupancy_grid_data[grid_y * map_width + grid_x] = 100

        # Publish the map as an OccupancyGrid
        map_msg = OccupancyGrid()
        map_msg.header = msg.header
        map_msg.info.resolution = map_resolution
        map_msg.info.width = map_width
        map_msg.info.height = map_height
        map_msg.info.origin.position.x = map_origin_x  # Set the x-coordinate of the map's origin
        map_msg.info.origin.position.y = map_origin_y  # Set the y-coordinate of the map's origin
        map_msg.data = occupancy_grid_data

        self.map_pub.publish(map_msg)

if __name__ == '__main__':
    map_builder = MapBuilder()
    rospy.spin()
