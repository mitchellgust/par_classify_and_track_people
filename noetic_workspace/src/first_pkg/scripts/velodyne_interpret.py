#!/usr/bin/env python


import rospy
import open3d
import math
import itertools
import numpy as np

from datetime import datetime
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

"""
Capture velodyne data, remove noise, then publish the filtered_cloud for down stream processing. 
"""
class InterpretVelodyne:

    """
    Setup publisher and private variables.
    """
    def __init__(self):
        self.velodyne_subscriber = None
        self.last_update = 0
        self.update_frequency_seconds = 10
        self.filtered_point_cloud_publisher = rospy.Publisher('filtered_cloud', PointCloud2, queue_size=10)

    """
    Attempts to filter out the noise within the provided open3d PointCloud.
    Applies point cropping and radius based outlier removal techniques.
    return a filtered point cloud.
    """
    def filter_point_cloud(self, open3d_cloud : open3d.geometry.PointCloud) -> open3d.geometry.PointCloud:
        
        # Pass through filter to reduce noise. Needs to be tweaked for velodyne.
        # Only accept z points between min and max.
        bounds = [[-math.inf, math.inf], [-math.inf, math.inf], [-0.1,0.5]]
        bounding_box_points = list(itertools.product(*bounds))
        bounding_box = open3d.geometry.AxisAlignedBoundingBox.create_from_points(
            open3d.utility.Vector3dVector(bounding_box_points)) # Create bounding box object.
        
        # Crop the point cloud using the bounding box.
        pcd_cropped = open3d_cloud.crop(bounding_box)
        
        # Apply radius outlier removal that removes all points that 
        # don't have a specific number of points around it. 
        pcd_rad, ind_rad = pcd_cropped.remove_radius_outlier(nb_points=20, radius=0.05)
        outlier_rad_pcd = pcd_cropped.select_by_index(ind_rad, invert=True)
        
        # Apply statistical outlier removal filter.
        # This filter removes points that are further away from their
        # neighbours.
        #pcd_stat, ind_stat = outlier_rad_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=0.05)
        #outlier_stat_pcd = outlier_rad_pcd.select_by_index(ind_stat, invert=True)
        
        return outlier_rad_pcd

    """
    Convert open3d cloud to ros based PointCloud2.
    See - https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
    """
    def convert_cloud_from_open3d_to_ros(self, open3d_cloud, header):
        
        FIELDS_XYZ = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        header_new = Header()
        header_new.frame_id = header.frame_id
        header_new.stamp = header.stamp

        cloud_data = np.asarray(open3d_cloud.points)
        fields = FIELDS_XYZ
        
        return point_cloud2.create_cloud(header, fields, cloud_data)

    """
    Convert ros based PointCloud2 to open3d PointCloud.
    """
    def convert_ros_to_open3d(self, header, point_cloud_list):
        open3d_cloud = open3d.geometry.PointCloud()

        if len(point_cloud_list) == 0:
            return none

        xyz = [(x,y,z) for x,y,z,intensity,ring,time in point_cloud_list] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

        return open3d_cloud

    """
    Interprets the ROS based PointCloud2 data, converts it to an Open3d PointCloud, applies pass through filters to the data, converts it back to 
    ros based PointCloud2 and publishes it. 
    """
    def interpret_point_cloud_2(self, point_cloud_data : PointCloud2):
        
        seconds_since_last_update = datetime.today().timestamp() - self.last_update

        # if point_cloud_data is not None.
        if point_cloud_data is not None:
            
            # Update time since last update.
            self.last_update = datetime.today().timestamp()

            # Extract header information.
            header = point_cloud_data.header

            # Extract field information.
            field_names = list()
            fields = point_cloud_data.fields
            
            for point_field in fields:
                field_names.append(point_field.name)

            # Extract points.
            filtered_point_cloud_list = point_cloud2.read_points_list(point_cloud_data, field_names, skip_nans=True)
            
            # Apply filters to points.
            open3d_cloud = self.convert_ros_to_open3d(header, filtered_point_cloud_list)

            if open3d_cloud is not None:
                # Recreate point cloud - old way.
                # filtered_point_cloud = point_cloud2.create_cloud(header, fields, filtered_point_cloud_list)

                # Apply filters to point cloud.
                filtered_open3d_cloud = self.filter_point_cloud(open3d_cloud)

                # Convert back to ROS cloud.
                filtered_point_cloud = self.convert_cloud_from_open3d_to_ros(filtered_open3d_cloud, header)

                # Publish filtered cloud.
                self.filtered_point_cloud_publisher.publish(filtered_point_cloud)
    
    """
    Start subsribers.
    """
    def execute(self):
        self.velodyne_subscriber = rospy.Subscriber('/velodyne_points', PointCloud2, self.interpret_point_cloud_2)
        # self.projected_map_subscriber = rospy.Subscriber('/projected_map', OccupancyGrid, self.publish_map)
        rospy.spin()

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('velodyne_interpret', anonymous=True)
        interpret_velodyne = InterpretVelodyne()
        interpret_velodyne.execute()

    except rospy.ROSInterruptException:
        pass
