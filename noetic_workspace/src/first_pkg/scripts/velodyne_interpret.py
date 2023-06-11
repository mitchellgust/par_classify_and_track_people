#!/usr/bin/env python


import rospy
#import open3d as o3d
import numpy as np
#import pclpy
#from pclpy import pcl
from datetime import datetime
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

# Goals - 
# 1. Register when object enters within a certain distance.
# 2. Register object deviation from center 

class InterpretVelodyne:
    def __init__(self):
        self.velodyne_subscriber = None
        self.last_update = 0
        self.update_frequency_seconds = 10
        self.filtered_point_cloud_publisher = rospy.Publisher('filtered_cloud', PointCloud2, queue_size=10)
        self.map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=10)

    # # Techniques taken from here: https://betterprogramming.pub/point-cloud-filtering-in-python-e8a06bbbcee5
    # def remove_noise_from_point_cloud_2(self, filtered_cloud):
        
    #     xyz = np.array([[0,0,0]])
    #     rgb = np.array([[0,0,0]])

    #     # Data conversion from sensor_msgs pointcloud to o3d pointcloud: https://answers.ros.org/question/255351/how-o-save-a-pointcloud2-data-in-python/
    #     for x in filtered_cloud:
    #         test = x[3]

    #         # cast float32 to int so that bitwise operations are possible
    #         s = struct.pack('>f' ,test)
    #         i = struct.unpack('>l',s)[0]
            
    #         # you can get back the float value by the inverse operations
    #         pack = ctypes.c_uint32(i).value
    #         r = (pack & 0x00FF0000)>> 16
    #         g = (pack & 0x0000FF00)>> 8
    #         b = (pack & 0x000000FF)
            
    #         # prints r,g,b values in the 0-255 range
    #                     # x,y,z can be retrieved from the x[0],x[1],x[2]
    #         xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
    #         rgb = np.append(rgb,[[r,g,b]], axis = 0)
        
    #     out_pcd = o3d.geometry.PointCloud()
    #     out.pcd.points = o3d.utility.Vector3dVector(xyz)
    #     out_pcd.colors = o3d.utility.Vector3dVector(rgb)

    #     # Create bounding box:
    #     bounds = [[-math.inf, math.inf], [-math.inf, math.inf], [0.8, 2]]  # set the bounds
    #     bounding_box_points = list(itertools.product(*bounds))  # create limit points
    #     bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
    #         o3d.utility.Vector3dVector(bounding_box_points))  # create bounding box object

    #     # Crop the point cloud using the bounding box:
    #     pcd_croped = pcd.crop(bounding_box)
        
    #     return out_pcd.points


    def interpret_point_cloud_2(self, point_cloud_data : PointCloud2):
        
        seconds_since_last_update = datetime.today().timestamp() - self.last_update

        # if point_cloud_data is not None and seconds_since_last_update > 10:
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

            # Recreate point cloud.
            filtered_point_cloud = point_cloud2.create_cloud(header, fields, filtered_point_cloud_list)

            # Publish filtered cloud.
            self.filtered_point_cloud_publisher.publish(filtered_point_cloud)

            # Recreate point cloud.
            # header = Header()
            # header = point_cloud_data.header                        # Field names included in list.
            # filtered_point_cloud = point_cloud2.create_cloud_xyz32(header, filtered_point_cloud_list)
            
            # self.filtered_point_cloud_publisher.publish(filtered_point_cloud)

            # std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
            # pcl::removeNaNFromPointCloud(*source_cloud, *indices);
            # pcl::ExtractIndices<pcl::PointXYZ> extract;
            # extract.setInputCloud(source_cloud);
            # extract.setIndices(indices);
            # extract.setNegative(true);
            # extract.filter(*source_cloud);

            # out = self.remove_noise_from_point_cloud_2(filtered_cloud)
            # self.filtered_point_cloud_publisher.publish(point_cloud_data)

    
    def publish_map(self, occupancy_grid : OccupancyGrid):
        if occupancy_grid is not None:
            self.map_publisher.publish(occupancy_grid)
    
    """
    Start subsribers.
    """
    def execute(self):
        self.velodyne_subscriber = rospy.Subscriber('/velodyne_points', PointCloud2, self.interpret_point_cloud_2)
        self.projected_map_subscriber = rospy.Subscriber('/projected_map', OccupancyGrid, self.publish_map)
        rospy.spin()

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('velodyne_interpret', anonymous=True)
        interpret_velodyne = InterpretVelodyne()
        interpret_velodyne.execute()

    except rospy.ROSInterruptException:
        pass
