#!/usr/bin/env python

import rospy
from datetime import datetime
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
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

    def interpret_point_cloud_2(self, point_cloud_data : PointCloud2):
        
        seconds_since_last_update = datetime.today().timestamp() - self.last_update

        if point_cloud_data is not None:
            
            # for p in filtered_cloud_point2:
            #     print(f'x: {p[0]}, y: {p[1]}, z: {p[2]}')            
            
            # Update time since last update.
            self.last_update = datetime.today().timestamp()

            # Filter out points.
            filtered_point_cloud_list = point_cloud2.read_points_list(point_cloud_data, field_names = ("x", "y", "z"), skip_nans=True)
            
            # Recreate point cloud.
            # header = Header()
            header = point_cloud_data.header                        # Field names included in list.
            filtered_point_cloud = point_cloud2.create_cloud_xyz32(header, filtered_point_cloud_list)
            
            self.filtered_point_cloud_publisher.publish(filtered_point_cloud)

    """
    Start subsribers.
    """
    def execute(self):
        self.velodyne_subscriber = rospy.Subscriber('/velodyne_points', PointCloud2, self.interpret_point_cloud_2)
        rospy.spin()

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('velodyne_interpret', anonymous=True)
        interpret_velodyne = InterpretVelodyne()
        interpret_velodyne.execute()

    except rospy.ROSInterruptException:
        pass