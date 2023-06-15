import rospy
import math

from std_msgs.msg import (
    String
)

from geometry_msgs.msg import Quaternion, Twist
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

import actionlib

class Tracker:

    """
    Setup tracking object.
    """
    def __init__(self):
        # Subscribers.
        self.velodyne_track_id_subscriber = None
        self.velodyne_marker_subscriber = None
        self.zed_marker_subscriber = None

        # Gates.
        self.track_velodyne_object = False
        self.track_zed_object = False

        # Track id.
        self.track_id_velodyne = -1

        # Output.
        self.out_publisher = rospy.Publisher('tracking_out', String, queue_size=10)
        self.pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    """
    Issues commands to begin moving the bot based on the markers 
    published by the velodyne interpreter.
    Usage: rostopic pub /tracker_id std_msgs/String "data: '0'" 
    """
    def track_object(self, object_id : String):
        
        # Convert object_id to int. 
        self.track_id_velodyne = int(object_id.data)

        self.out_publisher.publish(object_id)

        # Begin tracking via velodyne.
        self.track_velodyne_object = True

        
    """
    Starts issuing move commands to a designated object id based on ids designated
    by the marker array.
    """
    def track_velodyne_marker(self, marker_array : MarkerArray):

        if marker_array is not None:
            markers = marker_array.markers

            self.out_publisher.publish("Track velodyne marker hit with marker len" + str(len(markers)))

            # Figure out when to run tracking.
            if self.track_velodyne_marker and self.track_id_velodyne > -1:
                
                # Disable tracking.
                self.track_velodyne_marker = False

                # Check if array contains marker index.
                if len(markers) > self.track_id_velodyne:
                    
                    self.out_publisher.publish("Track Velodyne")

                    self.rotate_to_closest_marker(markers[self.track_id_velodyne])
                    self.track_velodyne_marker = False
                    self.track_zed_object = True

                else:
                    self.track_velodyne_marker = True


    """
    Issues a command to begin tracking through the zed?
    """
    def track_zed_marker(self, marker : Marker):
        pass

    def rotate_to_closest_marker(self, marker : Marker):
    

        if marker is not None:

            # Now you can use this angle to drive your robot
            cmd = Twist()
            
            # Define maximum rotation speed
            MAX_ROT_SPEED = 1.0  # radians/second
            position = [marker.pose.position.x - 1.0,marker.pose.position.y - 1.0]
            
            angle = math.atan2(position[1], position[0])

            # Scale angle to limit maximum rotation speed
            cmd.angular.z = max(min(angle, MAX_ROT_SPEED), -MAX_ROT_SPEED)

            # # Determine distance to closest marker
            # distance = math.hypot(marker_positions[closest_marker_id][0], marker_positions[closest_marker_id][1])

            # # Make it stay a meter away from the marker
            # distance -= 1.0

            # # MAX Linear Speed
            # MAX_LIN_SPEED = 0.1  # meters/second

            # # Scale distance to limit maximum linear speed
            # cmd.linear.x = max(min(distance, MAX_LIN_SPEED), -MAX_LIN_SPEED)

            pub_drive.publish(cmd)

    """
    Publish a goal to move the bot to.
    """
    def move_bot(self, marker : Marker):
        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = marker.header.frame_id
        # goal.target_pose.header.stamp = marker.header.stamp

        # # Reduce position to be a meter away
        position = [marker.pose.position.x - 1.0,marker.pose.position.y - 1.0]

        self.out_publisher.publish("x: " + str(marker.pose.position.x) + " y: " + str(marker.pose.position.y))

        # goal.target_pose.pose.position.x = position[0]
        # goal.target_pose.pose.position.y = position[1]

        # self.move_base_client.send_goal(goal)
        # self.move_base_client.wait_for_result()

        pub_gotopose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        map_frame = rospy.get_param("~map_frame", 'map')
        robot_frame = rospy.get_param("~robot_frame", '/base_link')

        target_pose = PoseStamped()
        target_pose.header.frame_id = map_frame
        target_pose.header.stamp = marker.header.stamp
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.orientation.w = 1

        pub_gotopose.publish(target_pose)

        self.out_publisher.publish("done")

        rate = rospy.Rate(1)


    """
    Initiate subscribers and begin tracking.
    """
    def execute(self):
        self.track_id_subscriber = rospy.Subscriber("/tracker_id", String, self.track_object)
        self.velodyne_marker_subscriber = rospy.Subscriber("/viz", MarkerArray, self.track_velodyne_marker)
        self.zed_marker_subscriber = rospy.Subscriber("/visualization_marker", Marker, self.track_zed_marker)

        rospy.spin()

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('tracker', anonymous=True)
        tracker = Tracker()
        tracker.execute()

    except rospy.ROSInterruptException:
        pass