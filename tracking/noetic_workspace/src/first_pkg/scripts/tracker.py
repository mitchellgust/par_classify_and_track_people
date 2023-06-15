import rospy

from std_msgs.msg import (
    String
)

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

        # Move command.
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Output.
        self.out_publisher = rospy.Publisher('tracking_out', String, queue_size=10)

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

                    self.move_bot(markers.marker[self.track_id_velodyne])
                    self.track_velodyne_marker = False
                    self.track_zed_object = True

                else:
                    self.track_velodyne_marker = True


    """
    Issues a command to begin tracking through the zed?
    """
    def track_zed_marker(self, marker : Marker):
        pass

    """
    Publish a goal to move the bot to.
    """
    def move_bot(self, marker : Marker):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = marker.header.frame_id
        goal.target_pose.header.stamp = marker.header.stamp

        # Reduce position to be a meter away
        position = [marker.pose.position.x - 1.0,marker.pose.position.y - 1.0]

         self.out_publisher.publish("x: " + str(marker.pose.position.x) + " y: " + str(marker.pose.position.y))

        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

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