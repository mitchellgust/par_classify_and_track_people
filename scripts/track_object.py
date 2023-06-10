#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
from datetime import datetime

# Stores Data based on each Object Identified through Topic
identified_objects = []

class IdentifiedObject:    
    def __init__(self, object_id: int, position: Pose):
        self.object_id: int = object_id

        # List of Tracked Positions - (Pose, timestamp)
        self.tracked_positions = []
        self.tracked_positions.append(position)

        # Store Number of Tracked Positions
        self.num_tracked_positions: int = len(self.tracked_positions)

        self.last_position: Pose = None
        self.current_position: Pose = Pose

    def get_object_id(self) -> int:
        return self.object_id
    
    def get_tracked_positions(self) -> list:
        return self.tracked_positions
    
    def get_num_tracked_positions(self) -> int:
        return self.num_tracked_positions
    
    def add_tracked_position(self, tracked_position):
        self.num_tracked_positions += 1
        self.tracked_positions.append(tracked_position)
        self.last_position = tracked_position

    def get_timestamp_of_tracked_position(self, tracked_position) -> datetime:
        return tracked_position.timestamp

class TrackedPosition:
    def __init__(self, position: Pose, timestamp: datetime):
        self.position = position
        self.timestamp = timestamp

    def get_position(self):
        return self.position
    
    def get_timestamp(self):
        return self.timestamp

def viz_callback(markerArray: MarkerArray):
    # For each Marker in MarkerArray
    for marker in markerArray.markers:
        # If Marker is a New Object
        if marker.id not in identified_objects:
            # Create New Object
            identified_objects[marker.id] = IdentifiedObject(marker.id, marker.pose)
        # Else, Update Object
        else:
            identified_objects[marker.id].add_tracked_position(marker.pose)

def output_identified_objects():
    # For each Identified Object
    for object_id in identified_objects:
        rospy.loginfo("Object ID: " + object_id)
        # rospy.loginfo("Tracked Positions: " + identified_objects[object_id].get_tracked_positions())
        # rospy.loginfo("Num. of Tracked Positions: " + identified_objects[object_id].get_num_tracked_positions())
        # rospy.loginfo("Last Position: " + identified_objects[object_id].get_last_position())

        # Output difference between last and current position
        rospy.loginfo("Difference between last and current position: " + identified_objects[object_id].get_last_position() - identified_objects[object_id].get_current_position())


if __name__ == '__main__':
    try:
        rospy.init_node('track_viz_data', anonymous=True)

        # Drive publisher
        pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscribe to MarkerArray
        rospy.Subscriber("/viz", MarkerArray, viz_callback)

        # Output the Identified Objects and their Tracked Positions every 5 seconds
        while not rospy.is_shutdown():
            output_identified_objects()
            rospy.sleep(5)

    except rospy.ROSInterruptException:
        pass
        
