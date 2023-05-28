#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from datetime import datetime

# TODO: Not tested AT ALL. Just started planning the structure of the script

# Stores Data based on each Object Identified through Topic
IdentifiedObject: identified_objects = []

# Holds Currently Tracked Object ID
tracked_object_id = None 

# Num. of Times an Object is Seen before Average is Produced
# - Use this to be confident on its movement.
POSITIONS_BEFORE_AVERAGE_IS_MADE = 3

class IdentifiedObject:    
    def __init__(self, object_id, position):
        self.object_id = object_id

        # List of Tracked Positions - (Pose, timestamp, probability)
        self.tracked_positions = []
        self.tracked_positions.append(position)

        # Track Number of Tracked Positions
        self.num_tracked_positions = len(tracked_positions)

        # Previous Average of x Most Recent Position - (Pose)
        self.previous_position_average = None

        # Average of x Most Recent Positions - (Pose)
        self.latest_position_average = None # Produce Average on Every 5 Reading

    def get_object_id(self):
        return self.object_id
    
    def get_latest_position_average(self):
        return self.latest_position_average
    
    def get_tracked_positions(self):
        return self.tracked_positions
    
    def get_num_tracked_positions(self):
        return self.num_tracked_positions
    
    def add_tracked_position(self, tracked_position):
        self.num_tracked_positions += 1
        self.tracked_positions.append(tracked_position)
    
    def calc_average_from_tracked_positions(self, object_id):
        x_average = 0
        x_values = []

        y_average = 0
        y_values = []

        z_average = 0
        z_values = []

        # Sort Tracked Positions by Timestamp
        self.tracked_positions = sorted(self.tracked_positions, key=self.get_timestamp_of_tracked_position)

        for index, position in enumerate(self.tracked_positions):
            x_values.append(position.x)
            y_values.append(position.y)
            z_values.append(position.z)

            # Only Get x Most Recent Positions
            if index + 1 == POSITIONS_BEFORE_AVERAGE_IS_MADE:
                break
        
        # Get the average of the x Most Recent Position Values (x,y,z)
        x_average = sum(x_values) / len(x_values)
        y_average = sum(y_values) / len(y_values)
        z_average = sum(z_values) / len(z_values)

        self.previous_position_average = self.latest_position_average

        self.latest_position_average = Pose(x_average, y_average, z_average)

    def get_timestamp_of_tracked_position(self, tracked_position):
        return tracked_position.timestamp

class TrackedPosition:
    def __init__(self, Pose: position, datetime: timestamp, int: confidence):
        self.position = position
        self.timestamp = timestamp
        self.confidence = confidence

def found_object_callback(found_object):

    # Get objectID, trackedPosition
    # If Exists
    if identified_objects.contains(found_object.object_id):
        # TODO: Update Object based on Object_id
        identified_objects.update()

    # If Doesnt Exists
    else:
        # Create New Object
        identified_objects.append(IdentifiedObject(found_object.object_id, tracked_position))


    # If NumTrackPositions > 5
    # Calculate latest_position_average
    # Update latest_position average value in identified_objects list

    # If latest tracked position of identified_object_id is not within 10 seconds of current time, clear identified_object_id

    # If identified_object_id is -1
    # Set identified_object_id to objectID


# Use tracked_object_id to determine which object to track
# Determine if object is going left or right
# Perform Rotation
# def follow_object():


def rotate_left():
    # Rotate Left
    cmd.angular.z = speed
    pub_drive.publish(cmd)
    rate.sleep()

    # Stop Robot
    cmd.angular.z = 0
    pub_drive.publish(cmd)

def rotate_right():
    # Rotate Right
    cmd.angular.z = -speed
    pub_drive.publish(cmd)
    rate.sleep()

    # Stop Robot
    cmd.angular.z = 0
    pub_drive.publish(cmd)
     

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('track_object_by_rotating', anonymous=True)
       
        # Initiate Twist parameters
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0

        # Drive publisher
        pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Paramaters
        speed = rospy.get_param("~speed", 0.5)
        max_duration = rospy.get_param("~duration", 3)
        t_start = rospy.Time.now()
        stop = False

        rate = rospy.Rate(10)

        # Subscribe to Data Passed from Lidar Object Detection Node
        rospy.Subscriber("/identified_objects", FoundObject, found_object_callback)

    except rospy.ROSInterruptException:
        pass