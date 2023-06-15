import rospy
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Twist
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# Define marker publisher as a global variable
# Define closest marker id as a global variable
marker_pub = None
closest_marker_id = None
# Dictionary to store the marker positions
marker_positions = {}
# Keep track of the last update time for each marker
marker_times = {}

pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


# Get all the id/label/confidence from data objects
# Update the marker position for the current object
# Remove markers that have not been updated
# Publish the markers
def callback(data):
    global closest_marker_id

    current_time = rospy.get_rostime()
    updated_markers = []

    for obj in data.objects:
        label = obj.label
        label_id = obj.label_id
        sublabel = obj.sublabel
        confidence = obj.confidence
        position = obj.position
        tracking_state = obj.tracking_state

        if label == "Person" and confidence >= 60.0:
            marker_positions[label_id] = position
            marker_times[label_id] = current_time
            updated_markers.append(label_id)

    for label_id in list(marker_positions.keys()): 
        if label_id not in updated_markers and (current_time - marker_times[label_id]).to_sec() > 5.0:
            del marker_positions[label_id]
            del marker_times[label_id]
            remove_marker(label_id)

    publish_markers()
    rotate_to_closest_marker()

# rotate the robot to th
# Scale angle to limit maximum rotation speed
# Move based to marker
def rotate_to_closest_marker():
    global closest_marker_id

    if closest_marker_id is None or closest_marker_id not in marker_positions:
        closest_marker_id = find_closest_marker()

    if closest_marker_id is not None:
        angle = compute_angle_to_marker(marker_positions[closest_marker_id])

        cmd = Twist()
        MAX_ROT_SPEED = 1.0
        cmd.angular.z = max(min(angle, MAX_ROT_SPEED), -MAX_ROT_SPEED)

        pub_drive.publish(cmd)
        move_to_marker(marker_positions[closest_marker_id])

# move the robot the the closest marker and follow it
# Reduce position to be a meter away
def move_to_marker(position):
    global move_base_client

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    position = [position[0] - 1.0, position[1] - 1.0]

    goal.target_pose.pose.position.x = position[0]
    goal.target_pose.pose.position.y = position[1]

    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()
    print("Moving to Person. WATCH OUT!!")

#get the angle between the robot and marker
def compute_angle_to_marker(position):
    return math.atan2(position[1], position[0])

# get the closest marker
def find_closest_marker():
    closest_marker_id = None
    closest_marker_distance = float('inf')

    for marker_id, position in marker_positions.items():
        distance = math.hypot(position[0], position[1])
        if distance < closest_marker_distance:
            closest_marker_distance = distance
            closest_marker_id = marker_id

    return closest_marker_id

# Create a marker for each object
# Set the frame ID of the marker to match your map's frame ID
def publish_markers():
    for label_id, position in marker_positions.items():
        marker = Marker()
        marker.header.frame_id = "map" 
        marker.id = label_id  
        marker.type = Marker.SPHERE  
        marker.pose.position.x = position[0]  
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0) 
        marker.scale.x = 0.2 
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  
        marker.color.r = 1.0  
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker_pub.publish(marker)

        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.id = label_id + 1000  
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.position.x = position[0]
        text_marker.pose.position.y = position[1]
        text_marker.pose.position.z = position[2] + 0.2  
        text_marker.text = str(label_id)
        text_marker.scale.z = 0.1  
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0

        marker_pub.publish(text_marker)

# Remove the both marker
def remove_marker(label_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = label_id
    marker.action = Marker.DELETE
    marker_pub.publish(marker)

    text_marker = Marker()
    text_marker.header.frame_id = "map"
    text_marker.id = label_id + 1000 
    text_marker.action = Marker.DELETE
    marker_pub.publish(text_marker)

def listener():
    global marker_pub

    print("Starting ZED Marker Maker")
    
    rospy.init_node('listener', anonymous=True)

    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    
    rospy.Subscriber("zed2i/zed_node/obj_det/objects", ObjectsStamped, callback)

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
