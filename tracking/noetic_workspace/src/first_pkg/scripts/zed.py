import rospy
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Twist
import math

# Define marker publisher as a global variable
marker_pub = None
closest_marker_id = None
# Dictionary to store the marker positions for each label ID
marker_positions = {}
# Keep track of the last update time for each marker
marker_times = {}

# Drive publisher
pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


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
            # Update the marker position for the current object
            marker_positions[label_id] = position
            marker_times[label_id] = current_time
            updated_markers.append(label_id)

    # Remove markers that have not been updated
    for label_id in list(marker_positions.keys()):  # Use list() to prevent RuntimeError due to changing size during iteration
        if label_id not in updated_markers and (current_time - marker_times[label_id]).to_sec() > 5.0:  # Change the threshold time as per your requirement
            del marker_positions[label_id]
            del marker_times[label_id]
            remove_marker(label_id)

    # Publish the markers
    publish_markers()
    rotate_to_closest_marker()

def rotate_to_closest_marker():
    global closest_marker_id

    if closest_marker_id is None or closest_marker_id not in marker_positions:
        closest_marker_id = find_closest_marker()

    if closest_marker_id is not None:
        angle = compute_angle_to_marker(marker_positions[closest_marker_id])

        # Now you can use this angle to drive your robot
        cmd = Twist()
        # Define maximum rotation speed
        MAX_ROT_SPEED = 1.0  # radians/second

        # Scale angle to limit maximum rotation speed
        cmd.angular.z = max(min(angle, MAX_ROT_SPEED), -MAX_ROT_SPEED)
        pub_drive.publish(cmd)

def compute_angle_to_marker(position):
    return math.atan2(position[1], position[0])

def find_closest_marker():
    closest_marker_id = None
    closest_marker_distance = float('inf')

    for marker_id, position in marker_positions.items():
        distance = math.hypot(position[0], position[1])  # Compute distance to marker
        if distance < closest_marker_distance:
            closest_marker_distance = distance
            closest_marker_id = marker_id

    return closest_marker_id

def publish_markers():
    for label_id, position in marker_positions.items():
        # Create a marker for each object
        marker = Marker()
        marker.header.frame_id = "map"  # Set the frame ID of the marker to match your map's frame ID
        marker.id = label_id  # Assign a unique marker ID based on the label ID
        marker.type = Marker.SPHERE  # Set the marker type to represent a sphere
        marker.pose.position.x = position[0]  # Set the position of the marker
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Initialize quaternion to identity orientation
        marker.scale.x = 0.2  # Set the size of the marker
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Set the transparency of the marker
        marker.color.r = 1.0  # Set the color of the marker (e.g., red)
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        marker_pub.publish(marker)

        # Create a text marker for the label ID
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.id = label_id + 1000  # Assign a unique marker ID based on the label ID, add 1000 to avoid ID clash
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.position.x = position[0]
        text_marker.pose.position.y = position[1]
        text_marker.pose.position.z = position[2] + 0.2  # Offset to place the text above the corresponding sphere marker
        text_marker.text = str(label_id)
        text_marker.scale.z = 0.1  # Text size
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0

        # Publish the text marker
        marker_pub.publish(text_marker)

def remove_marker(label_id):
    # Remove the sphere marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = label_id
    marker.action = Marker.DELETE  # This action removes the marker
    marker_pub.publish(marker)

    # Remove the text marker
    text_marker = Marker()
    text_marker.header.frame_id = "map"
    text_marker.id = label_id + 1000  # Use the same ID that you used for the text marker
    text_marker.action = Marker.DELETE
    marker_pub.publish(text_marker)

def listener():
    global marker_pub
    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("zed2i/zed_node/obj_det/objects", ObjectsStamped, callback)

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
