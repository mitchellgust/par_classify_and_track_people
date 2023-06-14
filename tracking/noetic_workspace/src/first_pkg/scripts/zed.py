import rospy
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
import time

# Define marker publisher as a global variable
marker_pub = None
# Dictionary to store the marker positions and timestamps for each label ID
marker_positions = {}

# Time (in seconds) after which a marker will be considered expired
MARKER_EXPIRATION_TIME = 5.0


def callback(data):
    # Get the current time
    current_time = rospy.Time.now()

    # Remove expired markers
    remove_expired_markers(current_time)

    for obj in data.objects:
        label = obj.label
        label_id = obj.label_id
        sublabel = obj.sublabel
        confidence = obj.confidence
        position = obj.position

        if label == "Person" and confidence >= 60.0:
            # Update the marker position and timestamp for the current object
            marker_positions[label_id] = (position, current_time)

    # Publish the markers
    publish_markers()


def remove_expired_markers(current_time):
    # List to store the IDs of the markers that need to be removed
    markers_to_remove = []

    for label_id, (_, marker_time) in marker_positions.items():
        # Calculate the elapsed time since the marker was last updated
        elapsed_time = (current_time - marker_time).to_sec()

        if elapsed_time >= MARKER_EXPIRATION_TIME:
            # Add the expired marker ID to the list
            markers_to_remove.append(label_id)

    # Remove the expired markers from the marker_positions dictionary
    for label_id in markers_to_remove:
        marker_positions.pop(label_id)


def publish_markers():
    for label_id, (position, _) in marker_positions.items():
        # Create a marker for each object
        marker = Marker()
        marker.header.frame_id = "map"  # Set the frame ID of the marker to match your map's frame ID
        marker.id = label_id  # Assign a unique marker ID based on the label ID
        marker.type = Marker.SPHERE  # Set the marker type to represent a sphere
        marker.pose.position.x = position[0]  # Set the position of the marker
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Initialize quaternion to identity orientation
        marker.scale.x = 0.1  # Set the size of the marker
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Set the color of the marker (e.g., red)

        # Publish the marker
        marker_pub.publish(marker)


def listener():
    global marker_pub  # Declare marker_pub as a global variable

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("zed2i/zed_node/obj_det/objects", ObjectsStamped, callback)

    # Create a publisher for visualization markers
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    listener()
