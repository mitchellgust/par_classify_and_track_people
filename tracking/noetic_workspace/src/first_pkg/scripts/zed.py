import rospy
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA

# Define marker publisher as a global variable
marker_pub = None
# Dictionary to store the marker positions and tracking states for each label ID
marker_data = {}


def callback(data):
    for obj in data.objects:
        label = obj.label
        label_id = obj.label_id
        sublabel = obj.sublabel
        confidence = obj.confidence
        position = obj.position
        tracking_state = obj.tracking_state

        if label == "Person" and confidence >= 60.0:
            if tracking_state == 0:
                # Remove the marker if the object's tracking state is 0
                remove_marker(label_id)
            else:
                # Update the marker position and tracking state for the current object
                marker_data[label_id] = (position, tracking_state)

    # Publish the markers
    publish_markers()


def remove_marker(label_id):
    # Remove the marker from the marker_data dictionary
    marker_data.pop(label_id)


def publish_markers():
    for label_id, (position, _) in marker_data.items():
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
