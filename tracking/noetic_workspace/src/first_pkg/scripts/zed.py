import rospy
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion

# Define marker publisher as a global variable
marker_pub = None
# Dictionary to store the marker positions for each label ID
marker_positions = {}
# Keep track of the last update time for each marker
marker_times = {}

def callback(data):
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
            rospy.loginfo("Object: Label=%s, Label ID=%d, Sublabel=%s, Confidence=%.2f, Position=%.2f, %.2f, %.2f, tracking=%d",
                          label, label_id, sublabel, confidence, position[0], position[1], position[2], tracking_state)

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
        marker.scale.x = 0.5  # Set the size of the marker
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # Set the transparency of the marker
        marker.color.r = 1.0  # Set the color of the marker (e.g., red)
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        marker_pub.publish(marker)

def remove_marker(label_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = label_id
    marker.action = Marker.DELETE  # This action removes the marker
    marker_pub.publish(marker)

def listener():
    global marker_pub

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("zed2i/zed_node/obj_det/objects", ObjectsStamped, callback)

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
