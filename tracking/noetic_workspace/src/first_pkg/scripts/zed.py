import rospy
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion

# Define marker publisher as a global variable
marker_pub = None

def callback(data):
    for obj in data.objects:
        label = obj.label
        label_id = obj.label_id
        sublabel = obj.sublabel
        confidence = obj.confidence
        position = obj.position

        rospy.loginfo("Object: Label=%s, Label ID=%d, Sublabel=%s, Confidence=%.2f, Position=%.2f, %.2f, %.2f",
                      label, label_id, sublabel, confidence, position[0], position[1], position[2])

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
        marker.color.a = 1.0  # Set the transparency of the marker
        marker.color.r = 1.0  # Set the color of the marker (e.g., red)
        marker.color.g = 0.0
        marker.color.b = 0.0

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
