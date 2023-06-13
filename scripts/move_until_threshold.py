# subscribe to /zed2i/zed_node/obj_det/objects 
# place the objected as Markers 
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from first_pkg.msg import ObjectsStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA

CONFIDENCE_THRESHOLD = 0.65
ZED_PERSON_LABEL = 'Person'

class MarkerPublisher:
    def __init__(self):
        self.marker_pub = rospy.Publisher('/object_marker', Marker, queue_size=1)
        self.object_sub = rospy.Subscriber('/zed2i/zed_node/obj_det/objects', ObjectsStamped, self.object_callback)
        self.objects = None

    def object_callback(self, msg):
        if len(msg.objects) > 0:
            for i in range(len(msg.objects)):
                if msg.objects[i].label == ZED_PERSON_LABEL:
                    if msg.objects[i].confidence > CONFIDENCE_THRESHOLD:
                        self.objects = msg.objects[i]
                        rospy.loginfo('Object detected: %s Confidence: %s Action: %s', msg.objects[i].label, msg.objects[i].confidence, msg.objects[i].action_state)
                        # Print idle if action_status = 0 and walking if action_status = 1
                        if msg.objects[i].action_state == 0:
                            rospy.loginfo('Idle')
                        elif msg.objects[i].action_state == 1:
                            rospy.loginfo('Walking')
                        
                        self.rotate_to_object(msg.objects[i])

    def rotate_to_object(self, object):
        # Rotate the robot to face the object
        rospy.loginfo('Rotating to object')
        
        pose = Pose()
        pose.position.x = object.position[0]
        pose.position.y = object.position[1]
        pose.position.z = object.position[2]

        # Calculate the angle to rotate
        angle = math.atan2(pose.position.y, pose.position.x)
        
        rospy.loginfo('Angle to rotate: %s', angle)

        # TO DO!!!!!
    

    def publish_marker(self, object):
        if object is None:
            return
            
        marker = Marker()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

        # Create a Pose instance
        pose = Pose()
        pose.position.x = object.position[0]
        pose.position.y = object.position[1]
        pose.position.z = object.position[2]
        marker.pose = pose

        self.marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('marker_publisher')
    marker_publisher = MarkerPublisher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
