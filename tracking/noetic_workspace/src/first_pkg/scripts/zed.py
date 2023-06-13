import rospy
from zed_interfaces.msg import ObjectsStamped

def callback(data):
    # This is where you process the ObjectsStamped data.
    # For example, you might print the 3D position of the first detected person:
    for obj in data.objects:  
        if obj.label == "person":  # if this object is a person
            rospy.loginfo(f"Detected a person at position: {obj.position}")
            break  # stop after finding the first person

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("zed2/zed_node/obj_det/objects", ObjectsStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
