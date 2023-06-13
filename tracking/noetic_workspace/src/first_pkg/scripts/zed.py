import rospy
from zed_interfaces.msg import ObjectsStamped

def callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("zed2/zed_node/obj_det/objects", ObjectsStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
