#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

# Goals - 
# 1. Register when object enters within a certain distance.
# 2. Register object deviation from center 

class InterpretVelodyne:
    def __init__(self):
        self.velodyne_subscriber = None
        self.last_update = 0

    def interpret_point_cloud_2(self, point_cloud_data : PointCloud2):
        
        # Time since last update.
        print(point_cloud_data)
        
        # rospy.loginfo(len(multiArray.data))

        

        # if len(multiArray.data) > 0:
        #     id = multiArray.data[0]

        #     # Determine if this is a new hazard marker. 
        #     if self.hazardMarkers.__contains__(id):
        #         if not self.foundMarkers.__contains__(id):

        #             # if it's the start marker then publish the start command.
        #             if id == self.START_MARKER:
        #                 self.hazardPublisher.publish("start")
        #                 subprocess.Popen("roslaunch first_pkg start_explore.launch", shell=True)
        #                 self.foundMarkers.add(id)

        #             elif (self.START_MARKER in self.foundMarkers or self.ignoreStartMarker):
        #                 # Add the hazard marker. 
        #                 self.foundMarkers.add(id)
                        
        #                 # use for testing. 
        #                 self.hazardPublisher.publish("id identified: %.0f" % id)

        #                 # Publish marker identification. 
        #                 self.publishVisualMarker(id)

        #             # if five markers (and the start marker) have been found then publish the return command.
        #             if (not self.ignoreStartMarker and len(self.foundMarkers) >= 5) or (self.ignoreStartMarker and len(self.foundMarkers) >= 6):
        #                 # publish return.
        #                 self.hazardPublisher.publish("return")

    """
    Start subsribers.
    """
    def execute(self):
        self.velodyne_subscriber = rospy.Subscriber('/velodyne_points', PointCloud2, self.interpret_point_cloud_2)
        rospy.spin()



# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('velodyne_interpret', anonymous=True)
        interpret_velodyne = InterpretVelodyne()
        interpret_velodyne.execute()

    except rospy.ROSInterruptException:
        pass