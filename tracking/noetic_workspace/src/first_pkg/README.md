# Overview of Python Scripts
`velodyne_interpret.py` filteres noise in PointCloud2 data

1. Crops point cloud using a bounding box
2. Applies radius outlier removal for points that are further away than neighbours
3. Publishes /filtered_cloud

which is then read by an external module, 'Multiple-object-tracking-lidar' (https://github.com/praveen-palanisamy/multiple-object-tracking-lidar)

`zed.py` Reads the ZED Object Detection Data and moves to the owner (the closest tracked person on program launch). 

1. Reads ZED ObjectsStamped message data, supplied from the ZED 2 Object Detection Model. 
2. Filters the objects based on whether they are labelled a 'Person' and whether they have a high confidence level. 
3. Creates markers using the x,y position of the filtered objects.
2. Removes Markers that have not been updated for 5 seconds. 
3. Stores the ID of a tracked object/person - tracked person is initialised by selecting the object closest in distance.
4. Utilises velocity controls to move the robot towards the target at a safe pace.

`tracker.py` is a work in progress file that attempts to use move based, rather than utilising velocity controls. If complete, the solution would add obstacle avoidance whilst following the target, an ideal outcome.

`take_zed_images.py` is a work in progress file that attempts to capture images using the ZED 2i camera and cropping them based on the bounding box income of the ZED Object Detection Model. If complete, this would allow the user to gather images for a image training dataset - possibly leading to teaching the robot to classify particular individuals by apperance.

# Setting up Gazebo - 

We can dummy up the Gazebo implementation for the VLP-16 by cloning https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/ into TheConstruct.

Clone the repository into the *aiil_workspace/noetic_workspace/src* then from *aiil_workspace/noetic_workspace* perform a catkin_make

Once the package has been created you should be able to run the simulation using **roslaunch velodyne_description example.launch**

The vlp-16 publishes data to the /velodyne_points topic. As per the documentation this should be accurate Point2D Cloud data that the Panther velodyne publishes too.