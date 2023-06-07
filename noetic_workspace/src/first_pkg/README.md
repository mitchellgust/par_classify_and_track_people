# Velodyne

# Setting up Gazebo - 

We can dummy up the Gazebo implementation for the VLP-16 by cloning https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/ into TheConstruct.

Clone the repository into the *aiil_workspace/noetic_workspace/src* then from *aiil_workspace/noetic_workspace* perform a catkin_make

Once the package has been created you should be able to run the simulation using **roslaunch velodyne_description example.launch**

The vlp-16 publishes data to the /velodyne_points topic. As per the documentation this should be accurate Point2D Cloud data that the Panther velodyne publishes too.