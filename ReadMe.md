# Tim's Panther Development Instructions 

1. Stop all containers on dot 3 before shut down
1. Ensure the arm is off before shut down
1. Never run the bot with the power cable connected
1. Wait for full shutdown and restart
1. Software E-Lock can be disabled through **10.15.20.2:8000**

# Questions - 

1. How do you safely shutdown the panther?

> According to husarion documentation 
> https://husarion.com/manuals/panther/

> We press the power button and wait for the power button to stop blinking. Once it stops blinking you can disconnect the battery.

1. How do we know if the arm is on/off?

> According to the UR5e manual, the Robot arm itself has a state indicator either red, yellow, or green indicating that it's on. 

1. How do we shutdown the arm?

# Startup procedure

1. Remove charging cable
1. Turn battery switch to on
1. Turn on power
1. Disable hardware lock (big red knob on the back of the bot)
1. Disable software lock, go to **10.15.20.2:8000** and press the button

# Connecting to the bot

1. Connect to the Panther network
1. ssh into ```ssh husarion@10.15.20.3``` or .2 if you want to connect to the pi.


# Shutdown procedure

1. Connect to the dot 3 ```ssh husarion@10.15.20.3```
1. Check which containers are active ```docker ps```
1. Shutdown all docker containers ```docker stop $(docker ps -a -q)```
1. Ensure all docker containers have stopped ```docker ps```
1. Check the arm is off ```rostopic list``` and confirm UR5e is not running
1. Press the power button to initiate the shutdown sequence
1. When the power button stops blinking
1. Move battery switch to off position
1. Connect charging cable

> Use ```docker ps -a``` to see all containers.


Default LED Behaviour : https://husarion.com/manuals/panther/#animations


# Activating the Velodyne

1. Connect to dot 3 ```ssh husarion@10.15.20.3```
1. cd into the velodyne panther folder ```cd velodyne-docker/panther_velodyne/```
1. Start all docker containers ```docker compose up``'
1. Check if the velodyne container is running using ```docker ps```
1. ~Use docker start to start the velodyne ```docker start velodyne```~

**Run rviz***

1. Open a new terminal
1. set ros master ```set_ros_master 10.15.20.2```
1. rosrun rviz ```rosrun rviz rviz```


# Docker Commands

```sh
# Stop container.
docker stop CONTAINER

# Turn off auto restart.
docker update --restart=no CONTAINER


```