# ROS challenge

Consideration before compiling and running the application:

-Developed in Ubuntu 18.04, using ROS Melodic. 

-The application was developed within a Catkin Workspace

-User will need to have ROS and Catkin installed and configured: http://wiki.ros.org/ROS/Tutorials

-Bash file uses xterm. Can be downloaded with `sudo apt-get install xterm`. Otherwise, nodes can be executed independently in different terminals as explained later.

# Steps:

1) Position yourself in catkin workspace:

`$ cd ~/catkin_ws`

3) Compilation:

`$ catkin_make`

2) Source the environment setup file so that rosrun recognizes the packages

`$ source ./devel/setup.bash`

# Running bash file. This will open all needed nodes (roscore, turtlesim_node, robot node,  give_dirs node, services_node and rosbridge server):

`$ ./launch_challenge_bash.sh`

# Or opening nodes independently. You will need a terminal for each node:

Start Roscore: `$ roscore`

Start Turtlesim: `$ rosrun turtlesim turtlesim_node`

Start Robot node: `$ rosrun challenge robot`

Start give directions node: `$ rosrun challenge give_dirs`

Start services node: `$ rosrun challenge services_node.py`

# HTML position and velocity viewer:

An .html file has been written using the roslibjs library to display the turtle position and speed. To visualize it, one needs to have installed the rosbridge_server. If it is not installed, one can do so with `$ sudo apt-get install ros-<rosdistro>-rosbridge-server` where <rosdistro> needs to be replaced with the installed ros distribution (melodic for this repository).

Run the ros server with (if it was not opened with the bash file):

`$ roslaunch rosbridge_server rosbridge_websocket.launch port:=9091`

Then, if roscore and turtlesim_node are open, challenge _web.html can be opened to view the position and speed. This will be displayed in a table and in the web console.


# Notes:

- If you get a message "roscd: No such package", one should run:

`$ cd ~/catkin_ws`

`$ source ./devel/setup.bash`

- The services node accepts inputs 'kill', 'spawn', 'clear' and 'reset'. Further spawned turtles are not movable.

- The default port for the ros bridge server is 9090. However, i changed it to 9091 as 9090 was being used. To change it to another one, you need to change the port number in the rosbridge command and in challenge_web.html.
