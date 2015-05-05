# Teach-A-Robot

## Installation

Prerequisites: Ubuntu 11.10, 12.04 (Recommended) or 12.10

1) Install ROS (Groovy) and dependencies:

<pre>
sudo apt-get install ros-groovy-ros ros-groovy-rviz ros-groovy-geometric-shapes ros-groovy-arm-navigation-experimental ros-groovy-arm-navigation ros-groovy-interactive-markers ros-groovy-pr2-arm-navigation ros-groovy-navigation ros-groovy-rosbridge-server ros-groovy-tf2-web-republisher ros-groovy-interactive-marker-proxy
</pre> 

2) Get the rosinstall file:

<pre>
(Coming soon)
</pre>

3) Install the rosinstall file:

<pre>
rosinstall --verbose <installation_folder> tar_unstable.rosinstall
</pre>
  
4) Source the setup.bash that is created in the installation_folder directory. Also, source it in your bashrc.

5) Get the rosbuild files for sbpl_geometry_utils:

<pre>
(Coming soon)
</pre>

6) Follow the instructions in README.txt to convert sbpl_geometry_utils back from catkin to rosbuild.

7) rosmake dviz_core

## Installing and Running the Webserver

1) Install the apache2 webserver:

<pre>
sudo apt-get install apache2
</pre>

2) In /etc/apache2/sites-enabled/000-default, change 

<pre>
DocumentRoot /var/www
</pre>

to 

<pre>
DocumentRoot (path-to-dviz_web)
</pre> 

(e.g. “DocumentRoot /home/eratner/ros/teach-a-robot/dviz_web”) and change

<pre>
Directory /var/www
</pre>

to 

<pre>
Directory (path-to-dviz_web)
</pre>

(e.g. “Directory /home/eratner/ros/teach-a-robot/dviz_web”).

3) Restart apache2: 

<pre>
sudo /etc/init.d/apache2 restart
</pre>

4) Launch dviz_web:

<pre>
roslaunch dviz_web dviz_web.launch
</pre>
