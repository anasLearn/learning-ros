sudo apt-get update && sudo apt-get upgrade -y

sudo apt-get install build-essential gcc make perl dkms -y

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-get install curl -y

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


sudo apt-get update

sudo apt-get install ros-noetic-desktop-full -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc


sudo apt install python3-catkin-tools 
# sudo apt-get install python3-osrf-pycommon


# sudo apt-get install python3-rosdep2
# sudo apt-get install python3-rosdep


sudo apt-get install python3-rosdep

sudo rosdep init

rosdep update


sudo apt install ros-noetic-moveit

