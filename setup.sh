sudo apt install -y python3
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
sudo apt-get install -y python-dev python-pip python3-dev python3-pip python3-rospkg
sudo apt-get install -y ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
sudo pip install pymavlink
sudo pip3 install pymavlink
sudo pip install opencv-python
sudo pip3 install opencv-python
sudo pip install numpy
sudo pip3 install numpy
sudo pip install pyserial
sudo pip3 install pyserial
sudo echo "sudo pip3 uninstall pyserial" >> ~/.bashrc
sudo echo "sudo pip3 install pyserial" >> ~/.bashrc
