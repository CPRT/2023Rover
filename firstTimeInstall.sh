#!/bin/bash

locale  # check for UTF-8

set -e
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade -y

sudo apt install -y \
ros-humble-desktop \
ros-humble-ros-base \
ros-dev-tools \
python3-dev \
python3-pip \
clang-format \
clang-tidy

source /opt/ros/humble/setup.bash

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  echo "ROS 2 sourced in bashrc"
fi

# Add lines in .bashrc to source the cprt_bash_aliases file if it exists
if ! grep -qF "cprt_bash_aliases" ~/.bashrc; then
   {
      echo "if [ -f $SCRIPT_DIR/cprt_bash_aliases ]; then"
      echo "   . $SCRIPT_DIR/cprt_bash_aliases"
      echo "fi"
   } >> ~/.bashrc
fi

pip3 install black

echo "ROS2 humble install complete!"
