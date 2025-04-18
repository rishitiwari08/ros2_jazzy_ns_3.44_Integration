#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# Set locale
echo "Setting up locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "Locale set to: $(locale | grep LANG)"

# Enable required repositories
echo "Enabling required repositories..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
echo "Adding ROS 2 GPG key..."
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \ http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools
echo "Installing development tools..."
sudo apt update && sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools

# Create ROS 2 workspace and clone repositories
echo "Creating ROS 2 workspace..."
mkdir -p ~/ros2_jazzy/src
cd ~/ros2_jazzy
vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src

# Install dependencies using rosdep
echo "Installing dependencies..."
sudo apt upgrade -y
sudo rosdep init || true  # Ignore error if rosdep is already initialized
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build ROS 2
echo "Building ROS 2..."
cd ~/ros2_jazzy/
colcon build --symlink-install

# Setup environment
echo "Setting up ROS 2 environment..."
echo 'source ~/ros2_jazzy/install/local_setup.bash' >> ~/.bashrc
source ~/.bashrc

echo "ROS 2 Jazzy installation completed successfully!"
