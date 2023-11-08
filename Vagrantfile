# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
    config.vm.box = "ubuntu/jammy64"
  
    config.vm.synced_folder "./ros_ws", "/home/vagrant/ros_ws"
    config.vm.synced_folder "./.code-server", "/home/vagrant/.code_server"
  
    config.vm.provider "virtualbox" do |vb|
      # Display the VirtualBox GUI when booting the machine
      vb.gui = true
      vb.cpus = "4"
    
      # Customize the amount of memory on the VM:
      vb.memory = "16384"
  
      vb.customize ["modifyvm", :id, "--graphicscontroller", "vmsvga"]
      vb.customize ["modifyvm", :id, "--vram", "128"]
    end

    config.vm.provision "shell", inline: <<-SHELL
        echo "==========================================\n"
        echo "1. Initial OS + Ubuntu Desktop Setup\n"
        echo "==========================================\n\n"
        
        apt-get update
        apt-get install -y ubuntu-desktop
        apt-get install -y virtualbox-guest-dkms virtualbox-guest-utils virtualbox-guest-x11

        echo "\n\n"
        echo "==========================================\n"
        echo "2. Installing ROS Humble\n"
        echo "==========================================\n\n"

        echo "\n\n"
        echo "-------------\n"
        echo "2.1 - Locale Setup\n"
        echo "-------------\n\n"

        apt update
        apt install locales
        locale-gen en_US en_US.UTF-8
        update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8

        echo "\n\n"
        echo "-------------\n"
        echo "2.2 - Add Additional Repositories\n"
        echo "-------------\n\n"

        apt install -y software-properties-common
        add-apt-repository universe
        apt update
        apt install -y curl
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        # Finally, refresh the package lists:
        apt update
        apt upgrade # Recommended as ROS2 is tied tightly to ubuntu releases apparently

        echo "\n\n"
        echo "-------------\n"
        echo "2.3 - Install ROS2\n"
        echo "-------------\n\n"

        apt install -y ros-humble-desktop # Includes rviz + demos

        echo "\n\n"
        echo "-------------\n"
        echo "2.4 - Source ROS\n"
        echo "-------------\n\n"

        echo 'source /opt/ros/humble/setup.bash' >> /home/vagrant/.bashrc

        echo "\n\n"
        echo "==========================================\n"
        echo "3. Installing Navigation2\n"
        echo "==========================================\n\n"

        echo "\n\n"
        echo "-------------\n"
        echo "3.1 - Install Navigation 2\n"
        echo "-------------\n\n"

        apt install -y ros-humble-navigation2
        apt install -y ros-humble-nav2-bringup

        echo "\n\n"
        echo "-------------\n"
        echo "3.2 - Addenum - Install Cyclone DDS\n"
        echo "-------------\n\n"
        apt install -y ros-humble-rmw-cyclonedds-cpp
        echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /home/vagrant/.bashrc

        apt install -y ros-humble-slam-toolbox
        apt install -y ros-humble-tf-transformations

        echo "\n\n"echo "\n\n"
        echo "==========================================\n"
        echo "4. Installing other tools\n"
        echo "==========================================\n\n"

        apt install -y python3-colcon-common-extensions
        curl -Lk 'https://code.visualstudio.com/sha/download?build=stable&os=cli-alpine-x64' --output vscode_cli.tar.gz
        tar -xf vscode_cli.tar.gz
        mv code /usr/local/bin/

        echo "FILE=/home/vagrant/ros_ws/.env" >> /home/vagrant/.bashrc
        echo "if [ -f "$FILE" ]; then" >> /home/vagrant/.bashrc
        echo "    source /home/vagrant/ros_ws/.env" >> /home/vagrant/.bashrc
        echo "fi" >> /home/vagrant/.bashrc

        # Install the TurtleBot4 packages
        apt install -y ros-humble-turtlebot4-description ros-humble-turtlebot4-msgs \
          ros-humble-turtlebot4-navigation ros-humble-turtlebot4-node \
          ros-humble-turtlebot4-simulator


    SHELL

  end