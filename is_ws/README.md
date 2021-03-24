This project implements ROS2 Webots using trilateration to determine cell location of the E-PUCK robot.
The project has multiple sections consisting of the build section for lab3 task 2, the node section for lab3 task2, the node section for the mulitplce cylinders, and the package/build seciton for multiple cylinders. Both world builds share very very similar nodes and launch files, however there are some small differencees due to the need of generating a new world. The nodes are split up into a master node file that is responsible for publishing speed commands and running the main logic; the slave node file is responsible for communicating with Webots and getting the sensor values that are needed for the master file. The build files consists of build, launch, package.xml, and setup.py as the primary world generaters. Package.xml is responsible for declaring what packages will be used for the ndoe


INSTALLATION AND BUILD INSTRUCTIONS AND RUN INSTRUCTIONS
The necessary steps for installation, build, and run are:

1)Make sure you have a debian Ubuntu 20.04 OS installed


2A)Use https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html to install the debian package of ROS2
    -This will allow for the performing of ros2 capabilities.
    
2B)Alternatively, you can use https://docs.ros.org/en/foxy/Installation/Linux-Install-Binary.html as an installation guide for ROS2

3)Use https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html as a guide to build your workspace. This is where you will run your simulator/program.

4)Once you have your workspace built, you will fill your workspace with relevant WEBOTS information. 

USE THE FOLLOWING TERMINAL COMMANDS:
    source /opt/ros/$ROS_DISTRO/local_setup.bash

    # Retrieve the sources
    cd /path/to/ros2_ws
    git clone --recurse-submodules -b $ROS_DISTRO https://github.com/cyberbotics/webots_ros2.git src/webots_ros2

    # Check dependencies
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

    #Clone the repository for IS_WS
    git clone https://github.com/zhinnen/ROS2-Webots.git
    
    # Building packages
    colcon build

    # Source this workspace (careful when also sourcing others)
    source install/local_setup.bash
    
5)Runing the code:
    
    #Process for running aftering building
    #Go to your WS, CD to it
    cd <ws>
    
    #run foxy and install source
    ~/ros2_foxy/ros2-linux/setup.bash
    source install/setup.bash
    
    #build and run
    colcon build --packages-select is_ws
    ros2 launch is_ws navigation_cells_launch.py 
