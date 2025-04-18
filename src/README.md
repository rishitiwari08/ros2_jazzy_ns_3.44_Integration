# 1.0 ROS 2 TurtleSim Setup

This README provides instructions on how to create and use a ROS 2 package for controlling the TurtleSim and publishing the turtle's pose over a UDP port. The package contains scripts to move the turtle in a circular motion and send its pose data to an NS-3 simulation.
You can find the code under 
```bash 
cd src/ros2_code
```

## 1.1 Create a ROS 2 Package

First, ensure your ROS 2 installation is sourced. Then, navigate to the `src` folder in your workspace where you want to create the new package:

```bash
cd ~/ros2_ws/src
```
Run the following command to create a new ROS 2 package. For this tutorial, we'll use Python as the build type and include some basic setup:

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package
```

This command creates a new package with the following structure:
```arduino
my_package/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_package/
├── my_package/
│   └── __init__.py
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── my_package/my_node.py
```

## 1.2 Build the Package

After creating your package, navigate back to the root of your workspace to build the package:
```bash
cd ~/ros2_ws
colcon build
```
This will compile the ROS 2 package. If you want to build only your specific package, run:
```bash
colcon build --packages-select my_package
```

## 1.3 Source the Setup File
Once the package is built, you need to source the setup file to use your new package:

```bash
source install/local_setup.bash

```
## 1.4  Copy my Code into the Package
Now that the package is set up, you can copy the ros2_code folder (which contains the code for moving the turtle and publishing the pose over UDP) into the src directory of your package:

 1. Navigate to your ROS 2 workspace's src directory:
    ```bash 
    cd ~/ros2_ws/src/my_package
    ```
 2. Copy the src/ros2_code folder containing my custom Python scripts into the my_package folder. This will copy the necessary code turtle_circle_mover.py and turtle_pose_publisher.py  into your ROS 2 package.

## 1.5 Change the setup.py file inside your package

Change the last part of the setup.py file to include new python file:

```python
entry_points={
        'console_scripts': [
            'my_node = ns3_bridge.my_node:main',
            'turtle_pose_publisher = ns3_bridge.turtle_pose_publisher:main',
            'turtle_circle_mover = ns3_bridge.turtle_circle_mover:main',
        ],
    },
```
## 1.6 Build the package again and run your code 

Build the package again:
```bash
cd ~/ros2_ws
colcon build --packages-select my_package
```

First open a new terminal and run Turtle Sim(Preinstalled with ROS2_jazzy)

```bash
ros2 run turtlesim turtlesim_node
```

Again, open a new terminal and run our new program,to run your turtle simulation that moves the turtle in a circle, use:

```bash
ros2 run my_package turtle_circle_mover
```

On 3rd new terminal run, To run the code that publishes the turtle pose over UDP, use::

```bash
ros2 run my_package turtle_pose_publisher
```



# 2.0 NS-3 Setup

This README provides instructions on how to create and use a NS3 program to read the UDP signal sent by ROS2 and the working behind it.
You can find the code under 
```bash 
cd src/ns3_code
```

## 2.1 Create NS-3 Script 

- First, navigate to the scratch directory within your NS-3 installation. This is where you typically create and run custom NS-3 scripts.

```bash
cd ~/ns-allinone-3.44/ns-3.44/scratch
```
- Create a new C++ file named `turtle_publisher.cc` (or any name you prefer) using a text editor.

- Copy the code provided under `ns3_code` folder.

- Save the file and Exit

## 2.2 Build the NS-3 Script 

- Navigate to the root directory of your NS-3 installation.
``` bash 
cd ~/ns-allinone-3.44/ns-3.44
```
- Build the NS-3 script using the `ns3 build` command.

```bash
./ns3 build
```


## 2.3 Run the Script

- To enable logging for the `UdpServer` component and see detailed output, set the `NS_LOG` environment variable.
```bash 
export NS_LOG=UdpServer=info
```

- Run the NS-3 script using the `ns3 run` command.
```bash 
./ns3 run scratch/turtle_publisher
```
This will execute the simulation, start the UDP server on node 1, and print the number of received packets to the console after the simulation finishes.



# 3.0 Verifying the Communication

To verify that the communication between ROS 2 and NS-3 is working correctly:
- Run the ROS 2 nodes (`turtle_circle_mover` and `turtle_pose_publisher`) in separate terminals.
- Run the NS-3 script (`./ns3 run scratch/turtle_publisher`).
- Check the NS-3 console output. It should show the number of UDP packets received.
- Monitor the NS-3 logs (if enabled) to see the details of the received packets.

By following these steps, you can set up and run your NS-3 simulation to receive UDP data from your ROS 2 application, providing a foundation for your thesis work.