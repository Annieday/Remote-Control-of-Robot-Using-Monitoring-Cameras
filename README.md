# Remote-Control-of-Robot-Using-Monitoring-Cameras
This is the course work for MM 802 - Multimedia Communication (Winter 2018), Assignment 2 @ University of Alberta, Canada.

# Components
There are three modules in this project.
- Client, Server, and Producer infrastructure module.
- Robot controller module.
- Multimedia module.

# Dependencies
- OpenCV 3
- ROS Kinetic
- Socket API
- Python 3

# Code Execution
Please clone this repository before you execute and use it.

#### Using Client, Server, and Producer infrastructure module
Open a terminal window and run the following command to establish a server.
```bash
python3 socket_server.py
```

Then open another terminal window and run the following command to create a producer and start streaming.
```bash
python3 socket_producer.py
```

Finally, open the third terminal window and run the following command to open the client-side application.
```bash
python3 socket_client.py
```

#### Using robot controller module
Turn on the onboard computer on the robot, then run the following command in it.
```bashe
roscore
```

Then, using the following command to enable the depth camera.
```bash
roslaunch realsense2_camera rs_rgbd.launch
```

Then, run the following command to start the movement controller driver.
```bash
cd Downloads/RoboRTS
./build/modules/driver/serial/serial_node
```

Then, using the following command to open the configuration of the camera.
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Once the configuration pop-up, reduce the exposure down to the lowest. And, now the robot controller module is ready to use.

#### Using the auto-detection algorithm
On either the server or client machine, open a terminal and run the following command to start auto-detection algorithm.
```bash
python3 compressed_sub.py
```
