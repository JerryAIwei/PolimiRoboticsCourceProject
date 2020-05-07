Team member:
Shaoxun Xu 10730116
Aiwei Yin 10730328
Jiaying Lyu 10735282

# Description of the files inside the archive:

|-- PolimiRoboticsCourceProject
    |-- README.txt
    |-- data
    |   |-- project.bag // project data
    |-- distance_message
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- cfg
    |   |   |-- dynamic_distance.cfg // dynamic reconfigure to change the two thresholds
    |   |-- msg
    |   |   |-- Status.msg // difine of the costume message
    |   |-- src
    |       |-- pub.cpp // receive odom data and use distance_service to compute distance between car and obs, publish message to show car status
    |       |-- sub.cpp // receive the message published by pub, print it out
    |-- distance_service // service to compute distance between car and obs 
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- src
    |   |   |-- client.cpp // for test
    |   |   |-- compute_distance.cpp
    |   |-- srv
    |       |-- ComputeDistance.srv
    |-- launch // lanuch all the nodes
    |   |-- project.launch  // launch this file, it will start all nodes and play the ros bag of the project as well
    |-- lla2enu // for debug
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- build
    |   |-- scripts
    |   |   |-- lla2enu_py.py
    |   |-- src
    |       |-- sub.cpp 
    |-- lla2tf // for debug
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- build
    |   |-- src
    |       |-- lla2tf.cpp
    |-- lla2tfodom 
        |-- CMakeLists.txt
        |-- package.xml
        |-- build
        |-- src
            |-- lla2tfodom.cpp // receive data from project.bag and publish TF and Odom
 
# Name of the parameter:

In launch file, there are latitude_init, longitude_init and h0, which represents the zero point of the onversion from LLA to ENU.
"car" and "msgPathCar" represents the name of the car objectt and the topic name of the car GPS position provided by the ros bag file.
Similarly, "obs" and "msgPathObs" represents the name of the obstacle objectt and the topic name of the obstacle GPS position provided by the ros bag file.
 
# Structure of the tf tree:
/world
    └───>/obs
    └───>/car
     

# Structure of the custom message:

distance_message::Status includes
-float64 distance: the distance between car and obstacle.
-string status: "Safe" ,"Unsafe" ,and "Crash" indicate different status of the car.

# usage:

We test our project on ubuntu 18.04 with ROS Melodic

Prerequisites
```
mkdir -p ~/your_workspace/src
cd ~/your_workspace/src
```
put PolimiRoboticsCourceProject Folder here

Compile
```
cd ~/your_workspace
catkin_make

```
if there are some error during compile, please rerun ```catkin_make -j1```, this is because possible problems of the multithreading compile.

Running
```
source devel/setup.bash
roslaunch src/PolimiRoboticsCourceProject/launch/project.launch 
```
This will start all the necessary nodes and play the ros bag of the data as well. The bag is played at a rate of 3 because there are no obstacle GPS position data at the beginning, it will take a while and then you will get the output of the distance and status on the same terminal tab.

You will see the output like 
```[ INFO] [1588785320.187441988]: Distance: 6.272685, status: Safe.``` 
when there are data from both car and obstacle.

You can run ``` rosrun rqt_reconfigure rqt_reconfigure ``` to dynamicly reconfigure the parameters of the safe distance and crash distance between the car and the obstacle. The crash distance should be less than the safe distance. The default safe distance is 5m and the default crash distance is 1m as required by the project.

# Info you think are important/interesting:

---catkin_make may not always compile the code in the right way, we should add ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} in some CmakeLists.txt to make sure it compile some code first as the dependency of other code.







