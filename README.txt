Team member:
Shaoxun Xu
Aiwei Yin 10730328
Jiaying Lyu 10735282

#Description of the files inside the archive:

|-- PolimiRoboticsCourceProject
    |-- README.txt
    |-- data
    |   |-- project.bag //project data
    |-- distance_message
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- cfg
    |   |   |-- dynamic_distance.cfg //dynamic reconfigure to change the two thresholds
    |   |-- msg
    |   |   |-- Status.msg //difine of the costume message
    |   |-- src
    |       |-- pub.cpp //receive odom data and use distance_service to compute distance between car and obs, publish message to show car status
    |       |-- sub.cpp //for test
    |-- distance_service //service to compute distance between car and obs 
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- src
    |   |   |-- client.cpp //for test
    |   |   |-- compute_distance.cpp
    |   |-- srv
    |       |-- ComputeDistance.srv
    |-- launch //lanuch all the nodes
    |   |-- project.launch 
    |-- lla2enu //for debug
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- build
    |   |-- scripts
    |   |   |-- lla2enu_py.py
    |   |-- src
    |       |-- sub.cpp 
    |-- lla2tf //for debug
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
            |-- lla2tfodom.cpp //receive data from project.bag and publish TF and Odom
 
#Name of the parameter:


#Structure of the tf tree:
/world
    └───>/obs
    └───>/car
     

#Structure of the custom message:

distance_message::Status includes
-float64 distance: the distance between car and obstacle.
-string status: "Safe" ,"Unsafe" ,and "Crash" indicate different status of the car.

#usage:

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
catkin_make -j4

```
if there are some error during compile, please rerun ```catkin_make -j4```

Running
```
source devel/setup.bash
roslaunch src/PolimiRoboticsCourceProject/launch/project.launch 
```
You will see the output like 
```[ INFO] [1588785320.187441988]: Distance: 6.272685, status: Safe.``` 
when there are data from both car and obstacle.


#Info you think are important/interesting:

---catkin_make may not always compile the code in the right way, we should add ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} in some CmakeLists.txt to make sure it compile some code first as the dependency of other code.







