# Manipulation Kinova Gen 3
In this repository, you will find the necesary files to execute a control arquitecture for the P3Bot robot. This robot is a mobile manipulator that has 2 Kinova Gen 3 arms, a LIDAR and a RGBD camera. 
The control arquitecture is based on the Robocomp framework. The control arquitecture is based on the following components:

Some of the components was implemented to control a single arm, so you have to run them 2 times simultaneously, one for each arm. The right arm is called "pedro" and the left arm is called "pablo".

### Kinova_controller_cpp
This is the agent that control the robot. It is a C++ program that uses the Kinova API to control the robot. Added to this the agent uses the DSR to communicate with the other components. 

**You have to compile this program before running it.** You should read the README file in the Kinova_controller_cpp folder to know how to compile it. 

You have to run this program 2 times simultaneously, one for each arm. You can run it with the following command:
```
cd agents/kinova_controller_cpp
bin/Kinova_controller_cpp etc/config_brazo_[pablo/pedro]
```

### Camera_kinova
This is the component that reads the images from the camera. It is a python program that uses the OpenCV library to read the images.

You have to run this program 2 times simultaneously, one for each arm. You can run it with the following command:
```
cd components/camera_kinova
src/camera_kinova.py etc/config_brazo_[pablo/pedro]
```

### Toolbox_controller  
This is the component that control the arms. It is a python program created by Peter Corke. You have to install the Robotics Toolbox for Python to run this program. You can install it with the following command:
```
pip install roboticstoolbox-python
```
In this case, you have to run this program only one time. You can run it with the following command:
```
cd components/toolbox_controller
src/toolbox_controller.py etc/config
```

### Contactile_controller
This is the component that control the contactile sensor. It is a C++ program reads the contactile sensor values.

You have to run this program one time, because just one arm have sensor. You can run it with the following command:
```
cd ../../../robocomp-robolab/components/hardware/tactile/contactile
bin/contactile etc/config
```

### Pybullet_controller
This is the component that simulates the robot. It is a python program that uses the Pybullet library to simulate the robot.

You have to run this program one time, because the simulation is the same for both arms. You can run it with the following command:
```
cd components/pybullet_controller
src/pybullet_controller.py etc/config
```

## Install Robotics Toolbox for Python
To install the Robotics Toolbox for Python fork that include the P3Bot model, you can use the following command:
```
git clone https://github.com/jcalderon12/robotics-toolbox-python.git
cd robotics-toolbox-python
pip install -e .
```

