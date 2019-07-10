# Planning Functions for Robot Manipulation with Twin Palms

This repo contains a Python library to plan robot manipulation primitives: pushing, pulling, levering, and grasping. This repo is adapting from [Mpalms Repo](https://github.com/mcubelab/mpalms), which contains the full planning + feedback pipeline for the tactile dexterity project.

## Dependencies

### Mandatory
This repo contains depencies to the following python librairies

### Optional

## Installation and Setup
Below we detail installation instructions for OpenRAVE and the creation of a YumiPy workspace. 

> :warning: This instructions assume Ubuntu 14.04. Ubuntu can be installed as the [native OS](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0) or as [VM with VirtualBox](https://linus.nci.nih.gov/bdge/installUbuntu.html).

### Basic Ubuntu Packages

Ubuntu can be installed as the native OS (https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0) or as VM with VirtualBox (https://linus.nci.nih.gov/bdge/installUbuntu.html). 

After setting up Ubuntu the first item to install is ROS. We borrow instructions from the Personal Robotics Lab. Install ROS: 
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
$ sudo apt-get update && sudo apt-get install ros-indigo-desktop-full
```

Then install dependencies: 
```
$ sudo apt-get install git python-pip python-catkin-tools python-wstool python-enum34 libnewmat10-dev collada-dom2.4-dp* texlive-full texmaker vim 
$ sudo pip install -U pip # upgrade outdated pip in Ubuntu packages
```

ROS dep is used to initialize dependencies, so initialize it as well: `sudo rosdep init`. 

### OpenRAVE Installation

We next install OpenSceneGraph (for 3D graphics), FCL (Flexible Collision Library) and then OpenRAVE. 

For OpenSceneGraph: 
```
git clone https://github.com/openscenegraph/OpenSceneGraph.git
cd OpenSceneGraph
git checkout OpenSceneGraph-3.4
mkdir build && cd build
cmake .. -DDESIRED_QT_VERSION=4
make -j4
sudo make install
```

For FCL:
```
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout 0.5.0  # use FCL 0.5.0
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

For OpenRAVE
```
git clone https://github.com/rdiankov/openrave.git
cd openrave
git checkout latest_stable
mkdir build && cd build
cmake .. -DOSG_DIR=/usr/local/lib64/
make -j4
sudo make install
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_" >> ~/.bashrc
echo "export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)" >> ~/.bashrc
```

Test your OpenRAVE installation by running the following example. It should show a WAM arm manipulation objects on a table. 
```
bash
openrave.py --example graspplanning
```

For robots that use IKFast as the inverse kinematics generator, please use the following bug fix: https://github.com/rdiankov/openrave/issues/387


### Workspace Setup and Specific Packages

Next we set up our catkin workspace, again borrowing instructions from the Personal Robotics Lab. Create a catkin workspace (named `my-workspace` below): 
```
$ mkdir my-workspace && cd my-workspace
$ wstool init src
$ catkin init
$ catkin config --extend /opt/ros/indigo
$ catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Next we want to checkout code. We use `wstool` with the `.rosinstall` found in this repo. 
```
cd src
git clone https://github.com/mcubelab/yumipy.git
cd yumipy
cp yumipy.rosinstall ..
wstool merge yumipy.rosinstall
wstool up
```

We use rosdep to install the remaining dependencies: 
```
$ rosdep update
$ rosdep install -y -r --ignore-src --rosdistro=indigo --from-paths src
```

Ordinarily we would next use `catkin build`. However, currently due to the `abb-ros-catkin` package, we use `catkin_make`.

## Example Usage 
Coming Soon. 
