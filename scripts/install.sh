#!/bin/sh
sudo apt-get install git cmake build-essential libboost-system-dev libboost-program-options-dev libboost-thread-dev libboost-test-dev pkg-config libeigen3-dev libboost-filesystem-dev

# cmake makros
git clone https://github.com/rock-core/base-cmake.git   
mkdir base-cmake/build && cd base-cmake/build
cmake .. && sudo make -j8 install && cd ../..

# Logging 
git clone https://github.com/rock-core/base-logging.git
mkdir base-logging/build && cd base-logging/build
cmake .. && sudo make -j8 install && cd ../..

# Base Types
git clone https://github.com/rock-core/base-types.git
mkdir base-types/build && cd base-types/build
cmake .. -DUSE_SISL=OFF -DBINDINGS_RUBY=OFF -DROCK_VIZ_ENABLED=OFF
sudo make -j8 install && cd ../..

# URDF
sudo apt-get install liburdfdom-headers-dev liburdfdom-dev 

# KDL 
git clone https://github.com/orocos/orocos_kinematics_dynamics.git -b v1.5.1
mkdir orocos_kinematics_dynamics/orocos_kdl/build
cd orocos_kinematics_dynamics/orocos_kdl/build
cmake .. 
sudo make -j8 install &&  cd ../../..

# KDL parser
sudo apt-get install libtinyxml2-dev
git clone https://github.com/ros/kdl_parser.git -b 1.14.1
cd kdl_parser/kdl_parser
git archive --remote=git@git.hb.dfki.de:dfki-control/wbc/package_set.git HEAD patches/kdl_parser.patch | tar -x
git apply patches/kdl_parser.patch
mkdir build && cd build 
cmake .. 
sudo make -j8 install && cd ../..

# If not done yet, setup a ssh key pair using the command `ssh-keygen` and add the 
# key from `~/.ssh/id_rsa.pub `to the keys in your Gitlab account.

# qpOASES
git clone https://github.com/coin-or/qpOASES.git -b releases/3.2.0
cd qpOASES
git archive --remote=git@git.hb.dfki.de:dfki-control/wbc/package_set.git HEAD patches/qpOASES.patch | tar -x
git apply patches/qpOASES.patch
mkdir build && cd build
cmake .. && sudo make -j8 install && cd ../..

# WBC
git clone git@git.hb.dfki.de:dfki-control/wbc/wbc.git 
mkdir wbc/build && cd wbc/build
cmake ..
sudo make -j8 install

