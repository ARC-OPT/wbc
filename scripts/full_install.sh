#!/bin/sh
sudo apt-get -y install git cmake build-essential libboost-system-dev libboost-program-options-dev libboost-thread-dev libboost-test-dev pkg-config libeigen3-dev libboost-filesystem-dev

# cmake makros
git clone https://github.com/rock-core/base-cmake.git   
mkdir base-cmake/build && cd base-cmake/build
cmake .. && make -j8 && sudo make install && cd ../..

# Logging 
git clone https://github.com/rock-core/base-logging.git
mkdir base-logging/build && cd base-logging/build
cmake .. && make -j8 && sudo make install && cd ../..

# Base Types
git clone https://github.com/rock-core/base-types.git
mkdir base-types/build && cd base-types/build
cmake .. -DUSE_SISL=OFF -DBINDINGS_RUBY=OFF -DROCK_VIZ_ENABLED=OFF
make -j8 && sudo make install && cd ../..

# URDF
sudo apt-get -y install liburdfdom-headers-dev liburdfdom-dev 

# Clone WBC repo to have the patches for KDL and qpOASES
git clone https://github.com/ARC-OPT/wbc.git

# RBDL
git clone --branch v3.2.1 --recurse-submodules https://github.com/rbdl/rbdl.git
cd rbdl
git apply ../wbc/patches/rbdl.patch --ignore-whitespace
mkdir build && cd build
cmake .. -DRBDL_BUILD_ADDON_URDFREADER=ON
make -j8 && sudo make install && cd ../..

# KDL 
git clone https://github.com/orocos/orocos_kinematics_dynamics.git -b v1.5.1
mkdir orocos_kinematics_dynamics/orocos_kdl/build
cd orocos_kinematics_dynamics/orocos_kdl/build
cmake .. 
make -j8 && sudo make install &&  cd ../../..

# Pinocchio
git clone --branch v2.6.8 --recurse-submodules https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio
mkdir build && cd build
cmake .. -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_UNIT_TESTS=OFF 
make -j8 && sudo make install && cd ../..

# KDL parser
sudo apt-get -y install libtinyxml2-dev
git clone https://github.com/ros/kdl_parser.git -b 1.14.1
cd kdl_parser/kdl_parser
mkdir patches && cp ../../wbc/patches/kdl_parser.patch patches
git apply patches/kdl_parser.patch
mkdir build && cd build 
cmake .. 
make -j8 && sudo make install && cd ../../..

# If not done yet, setup a ssh key pair using the command `ssh-keygen` and add the 
# key from `~/.ssh/id_rsa.pub `to the keys in your Gitlab account.

# qpOASES
git clone https://github.com/coin-or/qpOASES.git -b releases/3.2.0
cd qpOASES
mkdir patches && cp ../wbc/patches/qpOASES.patch patches
git apply patches/qpOASES.patch
mkdir build && cd build
cmake .. && make -j8 && sudo make install && cd ../..

# eiquadprog
git clone --recurse-submodules https://github.com/stack-of-tasks/eiquadprog.git -b v1.2.5
cd eiquadprog
cp ../wbc/patches/eiquadprog.patch . && git apply eiquadprog.patch
mkdir build && cd build
cmake ..
make -j8 && sudo make install && cd ../..

# qpSWIFT
git clone https://github.com/qpSWIFT/qpSWIFT.git
cd qpSWIFT
cp ../wbc/patches/qpSWIFT.patch . && git apply qpSWIFT.patch
mkdir build && cd build
cmake .. 
make -j8 && sudo make install && cd ../.. 

# proxQP
git clone --recurse-submodules https://github.com/Simple-Robotics/proxsuite.git proxqp
cd proxqp
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF
make -j8 && sudo make install && cd ../..

# OSQP
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake ..
make -j8 && sudo make install && cd ../..
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make -j8 && sudo make install && cd ../..

# WBC
mkdir wbc/build && cd wbc/build
cmake .. -DROBOT_MODEL_RBDL=ON -DROBOT_MODEL_KDL=ON -DSOLVER_PROXQP=ON -DSOLVER_EIQUADPROG=ON -DSOLVER_QPSWIFT=ON -DSOLVER_OSQP=ON -DCMAKE_BUILD_TYPE=RELEASE
make -j8 && sudo make install && cd ..

sudo ldconfig
