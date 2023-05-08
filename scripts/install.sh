#!/bin/sh
sudo apt-get -y install git cmake build-essential libboost-system-dev libboost-program-options-dev libboost-thread-dev libboost-test-dev pkg-config libeigen3-dev libboost-filesystem-dev

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
sudo apt-get -y install liburdfdom-headers-dev liburdfdom-dev 

# RBDL
git clone --branch v3.2.1 git@github.com:rbdl/rbdl.git
cd rbdl
mkdir build && cd build
cmake .. 
sudo make -j8 install && cd ../..

# Clone WBC repo to have the patches for KDL and qpOASES
git clone https://github.com/ARC-OPT/wbc.git

# If not done yet, setup a ssh key pair using the command `ssh-keygen` and add the 
# key from `~/.ssh/id_rsa.pub `to the keys in your Gitlab account.

# qpOASES
git clone https://github.com/coin-or/qpOASES.git -b releases/3.2.0
cd qpOASES
mkdir patches && cp ../wbc/patches/qpOASES.patch patches
git apply patches/qpOASES.patch
mkdir build && cd build
cmake .. && sudo make -j8 install && cd ../..

# WBC
mkdir wbc/build && cd wbc/build
cmake ..
sudo make -j8 install

