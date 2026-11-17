#!/bin/sh
sudo apt-get -y install git cmake build-essential pkg-config libeigen3-dev libboost-system-dev libboost-test-dev pkg-config libeigen3-dev libboost-filesystem-dev libboost-serialization-dev

# URDF
sudo apt-get -y install liburdfdom-headers-dev liburdfdom-dev
# compatability with urdfdom >= 4.0.0
sudo apt-get -y install libtinyxml2-dev

# Clone WBC repo here to have the patches
git clone https://github.com/ARC-OPT/wbc.git

# Pinocchio
git clone --branch v2.6.8 --recurse-submodules https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio
mkdir build && cd build
cmake .. -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_UNIT_TESTS=OFF -DCMAKE_BUILD_TYPE=RELEASE
make -j && sudo make install && cd ../..

# RBDL
git clone --branch v3.2.1 --recurse-submodules https://github.com/rbdl/rbdl.git
cd rbdl
git apply ../wbc/patches/rbdl.patch --ignore-whitespace
mkdir build && cd build
cmake .. -DRBDL_BUILD_ADDON_URDFREADER=ON
make -j && sudo make install && cd ../..

# If not done yet, setup a ssh key pair using the command `ssh-keygen` and add the 
# key from `~/.ssh/id_rsa.pub `to the keys in your Gitlab account.

# qpOASES
git clone https://github.com/coin-or/qpOASES.git -b releases/3.2.0
cd qpOASES
mkdir patches && cp ../wbc/patches/qpOASES.patch patches
git apply patches/qpOASES.patch
mkdir build && cd build
cmake .. && make -j && sudo make install && cd ../..

# eiquadprog
git clone --recurse-submodules https://github.com/stack-of-tasks/eiquadprog.git -b v1.2.5
cd eiquadprog
cp ../wbc/patches/eiquadprog.patch . && git apply eiquadprog.patch
mkdir build && cd build
cmake ..
make -j && sudo make install && cd ../..

# proxQP
git clone --recurse-submodules https://github.com/Simple-Robotics/proxsuite.git proxqp
cd proxqp
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF -DPROXSUITE_BUILD_WITH_SERIALIZATION=OFF -DCMAKE_INSTALL_PREFIX=/usr/local
make -j && sudo make install && cd ../..

# OSQP
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake ..
make -j && sudo make install && cd ../..
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make -j && sudo make install && cd ../..

# HyRoDyn 
sudo apt install -y --no-install-recommends libgtest-dev
git clone --recursive https://github.com/dfki-ric/hyrodyn.git
cd hyrodyn
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
sudo make install && cd ../..

# Acados for HPIPM
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j
sudo make install

# WBC
mkdir -p wbc/build && cd wbc/build
cmake .. -DROBOT_MODEL_RBDL=ON -DSOLVER_PROXQP=ON -DSOLVER_EIQUADPROG=ON -DSOLVER_OSQP=ON -DROBOT_MODEL_HYRODYN=ON -DCMAKE_BUILD_TYPE=RELEASE
make -j && sudo make install && cd ..

sudo ldconfig
