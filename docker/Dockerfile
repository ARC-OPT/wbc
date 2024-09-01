FROM ubuntu:22.04
WORKDIR /root 
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get -y install git cmake build-essential libboost-system-dev libboost-program-options-dev libboost-thread-dev libboost-test-dev pkg-config libeigen3-dev libboost-filesystem-dev
WORKDIR /root
COPY install.sh /install.sh
RUN /install.sh
