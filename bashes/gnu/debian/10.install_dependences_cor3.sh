#!/bin/bash



echo " - Installing dependences for Coyot3pp::Cor3"
echo " "
sleep 1

sudo apt update
sudo apt install \
          build-essential \
          cmake \
          pkg-config \
          libjsoncpp-dev \
          libyaml-cpp-dev \
          liboping-dev \
          libgeographiclib-dev \
          rapidjson-dev
