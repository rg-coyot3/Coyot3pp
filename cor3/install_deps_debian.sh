#!/bin/bash



echo " - installing dependences for Coyot3pp::Cor3"

sudo apt update
sudo apt upgrade
sudo apt install -y \
        libjsoncpp-dev \
        rapidjson-dev \
        libgeographic-dev \
        liboping-dev
        