#!/bin/bash


echo " - installing dependences for component mqtt"

sudo apt install -y \
      mosquitto \
      mosquitto-dev \
      mosquitto-clients \
      libmosquitto-dev 


echo " " 
echo " - done"
echo " "