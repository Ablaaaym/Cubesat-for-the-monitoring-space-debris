# Cubesat-for-the-monitoring-space-debris

APPOINTMENT
------------
This code is intended to implement a stabilization system and a machine vision system for collecting data on space debris.

INSTALLATION
------------
The Stabilization.сpp file is intended for all Arudino microcontrollers using the l298n motor driver. Connecting components to arduino is described in the file.

The ObjectDetection.py file is intended for the Raspberry share and the camera connected to it. Make sure you have the OpenCV and Numpy library installed.

The MapDemonstration.py file is designed to run on a PC and visualizes the data obtained from the raspberry share. Before starting, also make sure that the OpenCV and Numpy libraries are installed.

When starting, make sure that the host in ObjectDetection.py and MapDemonstration.py share matches your ip address

WHAT's NEXT
------------
Algorithms for calculating the orbit of the spacecraft itself and the space objects detected by it are under development.
