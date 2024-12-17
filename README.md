## MOTION - Movement Observation and Tracking using IMU On-Hand Navigation -- Rewrite in C

I already have written some Code for this in Python but I decided to rewrite the project in C. So far it's only possible to read sensor data coming from the MPU6050 connected to an ESP32. I originally wrote this for a RPi but i find the ESP32 now to be the more elegant soution. 

The aim of this project is to build a system which can detect Hand-Movement and then further process it. This might include visualizing the motion, using it as some sort of movement input, or even running the Data through a simple Neural Network to recogize letters and shapes which are drawn. 

Step 1: At first there needs to be a program which is run on a mirco-contoller mounted on a persons Hand. This program will need to capture the IMU Data in real time and send them to another client, ideally over wifi, for further processing.  

# The Gyro Calibration Program
This program determines the bias that the Gyroscope has. But is still not working for this rewrite.

