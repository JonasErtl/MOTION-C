## MOTION - Movement Observation and Tracking using IMU On-Hand Navigation -- Rewrite in C

I already have written some Code for this in Python but I decided to rewrite the project in C. I originally wrote this for a RPi but i find the ESP32 now to be the more elegant soution. 

The aim of this project is to build a system which can detect Hand-Movement and then further process it. This might include visualizing the motion, using it as some sort of movement input, or even running the Data through a simple Neural Network to recogize letters and shapes which are drawn. 

## Progress
Currently the program can calibrate and read the gyroscope and accelerometer data, and then combine them using a complementary filter algorithm to determine the pitch roll and yaw of the sensor. I might now now try to visualize this. 


