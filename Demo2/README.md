# Demo 2

This folder contains code used for the SEED Lab "Demo 2" project.

In this folder, there is code for the Arduino which has implements multiple PID controllers to perform rotation and forward movement of the robot.
There is also code for the Raspberry Pi which communicates a markers angle and distance to a marker.
These programs can be found under `arduino_program` and `pi_program.py`.

There is also Python code for the Raspberry Pi which finds a marker and calculates the angle and distance to that marker.
This program can be found under `piCode` as `piCode.py`.

These subsystems are not integrated together for this demo, and have to be run seperatly at the moment.
