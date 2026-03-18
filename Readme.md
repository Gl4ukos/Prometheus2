This code operates a 4DoF, 3D printed, robotic arm called Prometheus.
The directory named "Prometheus2_embedded" contains all code already existent on the arm's microcontroller (Raspberry pi Pico),
while the scripts outside of it have not yet been integrated, although completely functional.

![alt text](IMG_3103b_small.PNG)
![alt text](IMG_3103a_small.PNG)

The config file, simply includes a python dictionary that contains all information about the arm, such as link lengths, joint angle limits etc.

The "prometheus2_kinematics_3d.py" script, uses the config information to then estimate the Inverse Kinematics of the arm numerically using Jacobian.
It manages to move the entire arm just by receiving the end effector's coordinates, which can be changed by small intervals using keystrokes,
the movement of the arm is displayed in real time by a 3D python plot.

!! This code despite being functional has not been imported to the microcontroller because there is no numpy librady for pico, which is used extensively in the script.

TODO: 
-> Find and compile firmware for the pico, which includes "ulab", to support linear algebra calculations.
-> Import Inverse Kinematics code to pico


