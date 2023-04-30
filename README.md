# KUKA_YouBot

### 4. Complete the following tasks by filling in the python code templates in the packages "cw1/cw1q4-srv" and "cw1/cw1q4" to create services that perform representation transformations. 

a. Fill in the template in package "cwl/cw1q4-srv" with the appropriate request and response message types for each service.
The quaternion message is "geometry-msgs/Quaternion", namely 'q'.
• The Rodrigues representation message is three "std-msgs/FIoat64" values, namely "x" , "y" and "z". 
• The Z-Y-X Euler angle representation message is three "std-msgs/FIoat64" values, namely "z" ,"y"and "x". 

b. Fill in the template in package "cwl/cwIq4" to create a service that converts a quaternion representation to an Euler angle Z-Y-X representation (Tait-Bryan, extrinsic). Your request should contain the quaternion you need to convert, whereas your response should store the requested Euler angles. 

c. Fill in the template in package "cwl/cwlq4" to create a service that converts a quaternion representation to a Rodrigues representation. Your request should contain the quaternion you need to convert, whereas your response should store the requested Rodriguez representation.

### 5. d. Complete this task by filling in the 'cwlq5d-node.py' code template, inside the package "cw1/cwlq5". Write a ROS script to compute the forward kinematics based on the URDF description. To complete this assignment, you must do the following: 
• Fill the "youbot-dh-parameters" dictionary with the youbot DH parameters.
• Fill the "youbot-joint-offsets" dictionary to account for the joint offsets between the you found and the xarco representation. 
• Implement the 'tkine-wrapper()' function and initialize the subscriber.

![](https://github.com/akshetP/KUKA_YouBot/blob/main/KUKA%20YouBot.jpeg) <br>
