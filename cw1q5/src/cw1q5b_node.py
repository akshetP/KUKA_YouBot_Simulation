#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

"""

To complete this assignment, you must do the following:
    - Fill the "youbot_dh_parameters" dictionary with the youbot DH parameters you found in question 5a
    - Complete the definition of standard_dh().
    - Complete the definition of forward_kinematics().
    - Complete the definition of fkine_wrapper(). To do this you should use your implementation of standard_dh() and forward_kinematics(). 
    - Initialise the subscriber to the topic that publishes joint states and its callback function fkine_wrapper()

In case you need to implement additional functions, define them in this file.

Complete the function implementation within the indicated area and do not modify the
assertions. The assertions are there to make sure that your code will accept and return
data with specific types and formats.

You can use code you developed during previous lab sessions, just make sure you adapt to follow this template.

** Important, you can use the cw1q5b.launch file to visualize the frames. The frames reported from 
your code are not supposed to match the preloaded youbot. This is why we have hidden it intentionally. 
Try to move the sliders in the joint_state_publisher gui and observe your robot's behaviour. If you code 
is correct, when you specify zero angles for all the joints using the robot state publisher, you should 
see all the frames stacked in the z axis (the home position).
"""

# TODO: populate the values inside the youbot_dh_parameters dictionary with the ones you found in question 5a.
youbot_dh_parameters = {'a':[0.0, 0.155, 0.135, 0.0, 0.0], #using 0.0 to represent float values to prevent data-type errors.
                        'alpha': [-np.pi/2, 0.0, 0.0, np.pi/2, 0.0],
                        'd' : [0.147, 0.0, 0.0, 0.0, 0.218],
                        'theta' : [0.0, -np.pi/2, 0.0, np.pi/2, 0.0]}


# Function for rotation matrix to quaternion conversion
def rotmat2q(T):
    q = Quaternion()
    tr = np.trace(T)
    if tr == 4:
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        return q
    angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1)/2)
    xr = T[2, 1] - T[1, 2]
    yr = T[0, 2] - T[2, 0]
    zr = T[1, 0] - T[0, 1]
    x = xr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    y = yr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    z = zr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    q.w = np.cos(angle/2)
    q.x = x * np.sin(angle/2)
    q.y = y * np.sin(angle/2)
    q.z = z * np.sin(angle/2)
    return q



def standard_dh(a, alpha, d, theta):
    # TODO complete the function

    """This function computes the homogeneous 4x4 transformation matrix T_i based 
    on the four standard DH parameters associated with link i and joint i.

    Args:
        a ([int, float]): Link Length. The distance along x_i ( the common normal) between z_{i-1} and z_i
        alpha ([int, float]): Link twist. The angle between z_{i-1} and z_i around x_i.
        d ([int, float]): Link Offset. The distance along z_{i-1} between x_{i-1} and x_i.
        theta ([int, float]): Joint angle. The angle between x_{i-1} and x_i around z_{i-1}

    Returns:
        [np.ndarray]: the 4x4 transformation matrix T_i describing  a coordinate 
        transformation from the concurrent coordinate system i to the previous coordinate system i-1
    """
    assert isinstance(a, (int, float)), "wrong input type for a"
    assert isinstance(alpha, (int, float)), "wrong input type for =alpha"
    assert isinstance(d, (int, float)), "wrong input type for d"
    assert isinstance(theta, (int, float)), "wrong input type for theta"
    A = np.zeros((4, 4))

    # your code starts here -----------------------------
    A[0, 0] = np.cos(theta) #computing each element of the 4x4 matrix (Source: Lecture_4_Robotic_Representations_and_Forward_Kinematics - Slide Number 16)
    A[0, 1] = -np.sin(theta)*np.cos(alpha)
    A[0, 2] = np.sin(theta)*np.sin(alpha)
    A[0, 3] = a*np.cos(theta)

    A[1, 0] = np.sin(theta)
    A[1, 1] = np.cos(theta)*np.cos(alpha)
    A[1, 2] = -np.cos(theta)*np.sin(alpha)
    A[1, 3] = a*np.sin(theta)

    A[2, 1] = np.sin(alpha)
    A[2, 2] = np.cos(alpha)
    A[2, 3] = d

    A[3, 3] = 1.0
    # your code ends here ------------------------------

    assert isinstance(A, np.ndarray), "Output wasn't of type ndarray"
    assert A.shape == (4,4), "Output had wrong dimensions"
    return A

def forward_kinematics(dh_dict, joints_readings, up_to_joint=5):
    # TODO complete the function

    """This function solves the forward kinematics by multiplying frame 
    transformations up until a specified frame number. The frame transformations
     used in the computation are derived from the dh parameters and joint_readings. 

    Args:
        dh_dict (dict): A dictionary containing the dh parameters describing the robot.
        joints_readings (list): the state of the robot joints. For youbot those are revolute.
        up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks. Defaults to 5.

    Returns:
        np.ndarray: A 4x4 homogeneous tranformation matrix describing the pose of frame_{up_to_joint} w.r.t the base of the robot.
    """
    assert isinstance(dh_dict, dict)
    assert isinstance(joints_readings, list)
    assert isinstance(up_to_joint, int)
    assert up_to_joint>=0
    assert up_to_joint<=len(dh_dict['a'])
    
    T = np.identity(4)
    # your code starts here ------------------------------
    for i in range(up_to_joint):
        Ti = standard_dh(dh_dict['a'][i], dh_dict['alpha'][i], dh_dict['d'][i], dh_dict['theta'][i] + joints_readings[i]) #setting the transformation matrix to multiply each transformation of the frame in loop for all the frames.
        T = np.matmul(T,Ti) #matrix multiplying each frame to calculate the position of the end effector with reference to frame 0
    # your code ends here -------------------------------

    assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
    assert T.shape == (4,4), "Output had wrong dimensions"
    return T

def fkine_wrapper(joint_msg, br):
    # TODO complete the function
    """This function integrates your robotics code with ROS and is responsible 
        to listen to the topic where joint states are published. Based on this,
        compute forward kinematics and publish it.
        In more detail this function should perform the following actions:
        - get joint angles from the rostopic that publishes joint data
        - Publish a set of transformations relating the frame 'base_link' and
            each frame on the arm 'arm5b_link_i' where i is the frame, using
            tf messages.
    Args:
        joint_msg (JointState): ros msg containing the joint states of the robot
        br (TransformBroadcaster): a tf broadcaster
    """
    assert isinstance(joint_msg, JointState), "Node must subscribe to a topic where JointState messages are published"
    # your code starts here ------------------------------
     # callback function
    transform = TransformStamped() #calling the Transfrom messages

    #This loop should iterate through all joints. joint frame 5.
    for i in range(5):

        #for every joint the forward kinematics are calculated in a loop
        arm_link = forward_kinematics(youbot_dh_parameters, list(joint_msg.position), i+1)

        #Here we define the transform from joint frame 0 to joint frame 1 using the tf2 broadcaster.
        #define the transfrom timestamp
        transform.header.stamp = rospy.Time.now()
        #define the transfrom header frame (parent frame)
        transform.header.frame_id = 'base_link'
        #define the transfrom child frame
        transform.child_frame_id = 'arm_link_'+str(i+1)
        #populate the transform field. It consists of translation and rotation.
        transform.transform.translation.x = arm_link[0, 3] #setting the tf in the matrix
        transform.transform.translation.y = arm_link[1, 3]
        transform.transform.translation.z = arm_link[2, 3]
        transform.transform.rotation = rotmat2q(arm_link) #converting to quaternion
        #broadcast the transform to tf2
        br.sendTransform(transform)
    # your code ends here ------------------------------

def main():
    rospy.init_node('forward_kinematic_node')
    
    #Initialize your tf broadcaster. 
    br = TransformBroadcaster()
    
    # TODO: Initialize a subscriber to the topic that 
    # publishes the joint angles, configure it to have fkine_wrapper 
    # as callback and pass the broadcaster as an additional argument to the callback
    
    # your code starts here ------------------------------
    sub = rospy.Subscriber('/joint_states', JointState, fkine_wrapper, br) #subscribing the topic of join_states and broadcasting the forward kinematics values
    # your code ends here ----------------------
    
    rospy.spin()


if __name__ == "__main__":
    main()