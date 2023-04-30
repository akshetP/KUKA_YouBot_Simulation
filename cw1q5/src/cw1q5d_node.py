#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from cw1q5b_node import forward_kinematics, rotmat2q
from geometry_msgs.msg import TransformStamped, Quaternion # students need to add this

"""
To complete this assignment, you must do the following:
    - Fill the "youbot_dh_parameters" dictionary with the youbot DH parameters you 
        found in question 5c
    - Fill the "youbot_joint_offsets" dictionary to account for the joint offsets
        between the "youbot_dh_parameters" you found and the xarco representation.
    - Complete the definition of fkine_wrapper(). In this example you can use the
        functions you implemented for 5b, we already imported forward_kinematics() which
        you most definitely need. If you haven't implemented forward_kinematics()
        during 5b, your node will not work.
    - Initialise the subscriber to the topic that publishes joint states and its callback
    function fkine_wrapper()


You may need to implement additional functions, for instance to convert rotation
matrices to quaternions. If that's the case, define and implement all those functions
inside this file.

In case you need to implement additional functions, define them in this file.

Complete the function implementation within the indicated area and do not modify the
assertions. The assertions are there to make sure that your code will accept and return
data with specific types and formats.

You can use code you developed during previous lab sessions, just make sure you adapt to follow this template.

Remember, in /joint_states messages are describing joint encoder readings.
Depending on how the encoders are mounted and also how your dh parameters have been defined
You may need to modify the joint_states by either applying an offset, changing the
sign of the reported angle or both. We already asked you to define an offset dictionary
which you can apply directly to dh parameters, but you also need to change the polarity
of the angle reading in order for the robot to work properly.

Running the launch file associated with this question, you should see that your frames
fall on the exact joint positions of the Youbot.
"""

# TODO: populate the values inside the youbot_dh_parameters dictionary with the ones you found in question 5c.
youbot_dh_parameters = {'a':[0.033,0.155, 0.135,0.0, 0.0], #adding the new DH Parameters
                        'alpha': [-np.pi/2,0 ,0 ,np.pi/2 ,0 ],
                        'd' : [0.145, 0.0, 0.0,0 ,0.218 ],
                        'theta' : [0.0,-np.pi/2,0 ,np.pi/2 , 0]}

# TODO: populate the values inside the youbot_joint_offsets dictionary with the ones you found in question 5c.

youbot_joint_offsets = [np.pi*170/180,-np.pi*65/180, np.pi*146/180, -np.pi*102.5/180,np.pi*167.5/180]

youbot_dh_offset_paramters = youbot_dh_parameters.copy()

youbot_dh_offset_paramters['theta']=[theta + offset for theta, offset in zip(youbot_dh_offset_paramters['theta'], youbot_joint_offsets)]

youbot_joint_readings_polarity = [-1,1,1,1,1]

def fkine_wrapper(joint_msg, br):
    # TODO complete the function
    """This function integrates your robotics code with ROS and is responsible 
        to listen to the topic where joint states are published. Based on this,
        compute forward kinematics and publish it.
        
        In more detail this function should perform the following actions:
        - get joint angles from the rostopic that publishes joint data
        - Publish a set of transformations relating the frame 'base_link' and
            each frame on the arm 'arm5d_link_i' where i is the frame, using
            tf messages.

    Args:
        joint_msg (JointState): ros msg containing the joint states of the robot
        br (TransformBroadcaster): a tf broadcaster
    """
    assert isinstance(joint_msg, JointState), "Node must subscribe to a topic where JointState messages are published"
    
    # your code starts here ------------------------------
    #depending on the dh parameters you may need to change the sign of some angles here

    #set transform class according to tf2 format
    transform = TransformStamped()

    #define the link 
    link_i=["fkine_link_1","fkine_link_2","fkine_link_3","fkine_link_4","fkine_link_5"] #assigning the links
    
    #read the joint message position from the slider GUI
    joint_msg.position=list(joint_msg.position)

    #change the sign of message position based on reading polarity list 
    for j in range(5):
        joint_msg.position[j]=youbot_joint_readings_polarity[j]*joint_msg.position[j]

    #perform transformation with tf2 for from base_link to each joint
    for i in range(5):
        T_fk= forward_kinematics(youbot_dh_offset_paramters,joint_msg.position,i+1)
        #define the transfrom timestamp
        transform.header.stamp=rospy.Time.now()
        #define the transfrom header frame (parent frame)
        transform.header.frame_id='base_link'
        #define the transfrom child frame
        transform.child_frame_id=link_i[i]
        #populate the transform field. It consists of translation and rotation.
        transform.transform.translation.x=T_fk[0,3]
        transform.transform.translation.y=T_fk[1,3]
        transform.transform.translation.z=T_fk[2,3]
        transform.transform.rotation=rotmat2q(T_fk)

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
    sub=rospy.Subscriber('/joint_states',JointState,fkine_wrapper,br)
    # your code ends here ----------------------
    
    rospy.spin()

if __name__ == "__main__":
    main()
