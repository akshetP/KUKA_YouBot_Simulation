#!/usr/bin/env python3

from inspect import trace
from urllib import response
import rospy
import numpy as np

# TODO: Include all the required service classes

# your code starts here -----------------------------
from cw1q4_srv.srv import quat2zyx, quat2zyxResponse, quat2rodrigues, quat2rodriguesResponse, quat2zyxRequest, quat2rodriguesRequest
# your code ends here -------------------------------

def convert_quat2zyx(request):
    # TODO complete the function
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (quat2zyxRequest): cw1q4_srv service message, containing
        the quaternion you need to convert.

    Returns:
        quat2zyxResponse: cw1q4_srv service response, in which 
        you store the requested euler angles 
    """
    assert isinstance(request, quat2zyxRequest)

    # Your code starts here ----------------------------
    response = quat2zyxResponse()
    qx = request.q.x
    qy = request.q.y
    qz = request.q.z
    qw = request.q.w

    qxx = qx * qx
    qyy = qy * qy
    qzz = qz * qz
    
    r11 = 1 - 2*qyy - 2*qzz
    r12 = 2*qx*qy - 2*qz*qw
    r13 = 2*qx*qz + 2*qy*qw
    r21 = 2*qx*qy + 2*qz*qw
    r22 = 1 - 2*qxx - 2*qzz
    r23 = 2*qy*qz - 2*qx*qw
    r31 = 2*qx*qz - 2*qy*qw
    r32 = 2*qy*qz + 2*qx*qw
    r33 = 1 - 2*qxx - 2*qyy

    cosBeta = np.sqrt(r23*r23 +r33*r33)
    beta = np.arctan2(r13, cosBeta )

    #due to double representation
    if abs(cosBeta) > 10**-6: #when cos beta approaches 0
        if beta > -np.pi/2 and beta < np.pi/2:
            x_star = np.arctan2(-r12, r11 )
            y_star = np.arctan2(r13, cosBeta )
            z_star = np.arctan2(-r23, r33 )
        elif beta > np.pi/2 and beta < np.pi*3: #when beta is between 90deg or 270deg
            x_star = np.arctan2(r12, -r11 )
            y_star = np.arctan2(r13, -cosBeta )
            z_star = np.arctan2(r23, -r33 )
        else:
            x_star = 0.0
            y_star = np.arctan2(r13, cosBeta )
            z_star = np.arctan2(r21, r22 )
    else:
        x_star = 0.0
        y_star = np.arctan2(r13, cosBeta )
        z_star = np.arctan2(r21, r22 )

    response.z.data = z_star
    response.y.data = y_star
    response.x.data = x_star
    
    # Your code ends here ------------------------------
    assert isinstance(response, quat2zyxResponse)
    return response


def convert_quat2rodrigues(request):
    # TODO complete the function

    """Callback ROS service function to convert quaternion to rodrigues representation
    
    Args:
        request (quat2rodriguesRequest): cw1q4_srv service message, containing
        the quaternion you need to convert

    Returns:
        quat2rodriguesResponse: cw1q4_srv service response, in which 
        you store the requested rodrigues representation 
    """
    assert isinstance(request, quat2rodriguesRequest)
    # Your code starts here ----------------------------

    response = quat2rodriguesResponse()
    #requesting quaternion values
    qx = request.q.x 
    qy = request.q.y
    qz = request.q.z
    qw = request.q.w

    #calculating the vector normal of the quaternion matrix
    vector_norm = qx * qx + qy * qy + qz * qz

    if vector_norm > 0: #checking if the normal is greater than 0
        sin_theta = np.sqrt(vector_norm)
        factorization = 2*np.arctan2(sin_theta , qw)/sin_theta
        x_v = qx *  factorization 
        y_v = qy *  factorization 
        z_v = qz *  factorization
    else: 
        factorization = 2 #checking if factorization is approaching zero or no
        x_v = qx * factorization 
        y_v = qy * factorization 
        z_v = qz*  factorization
    #printing the results
    response.x.data = x_v
    response.y.data = y_v
    response.z.data = z_v
    # Your code ends here ------------------------------

    assert isinstance(response, quat2rodriguesResponse)
    return response

def rotation_converter():
    rospy.init_node('rotation_converter')

    #Initialise the services
    rospy.Service('quat2rodrigues', quat2rodrigues, convert_quat2rodrigues)
    rospy.Service('quat2zyx', quat2zyx, convert_quat2zyx)

    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
