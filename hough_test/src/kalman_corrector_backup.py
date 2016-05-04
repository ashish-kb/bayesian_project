#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('hough_test')
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose2D,
    Point,
    Twist,
    TransformStamped,
    Quaternion,
)
from numpy import * 
from numpy.linalg import inv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def applyKalman():
    msg = rospy.wait_for_message("/vanishing_point", Pose2D)
    init_x = msg.x
    init_y = msg.y
    #time step of mobile movement
    print(init_x)
    print(init_y)
    dt = 0.079
    p = rospy.Publisher("/corrected_centers", Pose2D)
  #  p2 = rospy.Publisher("/measured_centers", Pose2D)
    # Initialization of state matrices 
    X = np.array([[init_y], [init_x],[1],[1]])
    #print(X.shape)
    P = diag((1, 1, 0.0, 0.0)) 
    A = np.array([[1,0,dt,0], [0,1,0,dt],[0,0,1,0],[0,0,0,1]]) 
    Q = diag((1, 1, 1, 1))
    B = eye(X.shape[0]) 
    U = zeros((X.shape[0],1))
     
    # Measurement matrices 

    Y = array([[X[0,0] + abs(np.random.randn(1)[0])], [X[1,0] +abs(np.random.randn(1)[0])]]) 
    H = array([[1, 0, 0, 0], [0, 1, 0, 0]]) 
    R = diag((5,5)) 

    # Applying the Kalman Filter 
    while(True): 
        (X, P) = kf_predict(X, P, A, Q, B, U)
        msg = rospy.wait_for_message("/vanishing_point", Pose2D)
        m_x = msg.x
        m_y = msg.y
        Y = array([[m_y],[m_x]])
        (X, P, K, IM, IS, LH) = kf_update(X, P, Y, H, R) 
        
        print("Y values")
        print(Y)
        print("X values")
        print(X)
        center = Pose2D()
        center.x = X[1,0]
        center.y = X[0,0]
        p.publish(center)


    
    



def kf_predict(X, P, A, Q, B, U): 
    X = dot(A, X) + dot(B, U) 
    P = dot(A, dot(P, A.T)) + Q 
    return(X,P) 

def kf_update(X, P, Y, H, R): 
    IM = dot(H, X) 
    IS = R + dot(H, dot(P, H.T)) 
    K = dot(P, dot(H.T, inv(IS))) 
    X = X + dot(K, (Y-IM)) 
    P = P - dot(K, dot(IS, K.T)) 
    LH = gauss_pdf(Y, IM, IS) 
    return (X,P,K,IM,IS,LH) 

def gauss_pdf(X, M, S): 
    if M.shape[1] == 1: 
        DX = X - tile(M, X.shape[1])   
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0) 
        E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(np.linalg.det(S)) 
        P = exp(-E) 
    elif X.shape[1] == 1: 
        DX = tile(X, M.shape[1])- M   
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0) 
        E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(np.linalg.det(S)) 
        P = exp(-E) 
    else: 
        DX = X-M   
        E = 0.5 * dot(DX.T, dot(inv(S), DX)) 
        E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(np.linalg.det(S)) 
        P = exp(-E) 
    return (P[0],E[0]) 
if __name__ == '__main__':  
	print('Initialized')
	rospy.init_node('kalmanCorrector', anonymous=True)
	applyKalman()
    

