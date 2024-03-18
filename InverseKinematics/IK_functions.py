#! /usr/bin/env python3

"""
    # {Adele Robaldo}
    # {robaldo@kth.se}
"""

from math import atan2, sin,cos,sqrt, acos, asin

def scara_IK(point):
    
    l0 = 0.07
    x = point[0] - l0
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]
    
    l1 = 0.3
    l2 = 0.35
    
    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2*(l1*l2))
    
    q[1] = acos(c2) 
    s2 = sin(q[1]) 
    q[0] = atan2(y,x) - atan2(l2*s2,l1 + l2*c2)
    
    q[2] = z
    
    return q


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2] - 0.311 - 0.078
    
    beta=atan2(-R[2][0],sqrt(R[0][0]**2+R[1][0]**2))
    alpha=atan2(R[1][0]/cos(beta),R[0][0]/cos(beta))
    gamma=atan2(R[2][2]/cos(beta),R[2][2]/cos(beta))
    
    joint_positions=[alpha, beta, gamma, 0.0, 0.0, 0.0, 0.0]
    
    q = joint_positions #it must contain 7 elements

    
    
    return q
