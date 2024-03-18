#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Adele Robaldo}
# {u14b14ek}
# {robaldo@kth.se}

from dubins import *
from dubins import Car
from math import*
car = Car()
counter=0
"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

"""

import math
import random

import numpy as np

from RRT_star import RRTStar,RRT
show_animation = True

use_RRT= True
use_RRTStar=False
straight_angle=0.15
def calc_angle(rx,ry,x,y,theta):
    dx = rx - x
    dy = ry - y
    
    alpha = atan2(dy,dx)

    if(alpha>theta):
        phi=+pi/4
    if(alpha<theta):
        phi=-pi/4
    if(abs(alpha-theta)<straight_angle):
        phi=0

    return phi     
   

def sat_phi(phi):
    if(phi>pi/4):
      phi=pi/4
    elif(phi<-pi/4):
      phi=-pi/4
    return phi  

def collisions(x,y,car):
    for obs in car.obs:
        d = math.sqrt((x - obs[0])**2 + (y - obs[1])**2)
        if d <= obs[2]:
            return True
    return False

def solution(car):
    global counter
    phi=0
    controls = []
    times = []
    rx=[]
    ry=[]
    counter=counter+1
    '''
    Your code below
    '''
    print(" ") 
    print("Next Solution") 

    # start and goal position
    sx = car.x0  #start x [m]
    sy = car.y0  # start y[m]
    gx = car.xt  #goal x [m]
    gy = car.yt  #goal y [m]
    xl=car.xlb   #lower bound
    yl=car.ylb
    xu=car.xub  #upper bound
    yu=car.yub
    # initial state
    theta = 0
    
    # set obstacle positions
    ox, oy = [], []
    max_obs=0
    for i in range(0,len(car.obs)):
        xobs=car.obs[i][0]
        yobs=car.obs[i][1]
        robs=car.obs[i][2]
        ox.append(xobs)
        oy.append(yobs)
        if i==0:
            max_obs=(robs)
        else:
            max_obs=max(max_obs,robs)  
    #print("Obs max dim")
    #print(max_obs)



        # ====Search Path with RRT====
    obstacleList = car.obs #[x, y, radius]
  # Set Initial parameters
    if use_RRTStar:  
        path_dist=0.3
        delta_t=0.05
        rrt = RRTStar(
            start=[sx, sy],
            goal=[gx, gy],
            rand_area=[xl, xu],
            obstacle_list=obstacleList,
            robot_radius=0.25,
            max_iter=10000,
            goal_sample_rate=10,
            path_resolution=path_dist,
            connect_circle_dist=path_dist*1.8,
            expand_dis=path_dist*1.5,
            play_area=[xl, xu, yl, yu]
            )
    if use_RRT:
        path_dist=1.1
        delta_t=0.03
        straight_angle=0.05
        rrt = RRT(
                start=[sx, sy],
                goal=[gx, gy],
                rand_area=[xl, xu],
                obstacle_list=obstacleList,
                play_area=[xl, xu, yl, yu] ,
                max_iter=15000,
                goal_sample_rate=10,
                path_resolution=path_dist,
                expand_dis=path_dist*1.5,
                robot_radius=0.45
                )

  
    path = rrt.planning(animation=False)
  
    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

    
    #copy path x y position 
    for i in range(0,len(path)):
        rx.append(path[i][0])
        ry.append(path[i][1])
    
    
    x=sx
    y=sy
    phi=sat_phi(calc_angle(rx[len(rx)-2],ry[len(rx)-2],x,y,0))
        
    controls.append(phi)
    times.append(0.0)
    time=0.0
    
    for i in range(0,len(rx)):
        #print(len(rx))
        #print(i) 
        j=len(rx)-1-i
        #print(j)
        # compute next state after 0.01 seconds
        #print(x-rx[j]) 
        #print(y-ry[j])
        if i>(len(rx)-5):
            print("GOAL REACHED") 
        p_dist=sqrt(((x-rx[j])**2) + ((y-ry[j])**2))
        if(p_dist>path_dist*4):
                print("CRASH")
                break
        
        while p_dist>path_dist*1.1:
            if(p_dist>path_dist*3.5):
                print("Diverging!!")
                break
            
            xn, yn, thetan = step(car, x, y, theta, phi,dt=delta_t)
            while thetan >= math.pi:
                thetan -= 2*math.pi
            while thetan <= -2*math.pi:
                thetan += math.pi   
            #print(thetan)  
            #Limit and compute
            phi=sat_phi(calc_angle(rx[j],ry[j],xn,yn,thetan))
            #print(phi)
            
            #Verify Collision    
            if collisions(xn,yn,car): 
                print("Close to Obstacle,next")
                #break       

            #next state
            x=xn
            y=yn
            theta=thetan
            
            p_dist=sqrt( ( (x-rx[j])**2 ) + ( (y-ry[j])**2 ) )
            time=time+delta_t
            # assemble path
            controls.append(phi)
            times.append(time)

    '''
    
    Your code above
    '''
    # len(controls) == len(times) - 1
    
    
    time=time+delta_t
    times.append(time)
    return controls, times