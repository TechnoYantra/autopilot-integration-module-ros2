#!/usr/bin/env python3
import math 
import numpy as np
import matplotlib.pyplot as plt

def distance(lat1, lat2, lon1, lon2): 
    EARTH_RADIOUS = 6371
    lon1 = math.radians(lon1) 
    lon2 = math.radians(lon2) 
    lat1 = math.radians(lat1) 
    lat2 = math.radians(lat2) 
    dlon = lon2 - lon1  
    dlat = lat2 - lat1 
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))  
    return(c * EARTH_RADIOUS) 

def get_target_goal_on_path(X1, Y1, X2, Y2, xr, yr, offset):
    a = Y1 - Y2
    b = X2 - X1
    c = X1*Y2 - X2*Y1 # a*X1 + b*Y1
    m = (Y2 - Y1)/(X2 - X1)

    dn = abs((a * xr + b * yr + c)) / (math.sqrt(a * a + b * b)) # robot_normal_dist_from_path
    dr = math.sqrt((X1 - xr)**2 + (Y1 - yr)**2) # robot_dist_from_start_coordinate
    drp = math.sqrt((dr**2 - dn**2)) # robot_corresponding_dist_on_path
    D = math.sqrt((X2 - X1)**2 + (Y2 - Y1)**2) # path_length

    # xrp = (X2*drp)/D
    # yrp = (Y2*drp)/D

    x_target = ((X2-X1)*(drp+offset))/D + X1
    y_target = ((Y2-Y1)*(drp+offset))/D + Y1
    return x_target, y_target

def get_pose_between_wp(X1, Y1, X2, Y2):
    points = []
    D = math.sqrt((X2 - X1)**2 + (Y2 - Y1)**2) # path_length
    i = 0
    j = 0   
    for d in np.arange(0, D, 0.05):
        points.append([X1 + (d/D)*(X2-X1) ,Y1 + (d/D)*(Y2-Y1)])
        i = i+1
    return points


# x_target,y_target = get_target_goal_on_path(0,0,10,0,5,15,0)
# X1 = 0.0
# Y1 = 0.0
# X2 = 10.0
# Y2 = 10.0
# get_pose_between_wp(X1, Y1, X2, Y2)
# print(x_target,y_target)

