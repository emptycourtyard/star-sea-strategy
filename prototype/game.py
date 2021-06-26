# -*- coding: utf-8 -*-
"""
Created on Tue Jun 22 22:33:20 2021

@author: Fractal
"""

import numpy as np
import math as m
from red_controller import*
from blue_controller import*

ship_half_l = 500 / 2
ship_half_h_w= 100 / 2

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = np.sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

# Returns Numpy array of size v1 and v2 with the min value of each index    
def vmin(v1, v2):
    result = np.empty(np.size(v1))
    for i in range(np.size(result)):
        result[i] = min(v1[i], v2[i])
    return result    
    
# Returns Numpy array of size v1 and v2 with the max value of each index
def vmax(v1, v2):
    result = np.empty(np.size(v1))
    for i in range(np.size(result)):
        result[i] = max(v1[i], v2[i])
    return result

# Returns Numpy array of size v with the reciprocal of each index 
def vreciprocal(v):
    result = np.empty(np.size(v))
    for i in range(np.size(result)):
        if v[i] == 0:
            result[i] = -2147483648
        else:
            result[i] = 1.0 / v[i]
    return result

ray_max_range = 2147483647

# Andrew Kensler/Roman Wiche varaint of the ray-AABB slab method for 
# detectiing collison. Returns true if ray intersects axis-aligned box.
def ray_AABB(r_dir, r_pos, b_pos, b_pos2):
    invD = vreciprocal(r_dir)
    t0s = (b_pos - r_pos) * invD
    t1s = (b_pos2 - r_pos) * invD
    tsmaller = vmin(t0s, t1s)
    tbigger  = vmax(t0s, t1s)
    tmin = 0
    tmax = ray_max_range
    tmin = max(tmin, max(tsmaller[0], max(tsmaller[1], tsmaller[2])))
    tmax = min(tmax, min(tbigger[0], min(tbigger[1], tbigger[2])))
    return (tmin < tmax)

# Return the 3x3 rotation matrix that rotates vector v1 to align in the same 
# direction as vector v2 in row-major form. 
def rotate_align(v1, v2):
    axis = np.cross(v1, v2)
    cosA = np.dot(v1, v2)
    k = 1.0 / (1.0 + cosA)
    result = np.array([[(axis[0] * axis[0] * k) + cosA,
                        (axis[1] * axis[0] * k) - axis[2], 
                        (axis[2] * axis[0] * k) + axis[1]],
                       [(axis[0] * axis[1] * k) + axis[2],  
                        (axis[1] * axis[1] * k) + cosA,      
                        (axis[2] * axis[1] * k) - axis[0]],
                       [(axis[0] * axis[2] * k) - axis[1],  
                        (axis[1] * axis[2] * k) + axis[0],  
                        (axis[2] * axis[2] * k) + cosA]])
    return result


# Call rotate_align with x-axis as v2.
def axis_align(v):
    v = normalize(v)
    axis = np.array([1,0,0]);
    return rotate_align(v, axis);

def translate_ship_min(pos):
    x = pos[0] - ship_half_l
    y = pos[1] - ship_half_h_w
    z = pos[2] - ship_half_h_w
    return np.array([x, y, z])

def translate_ship_max(pos):
    x = pos[0] + ship_half_l
    y = pos[1] + ship_half_h_w
    z = pos[2] + ship_half_h_w
    return np.array([x, y, z])

##############################################################################

red_pieces = np.array([[]])
blue_pieces = np.array([[]])
pieces_validity = False
move_validity = False

def check_set_pieces_valditiy( arr ):
    return False

def check_step_move_valditiy( arr ):
    return False

def check_step_attack_valditiy( arr ):
    return False



def board_setup():
    submitted_red_pieces = red_controller.set_fleet_pieces()
    submitted_blue_pieces = blue_controller.set_fleet_pieces()
    
    if(check_set_pieces_valditiy(submitted_red_pieces) & 
       check_set_pieces_valditiy(submitted_blue_pieces)):
        pieces_validity = True
    else:
        print("Pieces arrangement invalid.")
    if pieces_validity:
        red_pieces = submitted_red_pieces
        blue_pieces = submitted_blue_pieces
    pieces_validity = False


def board_step_move():
    submitted_red_move = red_controller.step_move_pieces(red_pieces, blue_pieces)
    submitted_blue_move = blue_controller.step_move_pieces(red_pieces, blue_pieces)
    if(check_step_move_valditiy(submitted_red_move) & 
       check_step_move_valditiy(submitted_blue_move)):
        move_validity = True
    else:
        print("Movement invalid.")
    if move_validity:
        step_change_acceleration(red_pieces, submitted_red_move)
        step_change_acceleration(blue_pieces, submitted_blue_move)
    step_move(red_pieces)
    step_move(blue_pieces)
    move_validity = False
        

def board_step_attack():
    return -1



##############################################################################

def step_change_acceleration(pieces, acceleration):
    for i in range(np.size(pieces)):
        pieces[i][7] += acceleration[i][1]
        pieces[i][8] += acceleration[i][1]
        pieces[i][9] += acceleration[i][1]

def step_move(pieces):
    for i in range(np.size(pieces)):
        pieces[i][4] += pieces[i][7]
        pieces[i][5] += pieces[i][8]
        pieces[i][6] += pieces[i][9]







