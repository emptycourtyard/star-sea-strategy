# -*- coding: utf-8 -*-
"""
Created on Wed Jun 23 01:18:50 2021

@author: emptycourtyard
"""

import numpy as np
import math as m


# Setup the composition and starting arrangement of the fleet. Returns array
# in the following format: np.array([[i0, S0, E0, A0, x0, y0, z0, u0, v0, w0]
#                                    [i1, S1, E1, A1, x1, y1, z1, u1, v1, w1]
#                                                       ...                  ])
# i -> unique idenitifying (integer)
# S -> Screen Shield Rating (1-7)
# E -> Expulsion Engine Rating (1-7)
# A -> Accelerator Armament Rating (1-7)
# x -> x position (0-100000)
# y -> y position (0-100000)
# z -> z position (0-100000)
# u -> initial x acceleration (-100-100)
# v -> initial y acceleration (-100-100)
# w -> initial z acceleration (-100-100)
#
# Tip: (i) can be used to group select pieces for later use.
#
# This function is called once at the start of game in board_setup.
#
def set_fleet_pieces():
    return -1
    
# Move each piece of the fleet by a change in accleration. Given parameters
# red_pieces and blue_pieces, return array in the following format:
# np.array([[i0, u0, v0, w0]
#           [i1, u1, v1, w1]
#                  ...      ])
# Array matrix must be same size as arr, with each index matching unique id.
# No change in accelration can be represented with u=0, v=0, w=0
#
# i -> unique idenitifying (integer)
# u -> change in x acceleration (-E-E)
# v -> change in y acceleration (-E-E)
# w -> change in z acceleration (-E-E)
#
# This function is called each turn in board_step_move. 
#
def step_move_pieces(red_pieces, blue_pieces):
    return -1
    
# Set the direction of attack in the shape of a beam. Given parameters
# red_pieces and blue_pieces, return attay in the following format:
# np.array([[i0, u0, v0, w0]
#           [i1, u1, v1, w1]
#                  ...      ])
# Array matrix must be same size as arr, with each index matching unique id.
# No attack can be represented with u=0, v=0, w=0
#
# i -> unique idenitifying (integer)
# u -> change in x acceleration (-E-E)
# v -> change in y acceleration (-E-E)
# w -> change in z acceleration (-E-E)
#
# Tip: it is simple to target a specific x,y,z coordinate instead of direction 
# by translating u, v, z by the corresponding piece's position x, y, z.
#
# This function is called each turn in board_step_attack. 
#
def step_attack_pieces():
    return -1