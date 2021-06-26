# -*- coding: utf-8 -*-
"""
Created on Wed Jun 16 06:21:19 2021

@author: emptycourtyard
"""

import matplotlib.pyplot as plt
import numpy as np
import math as m


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

# Andrew Kensler/Roman Wiche varaint of the ray-AABB slab method for 
# detectiing collison. Returns true if ray intersects axis-aligned box.
def ray_AABB(r_dir, r_pos, b_pos, b_pos2):
    invD = vreciprocal(r_dir)
    t0s = (b_pos - r_pos) * invD
    t1s = (b_pos2 - r_pos) * invD
    tsmaller = vmin(t0s, t1s)
    tbigger  = vmax(t0s, t1s)
    tmin = 0
    tmax = 2147483647
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


# Construct 4x4 transformation matrix given position and direction vectors but
# witout scaling.
def matrix_4x4(p, d):
    T = np.array([[1,0,0,p[0]],
                  [0,1,0,p[1]],
                  [0,0,1,p[2]],
                  [0,0,0,   1]])
    d = normalize(d)
    r = axis_align(d)
    R = np.array([[r[0][0],r[0][1],r[0][2],0],
                  [r[1][0],r[1][1],r[1][2],0],
                  [r[2][0],r[2][1],r[2][2],0],
                  [      0,      0,      0,1]])
    S = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,1,0],
                  [0,0,0,1]])
    M = np.matmul(np.matmul(T, R), S)
    return M


ship_half_l = 500 / 2
ship_half_h_w= 100 / 2


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


def test():
    B1 = np.array([3,0,1]); # ship position
    B2 = np.array([4,0,3]);
    B3 = np.array([2,0,4]);
    B4 = np.array([1,0,2]);

    v = np.array([1,0,2]) # ship dir
    r = axis_align(v)

    B1 = np.matmul(r, np.transpose(B1))
    B2 = np.matmul(r, np.transpose(B2))
    B3 = np.matmul(r, np.transpose(B3))
    B4 = np.matmul(r, np.transpose(B4))

    print(B1, B2, B3, B4)
    print()

    m = r

    p = np.array([0,0,-2])
    p2 = np.array([5,0,8])
    d = np.array([1,0,2])

    p = np.matmul(m, np.transpose(p))
    p2 = np.matmul(m, np.transpose(p2))
    d = np.matmul(m, np.transpose(d))

    x = (B1[0],B2[0],B3[0],B4[0], p[0], p2[0])
    y = (B1[2],B2[2],B3[2],B4[2], p[2], p2[2])
    plt.scatter(x ,y)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(B1[0], B1[1], B1[2])
    ax.scatter(B2[0], B2[1], B2[2])
    ax.scatter(B3[0], B3[1], B3[2])
    ax.scatter(B4[0], B4[1], B4[2])

    ax.scatter(p[0], p[1], p[2])
    ax.scatter(p2[0], p2[1], p2[2])

    print(p)
    print(p2)
    print(d)


def test0():
    B1 = np.array([7,0,-1]); # ship position
    B2 = np.array([9,0,3]);
    B3 = np.array([3,0,6]);
    B4 = np.array([1,0,2]);

    v = np.array([2,0,-1]) # ship dir
    #r = matrix_4x4( np.array([0,0,0]), v)
    r = axis_align(v)

    B1 = np.matmul(r, np.transpose(B1))
    B2 = np.matmul(r, np.transpose(B2))
    B3 = np.matmul(r, np.transpose(B3))
    B4 = np.matmul(r, np.transpose(B4))

    print(B1, B2, B3, B4)
    print()

    m = r
    p = np.array([0,0,2])
    p2 = np.array([5,0,7])
    d = np.array([1,0,1])
    
    p = np.matmul(m, np.transpose(p))
    p2 = np.matmul(m, np.transpose(p2))
    d = np.matmul(m, np.transpose(d))

    x = (B1[0],B2[0],B3[0],B4[0], p[0], p2[0])
    y = (B1[2],B2[2],B3[2],B4[2], p[2], p2[2])
    plt.scatter(x ,y)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(B1[0], B1[1], B1[2])
    ax.scatter(B2[0], B2[1], B2[2])
    ax.scatter(B3[0], B3[1], B3[2])
    ax.scatter(B4[0], B4[1], B4[2])

    ax.scatter(p[0], p[1], p[2])
    ax.scatter(p2[0], p2[1], p2[2])

    print(p)
    print(p2)
    print(d)


def test1():
     r_dir = np.array([1,0,0])
     r_pos = np.array([0.9,0.9,0.9])
     b_pos = np.array([1,1,1])
     b_pos2 = np.array([10,10,10])
     print(ray_AABB( r_dir, r_pos, b_pos, b_pos2))

     r_dir = np.array([1,0,0])
     r_pos = np.array([1,3,3])
     b_pos = np.array([1,1,1])
     b_pos2 = np.array([10,10,10])
     print(ray_AABB( r_dir, r_pos, b_pos, b_pos2))

     r_dir = np.array([1,0,0])
     r_pos = np.array([1,10,10])
     b_pos = np.array([1,1,1])
     b_pos2 = np.array([10,10,10])
     print(ray_AABB( r_dir, r_pos, b_pos, b_pos2))

     r_dir = np.array([-1,-1,-1])
     r_pos = np.array([-1,-1,-1])
     b_pos = np.array([-100,-101,-101])
     b_pos2 = np.array([-101,-100,-100])
     print(ray_AABB( r_dir, r_pos, b_pos, b_pos2))


def test2():
    redcur = np.array([1, 7, 7, 7, 0, 0, 0, 0, 0, 0])
    redatk = np.array([1, 10183, 10000, 9819])
    blucur = np.array([1, 7, 7, 7, 10000, 10000, 10000, -176.776695297, 50, 176.776695297])
    
    atk_dir = np.array([redatk[1], redatk[2], redatk[3]]) 
    red_pos = np.array([redcur[4], redcur[5], redcur[6]])
    
    
    blu_pos = np.array([blucur[4], blucur[5], blucur[6]])
    blue_dir = np.array([blucur[7], blucur[8], blucur[9]])
    
    r = axis_align(blue_dir)
    
    r_dir = np.matmul(r, np.transpose(atk_dir))
    r_pos = np.matmul(r, np.transpose(red_pos))
    
    blu_pos = np.matmul(r, np.transpose(blu_pos))
    b_pos = translate_ship_min(np.array([blu_pos[0], blu_pos[1], blu_pos[2]]))
    b_pos2 = translate_ship_max(np.array([blu_pos[0], blu_pos[1], blu_pos[2]]))
    
    
    print(ray_AABB( r_dir, r_pos, b_pos, b_pos2))

def test3(x, y):
    redcur = np.array([1, 7, 7, 7, x, y, -1, 0, 0, 0])
    redatk = np.array([1, 0, 0, 1])
    blucur = np.array([1, 7, 7, 7, 201.246117975, 156.524758425, 50, 2, 1, 0])
    
    atk_dir = np.array([redatk[1], redatk[2], redatk[3]]) 
    red_pos = np.array([redcur[4], redcur[5], redcur[6]])
    
    
    blu_pos = np.array([blucur[4], blucur[5], blucur[6]])
    blue_dir = np.array([blucur[7], blucur[8], blucur[9]])
    
    r = axis_align(blue_dir)
    
    r_dir = np.matmul(r, np.transpose(atk_dir))
    r_pos = np.matmul(r, np.transpose(red_pos))
    
    blu_pos = np.matmul(r, np.transpose(blu_pos))
    b_pos = translate_ship_min(np.array([blu_pos[0], blu_pos[1], blu_pos[2]]))
    b_pos2 = translate_ship_max(np.array([blu_pos[0], blu_pos[1], blu_pos[2]]))
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(blu_pos[0], blu_pos[1], blu_pos[2])
    ax.scatter(b_pos[0], b_pos[1], b_pos[2])
    ax.scatter(b_pos2[0], b_pos2[1], b_pos2[2])
    
    print(blu_pos)
    print(b_pos)
    print(b_pos2)
    '''
    return ray_AABB( r_dir, r_pos, b_pos, b_pos2)

def test4():
    x = 300
    y = 300
    xl = []
    yl = []
    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')
    for i in range(x):
        for j in range(y):
            if test3(i*10-200, j*10-200):
                #ax.scatter(i*10-200, j*10-200, 0)
                xl.append(i*10-200)
                yl.append(j*10-200)

    plt.scatter(xl, yl)
    #plt.show()
    #test3()
    #test0()

def test5():
    # triangle(h, a, 2a) 100, 44.72135955, 89.4427191
    # triangle(h, a, 2a) 500, 223.60679775, 447.2135955
    # center 201.246117975, 156.524758425, 50
    
    redcur = np.array([1, 7, 7, 7, -10, 1, 99, 0, 0, 0])
    redatk = np.array([1, 10, 0, 1])
    blucur = np.array([1, 7, 7, 7, 201.246117975, 156.524758425, 50, 2, 1, 0])
    
    atk_dir = np.array([redatk[1], redatk[2], redatk[3]]) 
    red_pos = np.array([redcur[4], redcur[5], redcur[6]])
    
    blu_pos = np.array([blucur[4], blucur[5], blucur[6]])
    blue_dir = np.array([blucur[7], blucur[8], blucur[9]])
    
    r = axis_align(blue_dir)
    
    r_dir = np.matmul(r, np.transpose(atk_dir))
    r_pos = np.matmul(r, np.transpose(red_pos))
    
    blu_pos = np.matmul(r, np.transpose(blu_pos))
    b_pos = translate_ship_min(np.array([blu_pos[0], blu_pos[1], blu_pos[2]]))
    b_pos2 = translate_ship_max(np.array([blu_pos[0], blu_pos[1], blu_pos[2]]))

    print(ray_AABB( r_dir, r_pos, b_pos, b_pos2))
