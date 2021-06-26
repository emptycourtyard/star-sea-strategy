# 6/11/21
import matplotlib.pyplot as plt
import numpy as np
import math as m

#
# hitbox detection attempt 1
#



def Rxyz_helper(V):
  x = V[0]
  y = V[1]
  z = V[2]
  # theta relative to axis
  ax = m.atan2(m.sqrt(y**2+z**2), x);
  ay = m.atan2(m.sqrt(z**2+x**2), y);
  az = m.atan2(m.sqrt(x**2+y**2), z);
  return np.array([ax,ay,az])

def Rxyz(V):
  x = V[0]
  y = V[1]
  z = V[2]
  if x == 0:
    x += 1
  x_hat = Rxyz_helper(np.array([0,y,z]))[0]
  
  y_hat = Rxyz_helper(np.array([x,0,z]))[0]
  if z > 0:
    y_hat = -y_hat
  z_hat = Rxyz_helper(np.array([x,y,0]))[0]
  if y < 0:
    z_hat = -z_hat
  if x < 0:
    z_hat = -m.fabs(z_hat-m.pi)
  return np.array([x_hat,y_hat,z_hat])


def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])


def rotate_matrix(v):
  phi = v[0]
  theta = v[1]
  psi = v[2]
  return Rz(psi) * Ry(theta) * Rx(phi)




def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = m.sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]

def axisangle_to_q(v, theta):
    v = normalize(v)
    x, y, z = v
    theta /= 2
    w = m.cos(theta)
    x = x * m.sin(theta)
    y = y * m.sin(theta)
    z = z * m.sin(theta)
    return w, x, y, z

#r_x = axisangle_to_q((1, 0, 0), m.pi/2)
#r_y = axisangle_to_q((0, 1, 0), m.pi/2)
#r_z = axisangle_to_q((0, 0, 1), m.pi/2)

#v = qv_mult(r_x, (1,2,1))
#v = qv_mult(r_y, v)
#v = qv_mult(r_z, v)

#right = (1,2,3)
#forward = np.cross(right, np.array([0,1,0]))
#up = qv_mult(axisangle_to_q(forward, m.pi/2), right)


def rotation_matrix(v):
    #/* Find cosφ and sinφ */
    c1 = m.sqrt(v[0]**2 + v[1]**2);
    s1 = v[2];
    #/* Find cosθ and sinθ; if gimbal lock, choose (1,0) arbitrarily */
    #float c2 = c1 ? v1.x / c1 : 1.0;
    #float s2 = c1 ? v1.y / c1 : 0.0;
    
    if c1 != 0: 
      c2 = v[0] / c1
    else:
      c2 = 1.0

    if c1 != 0:
      s2 = v[1] / c1
    else:
      s2 = 0.0

    return np.array([[v[0], -s2, -s1*c2,  0],
                     [v[1],  c2, -s1*s2,  0],
                     [v[2],   0,     c1,  0],
                     [   0,   0,      0,  1]])


def matrix4x4(x, y, z, u, v, w):
    
    T = np.array([[1,0,0,x],
                  [0,1,0,y],
                  [0,0,1,z],
                  [0,0,0,1]])
    
    
    uvw_norm = normalize((u,v,w))
    u = uvw_norm[0]
    v = uvw_norm[1]
    w = uvw_norm[2]
    
    '''
    # right(front): accelration vector(u, v, w)
    # forward: perpendicular to r and paralell to y axis
    # up: right rotated 90 degrees from axis forward
    r = (u,v,w)
    f = np.cross(r, np.array([0,1,0]))
    u = qv_mult(axisangle_to_q(f, m.pi/2), r)
    
    R = np.array([[r[0],u[0],f[0],0],
                  [r[1],u[1],f[1],0],
                  [r[2],u[2],f[2],0],
                  [0,0,0,1]])
    '''
    
    R = rotation_matrix(np.array([u,v,w]))
    
    xyz_norm = normalize((x,y,z))
    x = xyz_norm[0]
    y = xyz_norm[1]
    z = xyz_norm[2]
    S = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,1,0],
                  [0,0,0,1]])
    
    M = T * R * S
    return M

# Does the Line (L1, L2) intersect the Box?
def is_line_in_box( m_M, m_Extent, L1, L2 ):

  # Put line in box space
  MInv = np.linalg.inv(m_M)
  LB1 = np.matmul(MInv, L1)
  LB2 = np.matmul(MInv, L2)
  # Get line midpoint and extent
  LMid = (LB1+LB2)*0.5
  L = (LB1-LMid)
  LExt = np.array([ abs(L[0]), abs(L[1]), abs(L[2]) ])
  
  # Use Separating Axis Test
  # Separation vector from box center to line center is LMid, since the line is in box space
  if ( abs( LMid[0] ) > m_Extent[0] + LExt[0] ): return False;
  if ( abs( LMid[1] ) > m_Extent[1] + LExt[1] ): return False;
  if ( abs( LMid[2] ) > m_Extent[2] + LExt[2] ): return False;
  # Crossproducts of line and each axis
    
  if ( abs( LMid[1] * L[2] - LMid[2] * L[1])  >  (m_Extent[1] * LExt[2] + m_Extent[2] * LExt[1]) ): return False;
  if ( abs( LMid[0] * L[2] - LMid[2] * L[0])  >  (m_Extent[0] * LExt[2] + m_Extent[2] * LExt[0]) ): return False;
  if ( abs( LMid[0] * L[1] - LMid[1] * L[0])  >  (m_Extent[0] * LExt[1] + m_Extent[1] * LExt[0]) ): return False;
  # No separating axis, the line intersects
  return True;


def AABB_LineSegmentOverlap( m_M, m_Extent, L1, L2 ):

  MInv = np.linalg.inv(m_M)
  LB1 = np.matmul(MInv, L1)
  LB2 = np.matmul(MInv, L2)
  # Get line midpoint and extent
  LMid = (LB1+LB2)*0.5
  L = (LB1-LMid)    

  T = LMid
  E = m_Extent
  #hl*l = LExt
  l = L

  '''
  VECTOR& l, #line direction
  VECTOR& mid, #midpoint of the line
  # segment
  SCALAR hl, #segment half-length
  AABB& b #box

  VECTOR T = b.P - mid;
  '''

  #do any of the principal axes
  #form a separating axis?
  if( abs(T[0]) > E[0] + abs(l[0]) ): return False;
  if( abs(T[1]) > E[1] + abs(l[1]) ): return False;
  if( abs(T[2]) > E[2] + abs(l[2]) ): return False;
  '''
  if( abs(T[0]) > E[0] + hl*abs(l[0]) ): return False;
  if( abs(T[1]) > E[1] + hl*abs(l[1]) ): return False;
  if( abs(T[2]) > E[2] + hl*abs(l[2]) ): return False;
  '''
  # > scalar r
  if( abs(T[1]*l[2] - T[2]*l[1]) > E[1]*abs(l[2]) + E[2]*abs(l[1]) ): return False;
  if( abs(T[2]*l[0] - T[0]*l[2]) > E[0]*abs(l[2]) + E[2]*abs(l[0]) ): return False;
  if( abs(T[0]*l[1] - T[1]*l[0]) > E[0]*abs(l[1]) + E[1]*abs(l[0]) ): return False;

  return True;

'''

def GetIntersection( float fDst1, float fDst2, CVec3 P1, CVec3 P2, CVec3 &Hit):
  if ( (fDst1 * fDst2) >= 0.0f) return 0;
  if ( fDst1 == fDst2) return 0; 
  Hit = P1 + (P2-P1) * ( -fDst1/(fDst2-fDst1) );
  return 1;


def InBox( CVec3 Hit, CVec3 B1, CVec3 B2, const int Axis):
  if ( Axis==1 && Hit.z > B1.z && Hit.z < B2.z && Hit.y > B1.y && Hit.y < B2.y) return 1;
  if ( Axis==2 && Hit.z > B1.z && Hit.z < B2.z && Hit.x > B1.x && Hit.x < B2.x) return 1;
  if ( Axis==3 && Hit.x > B1.x && Hit.x < B2.x && Hit.y > B1.y && Hit.y < B2.y) return 1;
  return 0;

#// returns true if line (L1, L2) intersects with the box (B1, B2)
#// returns intersection point in Hit
def CheckLineBox( CVec3 B1, CVec3 B2, CVec3 L1, CVec3 L2, CVec3 &Hit):
  if (L2.x < B1.x && L1.x < B1.x) return false;
  if (L2.x > B2.x && L1.x > B2.x) return false;
  if (L2.y < B1.y && L1.y < B1.y) return false;
  if (L2.y > B2.y && L1.y > B2.y) return false;
  if (L2.z < B1.z && L1.z < B1.z) return false;
  if (L2.z > B2.z && L1.z > B2.z) return false;
  if (L1.x > B1.x && L1.x < B2.x &&
      L1.y > B1.y && L1.y < B2.y &&
      L1.z > B1.z && L1.z < B2.z) 
      {Hit = L1; 
      return true;}
  if ( (GetIntersection( L1.x-B1.x, L2.x-B1.x, L1, L2, Hit) && InBox( Hit, B1, B2, 1 ))
    || (GetIntersection( L1.y-B1.y, L2.y-B1.y, L1, L2, Hit) && InBox( Hit, B1, B2, 2 )) 
    || (GetIntersection( L1.z-B1.z, L2.z-B1.z, L1, L2, Hit) && InBox( Hit, B1, B2, 3 )) 
    || (GetIntersection( L1.x-B2.x, L2.x-B2.x, L1, L2, Hit) && InBox( Hit, B1, B2, 1 )) 
    || (GetIntersection( L1.y-B2.y, L2.y-B2.y, L1, L2, Hit) && InBox( Hit, B1, B2, 2 )) 
    || (GetIntersection( L1.z-B2.z, L2.z-B2.z, L1, L2, Hit) && InBox( Hit, B1, B2, 3 )))
    return true;

  return false;
'''

def Translate(m, Test):
  m[0][3] += Test[0] * m[0][0] + Test[1] * m[0][1] + Test[2] * m[0][2]; 
  m[1][3] += Test[0] * m[1][0] + Test[1] * m[1][1] + Test[2] * m[1][2];
  m[2][3] += Test[0] * m[2][0] + Test[1] * m[2][1] + Test[2] * m[2][2];
  m[3][3] += Test[0] * m[3][0] + Test[1] * m[3][1] + Test[2] * m[3][2];
  return(m)


# TODO
# 
# tests not accurate
#
def tests():
    
  BL = np.array([  0,  0,  0, 1])
  BH = np.array([500,100,100, 1])
  
  m_M = matrix4x4(1200, 1500, 1100, 1100,200,400)

  t = (BH + BL) * 0.5
  m_M = Translate(m_M, t)
  
  '''
  #translate m by t
  m_M = np.matmul(np.array([[1,0,0,t[0]],
                            [0,1,0,t[1]],
                            [0,0,1,t[2]],
                            [0,0,0,1]]), m_M)
  '''

  m_Extent = (BH - BL) / 2.0;

  x = 50
  y = 50
  z = 50
  
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  
  for i in range(x):
      for j in range(y):
          for k in range(z):
              #print(i,j)
              if is_line_in_box( m_M, m_Extent, np.array([i*100,j*100,k,0]), np.array([(i*100)+100,(j*100),k,1])): 
              #if AABB_LineSegmentOverlap( m_M, m_Extent, np.array([i*100,j*100,k,0]), np.array([(i*100)+100,(j*100),k,1])): 
                  ax.scatter(i,j, 0)
                  ax.scatter(-10,-10, k)
                  ax.scatter(-10,50, k)
                  ax.scatter(50,-10, k)
                  ax.scatter(50,50, k)
                  print(i,j,k)
                
  plt.show()

tests()














