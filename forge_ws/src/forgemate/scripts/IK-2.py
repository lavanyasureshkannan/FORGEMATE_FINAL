
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
#%matplotlib inline
import pylab as pl
import numpy as np
import math
from IPython import display
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') 
theta3=0
p =10
d1,d3,d5,d7 = 40,42,39.95,10.55
t01 = Matrix([[cos(theta1),   0     , -sin(theta1),          0], 
            [sin(theta1),     0,        cos(theta1),             0], 
            [0,              -1,           0,               d1], 
            [0,               0,           0,                1]])
t12 = Matrix([[cos(theta2),   0,         sin(theta2),            0], 
            [sin(theta2),     0,          -cos(theta2),          0], 
            [0,               1,           0,                0], 
            [0,               0,           0,                1]])


t23 = Matrix([[cos(theta3),   0,         sin(theta3),         0],
             [sin(theta3),    0 ,        -cos(theta3),        0],
             [0,              1,            0,               d3],
             [0,              0,            0,                1]])

t34 = Matrix([[cos(theta4),   0,          -sin(theta4),       0],
             [sin(theta4),    0,           cos(theta4),       0], 
             [0,              -1,            0,               0],
             [0,              0,             0,               1]])

t45 = Matrix([[cos(theta5),   0,          -sin(theta5),        0],
             [sin(theta5),    0,           cos(theta5),        0],
             [0,              -1,            0,                d5],
             [0,              0,             0,                1]])

t56 = Matrix([[cos(theta6),   0,           sin(theta6),         0],
             [sin(theta6),        0,          -cos(theta6),         0],
             [0,              1,             0,                 0],
             [0,              0,             0,                1]])


t67 = Matrix([[cos(theta7),  -sin(theta7),   0,                 0],
             [sin(theta7),   cos(theta7),    0,                 0],
             [0,              0,             1,              d7+p],
             [0,              0,             0,                1]])
t07 = (t01 * t12 * t23 * t34 * t45 * t56 * t67)
t02 = t01 * t12
t03 = t02 * t23
t04 = t03 * t34
t05 = t04 * t45
t06 = t05 * t56
x = t07[0,3]
y = t07[1,3]
z = t07[2,3]
z0 = Matrix([[0],[0],[1]])
z1 = t01[0:3,2]
z2 = t02[0:3,2]
z3 = t03[0:3,2]
z4 = t04[0:3,2]
z5 = t05[0:3,2]
z6 = t06[0:3,2]
Jacobian = Matrix( [[diff(x, theta1), diff(x,theta2), diff(x,theta4), diff(x,theta5), diff(x,theta6), diff(x,theta7)],
            [diff(y,theta1),  diff(y,theta2), diff(y,theta4), diff(y,theta5), diff(y,theta6), diff(y,theta7)],
            [diff(z,theta1),  diff(z,theta2), diff(z,theta4), diff(z,theta5), diff(z,theta6), diff(z,theta7)], 
            [z0,              z1,              z2,              z4,             z5,                      z6]] )
print(Jacobian)
def radian(x):
    return math.radians(x)

pi = 3.14
radius = 10
time = 40
#angular velocity = (2* pi)/ time
angular_vel = (2*pi) / time


    
def draw_joint(axes,Q):
    
    
    t01_new = t01.subs({theta1: Q[0], theta2: Q[1], theta3: q3, theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})
    t02_new = t02.subs({theta1: Q[0], theta2: Q[1], theta3: q3, theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})
    t03_new = t03.subs({theta1: Q[0], theta2: Q[1], theta3: q3, theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})
    t04_new = t04.subs({theta1: Q[0], theta2: Q[1], theta3: q3, theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})
    t05_new = t05.subs({theta1: Q[0], theta2: Q[1], theta3: q3, theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})
    t06_new = t06.subs({theta1: Q[0], theta2: Q[1], theta3: q3, theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})
    t07_new = t07.subs({theta1: Q[0], theta2: Q[1], theta3: q3, theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})
    axes.plot3D( [0,           t01_new[0,3] ],  [0,t01_new[1,3]],        [0,t01_new[2,3]],                   'blue')
    axes.plot3D( [t01_new[0,3], t02_new[0,3] ], [ t01_new[1,3], t02_new[1,3] ], [t01_new[2,3] ,t02_new[2,3]],'blue')
    axes.plot3D( [t02_new[0,3], t03_new[0,3] ], [ t02_new[1,3], t03_new[1,3] ], [t02_new[2,3] ,t03_new[2,3]],'blue')
    axes.plot3D( [t03_new[0,3], t04_new[0,3] ], [ t03_new[1,3], t04_new[1,3] ], [t03_new[2,3] ,t04_new[2,3]],'blue')
    axes.plot3D( [t04_new[0,3], t05_new[0,3] ], [ t04_new[1,3], t05_new[1,3] ], [t04_new[2,3] ,t05_new[2,3]],'blue')
    axes.plot3D( [t05_new[0,3], t06_new[0,3] ], [ t05_new[1,3], t06_new[1,3] ], [t05_new[2,3] ,t06_new[2,3]],'blue')
    axes.plot3D( [t06_new[0,3], t07_new[0,3] ], [ t06_new[1,3], t07_new[1,3] ], [t06_new[2,3] ,t07_new[2,3]],'red')



#considering the eqn of the circle 
def draw_circle(xcir, ycir, zcir, rad, s):             
    xx = []
    zz= []
    tt= np.linspace(2 * pi, 0, s)
    for i in tt:
        #x = r * cos(alpha)
        xx.append(rad * sin(i) + xcir)           
        zz.append(rad * cos(i) + zcir)           
    yy = np.ones(s) * ycir
    return xx, yy, zz

xcircle,ycircle,zcircle = draw_circle(0, 60, 68, 10, 10)
xcircle = np.array(xcircle,dtype = float)
ycircle =np.array(ycircle,dtype = float)
zcircle = np.array(zcircle,dtype = float)

alpha = np.pi/2

x_final = []
y_final = []
z_final = []


def cmp(k, mink, maxk):
    return max(min(maxk, k), mink)

q3=radian(-90)

Q = Matrix([ [radian(90.0001)], [radian(0.00001)], [radian(-90.0001)], [radian(0.00001)], [radian(0.00001)],  [radian(0.0001)]])

final_J = Jacobian.subs({theta1: Q[0], theta2: Q[1], theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})

Q_curr = Matrix([ [radian(0)], [radian(0)], [radian(0)], [radian(0)], [radian(0)], [radian(0)]])

limits = False
count = 0
aa = 2*np.pi + np.pi/2

while(alpha < aa):
    final_J = Jacobian.subs({theta1: Q[0], theta2: Q[1], theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5]})
    try:
        Jacobian_pseudo = final_J.inv()
    except:
        Q = (Q + Q_curr).evalf()
        continue
    
#Velocity of the end effector is given by = [vx, vy, vz, roll, pitch, yaw] 
    vx = -radius *sin(alpha)*angular_vel
    vy = 0
    vz = radius *cos(alpha)*angular_vel
    roll= 0
    pitch = 0
    yaw =0
    alpha = alpha+ angular_vel

    Velocity = Matrix([ [vx], [vy], [vz], [roll], [pitch], [yaw]])
    Q_curr=Jacobian_pseudo*Velocity
    
    u=0
    Q_L =len(Q_curr)
    
    while(u<Q_L and limits):
        Q_curr[i] = cmp(Q_curr[i],-1,1)
        u +=1

    Q= (Q + Q_curr).evalf()
    
    #inverse kinematics
    T = t07.subs({theta1: Q[0], theta2: Q[1], theta3: q3, theta4: Q[2], theta5: Q[3], theta6: Q[4], theta7: Q[5] })
    x = T[0,3].evalf()
    x_final.append(float(x))
    y = T[1,3].evalf()
    y_final.append(float(y))
    z = T[2,3].evalf()
    z_final.append(float(z))

      
    #visualization
    
    display.clear_output(wait=True)
    display.display(plt.gcf())
    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax = fig.add_subplot(1, 2, 1)
    ax.set_xlabel('xaxis')
    ax.set_ylabel('yaxis')
    ax.axis('equal')
    ax.scatter(x_final,z_final)

    ax3 = fig.add_subplot(1, 2, 2, projection='3d')
    ax3.cla()
    ax3.scatter3D(x_final, y_final, z_final,cmap='red')
    ax3.scatter3D(xcircle, ycircle, zcircle, 'r')
    draw_joint(ax3,Q)
    ax3.set_xlabel('xaxis')
    ax3.set_ylabel('yaxis')
    ax3.set_zlabel('zaxis')
    ax3.set_title('robot arm drawing on the wall in a 3Dview')

    count +=1 

plt.show()