import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from spatialmath import SE3
# link lenths in mm
a1 = float(input("a1 = "))
a2 = float(input("a2 = ")) 
a3 = float(input("a3 = ")) 
a4 = float(input("a4 = ")) 
a5 = float(input("a5 = ")) 

# link mm to meters converter
def mm_to_meter(a):
    m = 1000 # 1 meter = 1000 mm
    return a/m

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2)
a3 = mm_to_meter(a3)
a4 = mm_to_meter(a4)
a5 = mm_to_meter(a5)

# link limits converted to meters
lm3 = float(input("lm3 = ")) 
lm3 = mm_to_meter(lm3)

# Create Links
SCARA = DHRobot([
    RevoluteDH(a1,0,(0/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    PrismaticDH(0,a2,0,0,qlim=[0,0]),
    RevoluteDH(a3,0,(180/180)*np.pi,0,qlim=[(-90/180)*np.pi,(90/180)*np.pi]),
    PrismaticDH(0,a4,0,0,qlim=[0,0]),
    PrismaticDH(0,0,0,a5,qlim=[0,lm3])
], name='SCARA')

print(SCARA)

# degrees to radian converter
def deg_to_rad(T):
    return (T/180.0)*np.pi


# q Paths
#for SCARA Standard Joint Variables =([ T1, 0, T2, 0, d3])
q0 = np.array([0,0,0,0,0])
q1 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #1st path
                ])

q2 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #2nd path
                ])

q3 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])    

q4 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])  

q5 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])  

q6 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])  

q7 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])  

q8 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])  

q9 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])   

q10 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])  

q11 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])  

q12 = np.array([ deg_to_rad(float(input("T1 = "))),0,
                deg_to_rad(float(input("T2 = "))),0,
                mm_to_meter(float(input("d3 = "))) #3rd path
                ])             

#plot scale
x1 = -0.25
x2 = 0.25
y1 = -0.25
y2 = 0.25
z1 = -0.25
z2 = 0.25


# Trajectory commands
traj1 = rtb.jtraj(q0,q1,5)
traj2 = rtb.jtraj(q1,q2,5)
traj3 = rtb.jtraj(q2,q3,5)
traj4 = rtb.jtraj(q3,q4,5)
traj5 = rtb.jtraj(q4,q5,5)
traj6 = rtb.jtraj(q5,q6,5)
traj7 = rtb.jtraj(q6,q7,5)
traj8 = rtb.jtraj(q7,q8,5)
traj9 = rtb.jtraj(q8,q9,5)
traj10 = rtb.jtraj(q9,q10,5)
traj11 = rtb.jtraj(q10,q11,5)
traj12 = rtb.jtraj(q11,q12,5)


# for Joint Variable vs Time(s) table
#rtb.qplot(traj1.q)
#rtb.qplot(traj2.q)
#rtb.qplot(traj3.q)
#rtb.qplot(traj4.q)

SCARA.plot(traj1.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj2.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj4.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj5.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj6.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj7.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj8.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj9.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj10.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj11.q,limits = [x1, x2, y1, y2, z1, z2])
SCARA.plot(traj12.q,limits = [x1, x2, y1, y2, z1, z2],block= True)
#SCARA.teach(jointlabels=1)

