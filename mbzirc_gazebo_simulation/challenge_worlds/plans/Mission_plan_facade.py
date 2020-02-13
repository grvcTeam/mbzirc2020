#!/usr/bin/env python

from os import remove 
import numpy as np
import array 
import roslaunch
import math as m
import tf.transformations 

# origin = open( 'Data.txt', 'r' )

aux_angle=1.57

b=[20,23,0,0]
g=[25,30,0,0]

f4ap=[b[0]+0-2, b[1]+7.5, b[2]+5.3, -1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f4ap[3]+b[3])
# f4a =[(f4ap[0]-b[0])*m.cos(b[3])-(f4ap[1]-b[1])*m.sin(b[3])+b[0],(f4ap[0]-b[0])*m.sin(b[3])+(f4ap[1]-b[1])*m.cos(b[3])+b[1],f4ap[2],0,0,f4ap[3]+b[3]]
f4a =[(f4ap[0]-b[0])*m.cos(b[3])-(f4ap[1]-b[1])*m.sin(b[3])+b[0],(f4ap[0]-b[0])*m.sin(b[3])+(f4ap[1]-b[1])*m.cos(b[3])+b[1],f4ap[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]


f4bp = [b[0]+10+2, b[1]+6, b[2]+5.3, 1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f4bp[3]+b[3])
f4b = [(f4bp[0]-b[0])*m.cos(b[3])-(f4bp[1]-b[1])*m.sin(b[3])+b[0],(f4bp[0]-b[0])*m.sin(b[3])+(f4bp[1]-b[1])*m.cos(b[3])+b[1],f4bp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f4cp = [b[0]+4.5, b[1]+0-2, b[2]+5.3, 0+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f4cp[3]+b[3])
f4c = [(f4cp[0]-b[0])*m.cos(b[3])-(f4cp[1]-b[1])*m.sin(b[3])+b[0],(f4cp[0]-b[0])*m.sin(b[3])+(f4cp[1]-b[1])*m.cos(b[3])+b[1],f4cp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

#
f4dp=[b[0]+4.7, b[1]+10+2, b[2]+5.5, -3.1416+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f4dp[3]+b[3])
f4d=[(f4dp[0]-b[0])*m.cos(b[3])-(f4dp[1]-b[1])*m.sin(b[3])+b[0],(f4dp[0]-b[0])*m.sin(b[3])+(f4dp[1]-b[1])*m.cos(b[3])+b[1],f4dp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f4ep=[b[0]+0, b[1]+11.3, b[2]+2, -1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f4ep[3]+b[3])
f4e=[(f4ep[0]-b[0])*m.cos(b[3])-(f4ep[1]-b[1])*m.sin(b[3])+b[0],(f4ep[0]-b[0])*m.sin(b[3])+(f4ep[1]-b[1])*m.cos(b[3])+b[1],f4ep[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f5ap=[b[0]+0-2, b[1]+5, b[2]+7.5, -1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f5ap[3]+b[3])
f5a=[(f5ap[0]-b[0])*m.cos(b[3])-(f5ap[1]-b[1])*m.sin(b[3])+b[0],(f5ap[0]-b[0])*m.sin(b[3])+(f5ap[1]-b[1])*m.cos(b[3])+b[1],f5ap[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f5bp=[b[0]+10+2, b[1]+8.7, b[2]+8.25, 1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f5bp[3]+b[3])
f5b=[(f5bp[0]-b[0])*m.cos(b[3])-(f5bp[1]-b[1])*m.sin(b[3])+b[0],(f5bp[0]-b[0])*m.sin(b[3])+(f5bp[1]-b[1])*m.cos(b[3])+b[1],f5bp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f5cp=[b[0]+2, b[1]+0-2, b[2]+9.2, 0+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f5cp[3]+b[3])
f5c=[(f5cp[0]-b[0])*m.cos(b[3])-(f5cp[1]-b[1])*m.sin(b[3])+b[0],(f5cp[0]-b[0])*m.sin(b[3])+(f5cp[1]-b[1])*m.cos(b[3])+b[1],f5cp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]


#
f5dp=[b[0]+1.5, b[1]+10+2, b[2]+9.2, 3.1416+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f5dp[3]+b[3])
f5d=[(f5dp[0]-b[0])*m.cos(b[3])-(f5dp[1]-b[1])*m.sin(b[3])+b[0],(f5dp[0]-b[0])*m.sin(b[3])+(f5dp[1]-b[1])*m.cos(b[3])+b[1],f5dp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]


f6ap=[b[0]+0-2, b[1]+6.5, b[2]+10.3, -1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f6ap[3]+b[3])
f6a=[(f6ap[0]-b[0])*m.cos(b[3])-(f6ap[1]-b[1])*m.sin(b[3])+b[0],(f6ap[0]-b[0])*m.sin(b[3])+(f6ap[1]-b[1])*m.cos(b[3])+b[1],f6ap[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f6bp=[b[0]+10+2, b[1]+1.3, b[2]+11.25, 1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f6bp[3]+b[3])
f6b=[(f6bp[0]-b[0])*m.cos(b[3])-(f6bp[1]-b[1])*m.sin(b[3])+b[0],(f6bp[0]-b[0])*m.sin(b[3])+(f6bp[1]-b[1])*m.cos(b[3])+b[1],f6bp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f6cp=[b[0]+8.7, b[1]+0-2, b[2]+12, 0+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f6cp[3]+b[3])
f6c=[(f6cp[0]-b[0])*m.cos(b[3])-(f6cp[1]-b[1])*m.sin(b[3])+b[0],(f6cp[0]-b[0])*m.sin(b[3])+(f6cp[1]-b[1])*m.cos(b[3])+b[1],f6cp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f6dp=[b[0]+8.7, b[1]+10+2, b[2]+12.5, 3.1416+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, f6dp[3]+b[3])
f6d=[(f6dp[0]-b[0])*m.cos(b[3])-(f6dp[1]-b[1])*m.sin(b[3])+b[0],(f6dp[0]-b[0])*m.sin(b[3])+(f6dp[1]-b[1])*m.cos(b[3])+b[1],f6dp[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

f7=[30,10,0,0]
f8=[10,40,0,-1.570795]

# Facade Switch Points drone 1

sw34p=[b[0]+14, b[1]-4, b[2]+10, 1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, sw34p[3]+b[3])
sw34=[(sw34p[0]-b[0])*m.cos(b[3])-(sw34p[1]-b[1])*m.sin(b[3])+b[0],(sw34p[0]-b[0])*m.sin(b[3])+(sw34p[1]-b[1])*m.cos(b[3])+b[1],sw34p[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

sw41p=[b[0]-4, b[1]-4, b[2]+10,0+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, sw41p[3]+b[3])
sw41=[(sw41p[0]-b[0])*m.cos(b[3])-(sw41p[1]-b[1])*m.sin(b[3])+b[0],(sw41p[0]-b[0])*m.sin(b[3])+(sw41p[1]-b[1])*m.cos(b[3])+b[1],sw41p[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

# Facade Switch Points drone 2

sw12p=[b[0]-4, b[1]+14, b[2]+10,-1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, sw12p[3]+b[3])
sw12=[(sw12p[0]-b[0])*m.cos(b[3])-(sw12p[1]-b[1])*m.sin(b[3])+b[0],(sw12p[0]-b[0])*m.sin(b[3])+(sw12p[1]-b[1])*m.cos(b[3])+b[1],sw12p[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

sw23p=[b[0]+14, b[1]+14, b[2]+10,1.57+aux_angle]
quaternion = tf.transformations.quaternion_from_euler(0, 0, sw23p[3]+b[3])
sw23=[(sw23p[0]-b[0])*m.cos(b[3])-(sw23p[1]-b[1])*m.sin(b[3])+b[0],(sw23p[0]-b[0])*m.sin(b[3])+(sw23p[1]-b[1])*m.cos(b[3])+b[1],sw23p[2],quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

# Home Points to come back
home1=[2,0,10,0,0,0,1]
home2=[0,2,5,0,0,0,1]

# Waypoints to track
# Drone 1
z=[sw12,f6d,f5d,f4d,sw23,f6b,f5b,f4b,sw34,home2]
# Drone 2
y=[sw34,f6c,f5c,f4c,sw41,f6a,f5a,f4a,sw12,home1]

# Printing waypoints to .yaml
port = open ('facaderun_D1.yaml', 'w')
i=0
port.write("frame_id: arena")
port.write("\n")

while i<=len(z)-1:
    wp="wp_"+str(i)
    port.write(wp)   

    port.write(": ")

    port.write(str(z[i])) 
        
    port.write("\n")
    i=i+1
port.close()


port = open ('facaderun_D2.yaml', 'w')
i=0
port.write("frame_id: arena")
port.write("\n")

while i<=len(y)-1:
    wp="wp_"+str(i)
    port.write(wp)   

    port.write(": ")

    port.write(str(y[i])) 
        
    port.write("\n")
    i=i+1
port.close()
