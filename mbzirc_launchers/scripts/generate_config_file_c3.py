#!/usr/bin/env python

from os import remove 
import numpy as np
import array 
import roslaunch
import math as m
# origin = open( 'Data.txt', 'r' )

# building pose
b=[20,23,0,0]
# angle to fix facing fires
aux_angle=1.57

# vertical fires - facade a (facade 1) 
f4ap=[b[0]+0, b[1]+7.5, b[2]+5.3, -1.57]
f4a =[(f4ap[0]-b[0])*m.cos(b[3])-(f4ap[1]-b[1])*m.sin(b[3])+b[0],(f4ap[0]-b[0])*m.sin(b[3])+(f4ap[1]-b[1])*m.cos(b[3])+b[1],f4ap[2],0,0,f4ap[3]+b[3]]
f5ap=[b[0]+0, b[1]+5, b[2]+7.5, -1.57]
f5a=[(f5ap[0]-b[0])*m.cos(b[3])-(f5ap[1]-b[1])*m.sin(b[3])+b[0],(f5ap[0]-b[0])*m.sin(b[3])+(f5ap[1]-b[1])*m.cos(b[3])+b[1],f5ap[2],0,0,f5ap[3]+b[3]]
f6ap=[b[0]+0, b[1]+6.5, b[2]+10.3, -1.57]
f6a=[(f6ap[0]-b[0])*m.cos(b[3])-(f6ap[1]-b[1])*m.sin(b[3])+b[0],(f6ap[0]-b[0])*m.sin(b[3])+(f6ap[1]-b[1])*m.cos(b[3])+b[1],f6ap[2],0,0,f6ap[3]+b[3]]

# vertical fires - facade c (facade 2)
f4cp = [b[0]+4.5, b[1]+0, b[2]+5.3, 0]
f4c = [(f4cp[0]-b[0])*m.cos(b[3])-(f4cp[1]-b[1])*m.sin(b[3])+b[0],(f4cp[0]-b[0])*m.sin(b[3])+(f4cp[1]-b[1])*m.cos(b[3])+b[1],f4cp[2],0,0,f4cp[3]+b[3]]
f5cp=[b[0]+2, b[1]+0, b[2]+9.2, 0]
f5c=[(f5cp[0]-b[0])*m.cos(b[3])-(f5cp[1]-b[1])*m.sin(b[3])+b[0],(f5cp[0]-b[0])*m.sin(b[3])+(f5cp[1]-b[1])*m.cos(b[3])+b[1],f5cp[2],0,0,f5cp[3]+b[3]]
f6cp=[b[0]+8.7, b[1]+0, b[2]+12, 0]
f6c=[(f6cp[0]-b[0])*m.cos(b[3])-(f6cp[1]-b[1])*m.sin(b[3])+b[0],(f6cp[0]-b[0])*m.sin(b[3])+(f6cp[1]-b[1])*m.cos(b[3])+b[1],f6cp[2],0,0,f6cp[3]+b[3]]

# vertical fires - facade b (facade 3)
f4bp = [b[0]+10, b[1]+6, b[2]+5.3, 1.57]
f4b = [(f4bp[0]-b[0])*m.cos(b[3])-(f4bp[1]-b[1])*m.sin(b[3])+b[0],(f4bp[0]-b[0])*m.sin(b[3])+(f4bp[1]-b[1])*m.cos(b[3])+b[1],f4bp[2],0,0,f4bp[3]+b[3]]
f5bp=[b[0]+10, b[1]+8.7, b[2]+8.25, 1.57]
f5b=[(f5bp[0]-b[0])*m.cos(b[3])-(f5bp[1]-b[1])*m.sin(b[3])+b[0],(f5bp[0]-b[0])*m.sin(b[3])+(f5bp[1]-b[1])*m.cos(b[3])+b[1],f5bp[2],0,0,f5bp[3]+b[3]]
f6bp=[b[0]+10, b[1]+1.3, b[2]+11.25, 1.57]
f6b=[(f6bp[0]-b[0])*m.cos(b[3])-(f6bp[1]-b[1])*m.sin(b[3])+b[0],(f6bp[0]-b[0])*m.sin(b[3])+(f6bp[1]-b[1])*m.cos(b[3])+b[1],f6bp[2],0,0,f6bp[3]+b[3]]

# vertical fires - facade d (facade 4)
f4dp=[b[0]+4.7, b[1]+10, b[2]+5.5, -3.1416]
f4d=[(f4dp[0]-b[0])*m.cos(b[3])-(f4dp[1]-b[1])*m.sin(b[3])+b[0],(f4dp[0]-b[0])*m.sin(b[3])+(f4dp[1]-b[1])*m.cos(b[3])+b[1],f4dp[2],0,0,f4dp[3]+b[3]]
f5dp=[b[0]+1.5, b[1]+10, b[2]+9.2, 3.1416]
f5d=[(f5dp[0]-b[0])*m.cos(b[3])-(f5dp[1]-b[1])*m.sin(b[3])+b[0],(f5dp[0]-b[0])*m.sin(b[3])+(f5dp[1]-b[1])*m.cos(b[3])+b[1],f5dp[2],0,0,f5dp[3]+b[3]]
f6dp=[b[0]+8.7, b[1]+10, b[2]+12.5, 3.1416]
f6d=[(f6dp[0]-b[0])*m.cos(b[3])-(f6dp[1]-b[1])*m.sin(b[3])+b[0],(f6dp[0]-b[0])*m.sin(b[3])+(f6dp[1]-b[1])*m.cos(b[3])+b[1],f6dp[2],0,0,f6dp[3]+b[3]]

# vertical fire - building extension || facade a
f4ep=[b[0]+0, b[1]+11.3, b[2]+2, -1.57]
f4e=[(f4ep[0]-b[0])*m.cos(b[3])-(f4ep[1]-b[1])*m.sin(b[3])+b[0],(f4ep[0]-b[0])*m.sin(b[3])+(f4ep[1]-b[1])*m.cos(b[3])+b[1],f4ep[2],0,0,f4ep[3]+b[3]]


# print order - facade fires to .yaml
z=[f4a,f4b,f4c,f4d,f4e,f5a,f5b,f5c,f5d,f6a,f6b,f6c,f6d]

# outdoor fires
f7=[30,10,0,0]
f8=[10,40,0,-1.570795]
# print order - outdoor fires
out=[f7,f8]

# windows - facade a (facade 1)
a1p=[b[0], b[1]+3, b[2]+12.5,-1.57+aux_angle]
a1=[(a1p[0]-b[0])*m.cos(b[3])-(a1p[1]-b[1])*m.sin(b[3])+b[0],(a1p[0]-b[0])*m.sin(b[3])+(a1p[1]-b[1])*m.cos(b[3])+b[1],a1p[2],a1p[3]+b[3]]
a2p=[b[0], b[1]+7, b[2]+12.5,-1.57+aux_angle]
a2=[(a2p[0]-b[0])*m.cos(b[3])-(a2p[1]-b[1])*m.sin(b[3])+b[0],(a2p[0]-b[0])*m.sin(b[3])+(a2p[1]-b[1])*m.cos(b[3])+b[1],a2p[2],a2p[3]+b[3]]
a3p=[b[0], b[1]+3, b[2]+7.5,-1.57+aux_angle]
a3=[(a3p[0]-b[0])*m.cos(b[3])-(a3p[1]-b[1])*m.sin(b[3])+b[0],(a3p[0]-b[0])*m.sin(b[3])+(a3p[1]-b[1])*m.cos(b[3])+b[1],a3p[2],a3p[3]+b[3]]
a4p=[b[0], b[1]+7, b[2]+7.5,-1.57+aux_angle]
a4=[(a4p[0]-b[0])*m.cos(b[3])-(a4p[1]-b[1])*m.sin(b[3])+b[0],(a4p[0]-b[0])*m.sin(b[3])+(a4p[1]-b[1])*m.cos(b[3])+b[1],a4p[2],a4p[3]+b[3]]
# door - facade a
a5p=[b[0], b[1]+7, b[2]+1.5,-1.57+aux_angle]
a5=[(a5p[0]-b[0])*m.cos(b[3])-(a5p[1]-b[1])*m.sin(b[3])+b[0],(a5p[0]-b[0])*m.sin(b[3])+(a5p[1]-b[1])*m.cos(b[3])+b[1],a5p[2],a5p[3]+b[3]]

## windows - facade b (facade 3)
b1p=[b[0]+10, b[1]+3, b[2]+12.5,1.57+aux_angle]
b1=[(b1p[0]-b[0])*m.cos(b[3])-(b1p[1]-b[1])*m.sin(b[3])+b[0],(b1p[0]-b[0])*m.sin(b[3])+(b1p[1]-b[1])*m.cos(b[3])+b[1],b1p[2],b1p[3]+b[3]]
b2p=[b[0]+10, b[1]+7, b[2]+12.5,1.57+aux_angle]
b2=[(b2p[0]-b[0])*m.cos(b[3])-(b2p[1]-b[1])*m.sin(b[3])+b[0],(b2p[0]-b[0])*m.sin(b[3])+(b2p[1]-b[1])*m.cos(b[3])+b[1],b2p[2],b2p[3]+b[3]]
b3p=[b[0]+10, b[1]+3, b[2]+7.5,1.57+aux_angle]
b3=[(b3p[0]-b[0])*m.cos(b[3])-(b3p[1]-b[1])*m.sin(b[3])+b[0],(b3p[0]-b[0])*m.sin(b[3])+(b3p[1]-b[1])*m.cos(b[3])+b[1],b3p[2],b3p[3]+b[3]]
b4p=[b[0]+10, b[1]+7, b[2]+7.5,1.57+aux_angle]
b4=[(b4p[0]-b[0])*m.cos(b[3])-(b4p[1]-b[1])*m.sin(b[3])+b[0],(b4p[0]-b[0])*m.sin(b[3])+(b4p[1]-b[1])*m.cos(b[3])+b[1],b4p[2],b4p[3]+b[3]]
# door - facade b
b5p=[b[0]+10, b[1]+7, b[2]+1.5,1.57+aux_angle]
b5=[(b5p[0]-b[0])*m.cos(b[3])-(b5p[1]-b[1])*m.sin(b[3])+b[0],(b5p[0]-b[0])*m.sin(b[3])+(b5p[1]-b[1])*m.cos(b[3])+b[1],b5p[2],b5p[3]+b[3]]

# windows - facade c (facade 4)
c1p=[b[0]+3, b[1]+0, b[2]+12.5,0+aux_angle]
c1=[(c1p[0]-b[0])*m.cos(b[3])-(c1p[1]-b[1])*m.sin(b[3])+b[0],(c1p[0]-b[0])*m.sin(b[3])+(c1p[1]-b[1])*m.cos(b[3])+b[1],c1p[2],c1p[3]+b[3]]
c2p=[b[0]+7, b[1]+0, b[2]+12.5,0+aux_angle]
c2=[(c2p[0]-b[0])*m.cos(b[3])-(c2p[1]-b[1])*m.sin(b[3])+b[0],(c2p[0]-b[0])*m.sin(b[3])+(c2p[1]-b[1])*m.cos(b[3])+b[1],c2p[2],c2p[3]+b[3]]
c3p=[b[0]+3, b[1]+0, b[2]+7.5,0+aux_angle]
c3=[(c3p[0]-b[0])*m.cos(b[3])-(c3p[1]-b[1])*m.sin(b[3])+b[0],(c3p[0]-b[0])*m.sin(b[3])+(c3p[1]-b[1])*m.cos(b[3])+b[1],c3p[2],c3p[3]+b[3]]
c4p=[b[0]+7, b[1]+0, b[2]+7.5,0+aux_angle]
c4=[(c4p[0]-b[0])*m.cos(b[3])-(c4p[1]-b[1])*m.sin(b[3])+b[0],(c4p[0]-b[0])*m.sin(b[3])+(c4p[1]-b[1])*m.cos(b[3])+b[1],c4p[2],c4p[3]+b[3]]
# garage door - facade c
c5p=[b[0]+5, b[1]+0, b[2]+1.5,0+aux_angle]
c5=[(c5p[0]-b[0])*m.cos(b[3])-(c5p[1]-b[1])*m.sin(b[3])+b[0],(c5p[0]-b[0])*m.sin(b[3])+(c5p[1]-b[1])*m.cos(b[3])+b[1],c5p[2],c5p[3]+b[3]]

# windows - facade d (facade 2)
d1p=[b[0]+3, b[1]+10, b[2]+12.2,3.14+aux_angle]
d1=[(d1p[0]-b[0])*m.cos(b[3])-(d1p[1]-b[1])*m.sin(b[3])+b[0],(d1p[0]-b[0])*m.sin(b[3])+(d1p[1]-b[1])*m.cos(b[3])+b[1],d1p[2],d1p[3]+b[3]]
d2p=[b[0]+7, b[1]+10, b[2]+12.5,3.1416+aux_angle]
d2=[(d2p[0]-b[0])*m.cos(b[3])-(d2p[1]-b[1])*m.sin(b[3])+b[0],(d2p[0]-b[0])*m.sin(b[3])+(d2p[1]-b[1])*m.cos(b[3])+b[1],d2p[2],d2p[3]+b[3]]
d3p=[b[0]+3, b[1]+10, b[2]+7.5,3.1416+aux_angle]
d3=[(d3p[0]-b[0])*m.cos(b[3])-(d3p[1]-b[1])*m.sin(b[3])+b[0],(d3p[0]-b[0])*m.sin(b[3])+(d3p[1]-b[1])*m.cos(b[3])+b[1],d3p[2],d3p[3]+b[3]]
d4p=[b[0]+7, b[1]+10, b[2]+7.5,3.1416+aux_angle]
d4=[(d4p[0]-b[0])*m.cos(b[3])-(d4p[1]-b[1])*m.sin(b[3])+b[0],(d4p[0]-b[0])*m.sin(b[3])+(d4p[1]-b[1])*m.cos(b[3])+b[1],d4p[2],d4p[3]+b[3]]

# windows - facade extension ground floor
# ||facade a
e1p=[b[0]+0, b[1]+14, b[2]+2,-1.57+aux_angle]
e1=[(e1p[0]-b[0])*m.cos(b[3])-(e1p[1]-b[1])*m.sin(b[3])+b[0],(e1p[0]-b[0])*m.sin(b[3])+(e1p[1]-b[1])*m.cos(b[3])+b[1],e1p[2],e1p[3]+b[3]]
# ||facade d
e2p=[b[0]+5, b[1]+20, b[2]+2,3.1416+aux_angle]
e2=[(e2p[0]-b[0])*m.cos(b[3])-(e2p[1]-b[1])*m.sin(b[3])+b[0],(e2p[0]-b[0])*m.sin(b[3])+(e2p[1]-b[1])*m.cos(b[3])+b[1],e2p[2],e2p[3]+b[3]]
# ||facade c
e3p=[b[0]+10, b[1]+14, b[2]+2,1.57+aux_angle]
e3=[(e3p[0]-b[0])*m.cos(b[3])-(e3p[1]-b[1])*m.sin(b[3])+b[0],(e3p[0]-b[0])*m.sin(b[3])+(e3p[1]-b[1])*m.cos(b[3])+b[1],e3p[2],e3p[3]+b[3]]

# print order - windows and doors to .yaml
p=[a1,a2,a3,a4,a5,b1,b2,b3,b4,b5,c1,c2,c3,c4,c5,d1,d2,d3,d4,e1,e2,e3]

# printing all to .yaml
port = open ('../config/conf_ch3.yaml', 'w')

port.write("# Include only objects whose positions we know, even if they are inactive. For instance, if we have several possible poses for the facades, but only active per trial.\n")
port.write("arena:\n  x_min: 0\n  x_max: 50\n  y_min: 0\n  y_max: 60\n\nbuilding:")
port.write("\n  x_min: ")
port.write(str(b[0]))
port.write("\n  x_max: ")
port.write(str(b[0]+10))
port.write("\n  y_min: ")
port.write(str(b[1]))
port.write("\n  y_max: ")
port.write(str(b[1]+10))
port.write("\n  y_base_max: ")
port.write(str(b[1]+20))
port.write("\n  first_floor: 5\n  second_floor: 10\n\n")

port.write("fire:\n\n")
i=0
while i<=len(z)-1:
    wp="  - sub_type: \"facadefire\"\n    frame_id: \"arena\"\n    position_x: "
    port.write(wp)   
    port.write(str(z[i][0]))
    port.write("\n")
    wp_y="    position_y: "
    port.write(wp_y)
    port.write(str(z[i][1]))
    port.write("\n")
    wp_z="    position_z: "
    port.write(wp_z)
    port.write(str(z[i][2]))
    port.write("\n")
    wp_yaw="    yaw: "
    port.write(wp_yaw)
    port.write(str(z[i][3]))
    port.write("\n\n")

    i=i+1

i=0
while i<=len(out)-1:
    wp="  - sub_type: \"outdoorfire\"\n    frame_id: \"arena\"\n    position_x: "
    port.write(wp)   
    port.write(str(out[i][0]))
    port.write("\n")
    wp_y="    position_y: "
    port.write(wp_y)
    port.write(str(out[i][1]))
    port.write("\n")
    wp_z="    position_z: "
    port.write(wp_z)
    port.write(str(out[i][2]))
    port.write("\n")
    wp_yaw="    yaw: "
    port.write(wp_yaw)
    port.write(str(out[i][3]))
    port.write("\n\n")

    i=i+1

port.write("\n")
port.write("passage:\n\n")
i=0
while i<=len(p)-1:
    if (p[i][2])<5:
        wp="  - sub_type: \"ground\"\n    frame_id: \"arena\"\n    position_x: "
        port.write(wp)   
        port.write(str(p[i][0]))
        port.write("\n")
        wp_y="    position_y: "
        port.write(wp_y)
        port.write(str(p[i][1]))
        port.write("\n")
        wp_z="    position_z: "
        port.write(wp_z)
        port.write(str(p[i][2]))
        port.write("\n")
        wp_yaw="    yaw: "
        port.write(wp_yaw)
        port.write(str(p[i][3]))
        port.write("\n\n")
    elif (p[i][2])<10:
        wp="  - sub_type: \"first\"\n    frame_id: \"arena\"\n    position_x: "
        port.write(wp)   
        port.write(str(p[i][0]))
        port.write("\n")
        wp_y="    position_y: "
        port.write(wp_y)
        port.write(str(p[i][1]))
        port.write("\n")
        wp_z="    position_z: "
        port.write(wp_z)
        port.write(str(p[i][2]))
        port.write("\n")
        wp_yaw="    yaw: "
        port.write(wp_yaw)
        port.write(str(p[i][3]))
        port.write("\n\n")
    elif (p[i][2])>10:
        wp="  - sub_type: \"second\"\n    frame_id: \"arena\"\n    position_x: "
        port.write(wp)   
        port.write(str(p[i][0]))
        port.write("\n")
        wp_y="    position_y: "
        port.write(wp_y)
        port.write(str(p[i][1]))
        port.write("\n")
        wp_z="    position_z: "
        port.write(wp_z)
        port.write(str(p[i][2]))
        port.write("\n")
        wp_yaw="    yaw: "
        port.write(wp_yaw)
        port.write(str(p[i][3]))
        port.write("\n\n")

    i=i+1
