#!/usr/bin/env python

from os import remove 
import numpy as np
import array 
import roslaunch
import rospy
import sys

# Global variables to obtain data from launcher
global a
global b
global c

# Routine to extract information from data given in txt
def parser_request():
    origin = open(a, 'r' )
    # Creating an auxiliary file to extract information
    port = open ('Puente_es.txt', 'w')
    reading=(origin.readlines())
    # Converting every word with a comma or a symbol into a simple string
    for renglon in reading:
        for palabra in renglon.split(' '):
            dest=palabra.lower()
            if palabra=="1," or palabra=="1:":
                dest="1"
            elif palabra=="2," or palabra=="2:":
                dest="2"
            elif palabra=="3," or palabra=="3:":
                dest="3"
            elif palabra=="4," or palabra=="4:":
                dest="4"
            elif palabra=="5," or palabra=="5:":
                dest="5"
            elif palabra=="6," or palabra=="6:":
                dest="6"
            elif palabra=="7," or palabra=="7:":
                dest="7"
            elif palabra=="8," or palabra=="8:":
                dest="8"
            elif palabra=="9," or palabra=="9:":
                dest="9"
            else:
                port.write(" ")
            port.write(dest)
    port.close()

    port = open ('Puente_es.txt','r')
    fin = open(c,'w')
    reading_port=(port.readlines())

    for renglon in reading_port:
        for palabra in renglon.split(' '):
            # Translate UGV orders
            if palabra=="segment1":
                dest="S1"
            elif palabra=="segment2":
                dest="S2"
            elif palabra=="segment3":
                dest="S3"
            elif palabra=="segment4":
                dest="S4"
        # Translate UAV orders
            elif palabra=="channel1":
                dest="C1"
            elif palabra=="channel2":
                dest="C2"
            elif palabra=="channel3":
                dest="C3"
            elif palabra=="channel4":
                dest="C4"
            elif palabra=="channel5":
                dest="C5"
            elif palabra=="channel6":
                dest="C6"
        # Translate layers
            elif palabra=="layer1":
                dest=".1 " 
            elif palabra=="layer2":
                dest=".2 "
            elif palabra=="layer3":
                dest=".3 "
            elif palabra=="layer4":
                dest=".4 "
            elif palabra=="layer5":
                dest=".5 "
            elif palabra=="layer6":
                dest=".6 "
            elif palabra=="layer7":
                dest=".7 "
            elif palabra==" ":
                dest=" "
            # Translate colours and endline colours 
            elif palabra=="blue" or palabra=="blue\n":
                dest="B "
            elif palabra=="red" or palabra=="red\n":
                dest="R "
            elif palabra=="green" or palabra=="green\n":
                dest="G "
            elif palabra=="orange" or palabra=="orange\n":
                dest="O "
            else:
                dest=""
            fin.write(dest)
        fin.write("\n")
    fin.close()
    #Eliminating auxiliary txt
    remove('Puente_es.txt')

    fin = open(c,'r')
    reading_port=(fin.readlines())    
    # Obtaining segment-layer or channel-layer to read and print
    valor=b
    with open (c,'r') as f:
        l = [[str(num) for num in line.split(' ')] for line in f]

    i=0
    j=1
    while i<len(l):
        if l[i][j-1]==valor:
            # Uncomment next line to print the complete array
            # print(l[i])
            while j!=len(l[i]):
                print(l[i][j])
                j=j+1
            j=1
        i=i+1            
    
    fin.close()
            
        
            
if __name__ == '__main__':
    rospy.init_node('parser',anonymous=True)
    
    a = rospy.get_param('~File_data')
    b = rospy.get_param('~Line_to_read')
    c = rospy.get_param('~Extracted_data')
    parser_request()
