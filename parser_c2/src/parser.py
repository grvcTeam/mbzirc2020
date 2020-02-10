#!/usr/bin/env python
import roslaunch
import rospy

# Class to read the brick order 
class parser:

    rospy.init_node('parser',anonymous=True)
    # Obtaining the file names through a launch file
    ugv_file = rospy.get_param('~UGV_data')
    uav_file = rospy.get_param('~UAV_data')
    
    # Routine to read the UGV bricks
    def UGV_parser_request(self):
        origin = open(self.ugv_file, 'r' )
        reading=(origin.readlines())

        # Converting the data files into an array 
        with open (self.ugv_file,'r') as f:
            l = [[str(num) for num in line.split(' ')] for line in f]
        
        # Declaring a dictionary for the UGV
        UGV_dict= dict()
        i=0
        j=0
        # Read the number of rows with i 
        while i<len(l):
            j=0
            # Read every column and delete \n from array if needed
            while j!=len(l[i]):
                if l[i][j]=="R\n":
                    l[i][j]="R"
                elif l[i][j]=="G\n":
                    l[i][j]="G"
                elif l[i][j]=="B\n":
                    l[i][j]="B"
                elif l[i][j]=="O\n":
                    l[i][j]="O"
                j=j+1       
            # Obtaining KEY in dictionary, name of the layer of the current row
            layer = 'L' + str((len(l)-i))
            # Creating dictionary whit format: "LAYER : LIST OF COLOURS"
            UGV_dict[str(layer)]=l[i]
            i=i+1            
        origin.close()
        # Return dictionary 
        return UGV_dict

    # Routine to read UAV bricks
    def UAV_parser_request(self):
        origin = open(self.uav_file, 'r' )
        reading=(origin.readlines())

        # Converting the data files into an array 
        with open (self.uav_file,'r') as f:
            l = [[str(num) for num in line.split(' ')] for line in f]
        
        # Declaring a dictionary for the UAV 
        UAV_dict= dict()
        
        segment_length=7  # Maximun lenght of segments
        segment_number=3  # Maximun number of segments
        seg = ['','','','','','',''] # Auxiliar list to store segment data of the layers
        i=0
        j=0
        jmax=1  # index needed to separate segments in their respective layer
        w=0 # Index needed to go through the auxiliary list
        while i<len(l):
            # Iteration to go through every segment
            while jmax<segment_number+1:  
                # Iteration to store every segment in a layer into a dictionary
                while j<segment_length*jmax:
                    if l[i][j]=="R\n":
                        l[i][j]="R"
                    elif l[i][j]=="G\n":
                        l[i][j]="G"
                    elif l[i][j]=="B\n":
                        l[i][j]="B"
                    elif l[i][j]=="O\n":
                        l[i][j]="O"
                    seg[w]=l[i][j]
                    j=j+1
                    w=w+1
                    # Create KEY to the dictionary, composed by Layer + Segment
                    segment = 'L' + str((len(l)-i)) + 'S' + str(jmax)
                # Adding layer and segment to dictionary with format: "LayerXSegmentY : LIST OF COLOURS" 
                UAV_dict[str(segment)]=str(seg)
                w=0
                jmax=jmax+1
            i=i+1  
            w=0
            j=0    
            jmax=1     
        origin.close()
        # Return UAV dictionary
        return UAV_dict

if __name__ == '__main__':

    # Read parser class and obtain dictionaries
    Read_data = parser()
    UAV = Read_data.UAV_parser_request()
    UGV = Read_data.UGV_parser_request()

    # # Uncomment this lines to print in terminal
    # for x, y in UAV.items():
    #     print (x,y)

    # for x, y in UGV.items():
    #     print (x,y)