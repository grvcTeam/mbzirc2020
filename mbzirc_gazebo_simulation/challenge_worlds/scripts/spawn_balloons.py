#!/usr/bin/python

import random
import getopt
import sys
import re
import csv
import subprocess
import rospkg

rospack = rospkg.RosPack()
ch1_path = rospack.get_path('challenge_worlds')

################################################################################
#                            DEFAULT PARAMETERS                                #
################################################################################

rand = False
csv_path = ch1_path + "/config/dist1.csv"
xacro_path = ch1_path + "/urdf/balloon.xacro"
max_height = 4
size_x = 90
size_y = 50


colour_list = ['green', 'blue', 'orange', 'yellow', 'white', 'black', 'red']
################################################################################

# TODO: READ CORRECTLY PARAMETERS
# TODO: IMPLEMENT RANDOM PLACES


aux_prev_ids = []


def main():
    global rand, csv_path, xacro_path

    with open(csv_path) as csvfile:

        balloonsreader = csv.reader(csvfile, delimiter=',')
        for balloon in balloonsreader:  # balloon = [id, x, y, z, colour]

            colour = balloon[4]
            # cast numbers in the list
            balloon = [float(i) for i in balloon[0:4]]
            balloon.append(re.sub(" ", "", colour))
            bound(balloon)

            colour = "Gazebo/"+balloon[4].lower()

            command = 'rosrun xacro xacro --inorder {path} id:={i} height:={h} colour:={c} > tmp.sdf'\
                .format(path=xacro_path, i=balloon[0], h=balloon[3], c=colour)

            proc = subprocess.Popen(
                ['/bin/bash'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            stdout = proc.communicate(command)

            command = 'rosrun gazebo_ros spawn_model -file tmp.sdf -sdf -model {name} -y {y} -x {x}'\
                .format(name="balloon_"+str(int(balloon[0])), x=balloon[1], y=balloon[2])
            proc = subprocess.Popen(
                ['/bin/bash'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            stdout = proc.communicate(command)


def getArgs():
    # TODO: fix file_path as argument
    global rand, csv_path, size_x, size_y, max_height, MIN_X, MIN_Y, MIN_Z, MAX_X, MAX_Y, MAX_Z

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hrf:x:y:z:", [
                                   "help", "random", "file=", "xsize=", "ysize=", "zsize="])
    except getopt.GetoptError as err:
        print str(err)
        # usage()
        sys.exit(2)

    for o, a in opts:
        if o == "-r":
            rand = True
        elif o in ("-h", "--help"):
            help()
            sys.exit()
        elif o in ("-f", "--file"):
            print "CSVPATH: "+str(a)
            csv_path = str(a)
        elif o in ("-x", "--xsize"):
            size_x = float(a)
            print "size_x: "+a
        elif o in ("-y", "--ysize"):
            size_y = float(a)
            print "size_y: "+a
        elif o in ("-z", "--zsize"):
            max_height = float(a)
            print "size_z: "+a
        else:
            assert False, "unhandled option"

    MAX_X = 100-((100-size_x)/2)
    MIN_X = (100-size_x)/2
    MAX_Y = 60-((60-size_y)/2)
    MIN_Y = (60-size_y)/2
    MAX_Z = max_height
    MIN_Z = 0


def bound(balloon):

    global MIN_X, MIN_Y, MIN_Z, MAX_X, MAX_Y, MAX_Z, aux_prev_id, colour_list

    while(balloon[0] in aux_prev_ids):
        print("[{}]: Id {} was repeated. Id --> Id++").format(__file__, balloon[0])
        balloon[0] += 1

    aux_prev_ids.append(balloon[0])

    if(balloon[1] < MIN_X or balloon[1] > MAX_X):
        print("[{}]: Balloon with index {} is out of bounds [x]. Will be move to random place").format(
            __file__, int(balloon[0]))
        balloon[1] = random.randint(MIN_X, MAX_X)

    if(balloon[2] < MIN_Y or balloon[2] > MAX_Y):
        print("[{}]: Balloon with index {} is out of bounds [Y]. Will be move to random place").format(
            __file__, int(balloon[0]))
        balloon[2] = random.randint(MIN_Y, MAX_Y)

    if(balloon[3] < MIN_Z or balloon[3] > MAX_Z):
        print("[{}]: Balloon with index {} is too high. Height will be randomly set").format(
            __file__, int(balloon[0]))
        balloon[3] = random.randint(MIN_Z, MAX_Z)

    if not(balloon[4].lower() in colour_list):
        print(balloon[4])
        print(colour_list)
        print("[{}]: Balloon with index {} has a wrong colour. Will be set to red").format(
            __file__, int(balloon[0]))
        balloon[4] = 'red'


def help():

    print "TODO: help "


if __name__ == "__main__":
    getArgs()
    main()
