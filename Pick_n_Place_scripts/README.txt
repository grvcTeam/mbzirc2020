This folder contains a series of scripts designed for the UGV actions. Each script corresponds to a class from which some are called by others.

edge_detector_class.py - this class receives as input the side (string "left" or "right"), color (string "red", "blue", "green, "orange")
    and level (int 1 to 5) of the brick which will be used as reference. It will output a PointStamped corresponding to the point of the desired edge. 
    This edge is then used as reference. The script itself does color and depth segmentation using opencv.

frame_allign.py - this script is yet to be tested but is designed to allign the UGV with the frame and position for the first brick only.

laser_allign_class.py - this script uses the Hokuyo data to allign the UGV with the bricks to pick or place.

pcloud_concat.py - this script is designed to merge pointclouds. We are currently not using this approach as it is very demanding.

pick_place.py - This script is designed as an "action server". It calls the required functions, from the other classes, to complete a single pick and place. 

pick_RGB_class.py - This class handles the picking by the UGV. It will call the plate detection class for the detection of the metal plate. 

place_algorithm_class.py - This class handles the placing algorithm and call the edge detection to allign the placing position with existing bricks.

plate_detector_class.py - This class is designed to detect the brick plates. Currently the HSV values are tuned for grey metal but they can easily be changed to white. 
    Additionally, this class applies a depth filter to remove the ground (which in the tests we performed was also grey). Lastly, it rotates the position according to our 
    expected input for the arm movement. It all runs in opencv.

wall_dispatcher.py - This class uses a wall configuration, manually added, and computes the parameters to call the pick place class considering the inputs necessary
    for all the classes involved. Currently it has a getchar and prints to be visually possible to assess. 

FOR UAV:
    possible interesting scripts are the edge_detector_class, plate_detector_class and wall_dispatcher.