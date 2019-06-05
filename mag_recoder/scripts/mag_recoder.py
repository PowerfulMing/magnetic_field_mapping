#!/usr/bin/env python  
# ROS
import roslib
roslib.load_manifest('mag_recoder')
import rospy
import tf
# Utils
import math
import numpy as np
import cv2
# Read Yaml File
import yaml
import re
# Plotting Map
import matplotlib as mpl
import matplotlib.cm as mtpltcm
import matplotlib.pyplot as plt
import time

def read_pgm(filename, byteorder='>'):
    """
        Return image data from a raw PGM file as numpy array.
    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))


def calc_p2p_dist(x1, y1, x2, y2):
    return math.hypot((x1-x2),(y1-y2))
    

if __name__ == '__main__':
    map_origin = [0.0, 0.0, 0.0]
    map_resolution = 0.0;
    
    # Read YAML
    stream = open("/home/lui/itri_map.yaml", "r")
    docs = yaml.load_all(stream)
    for doc in docs:
        for k,v in doc.items():
            if k=='origin':
                map_origin = v
            if k=='resolution':
                map_resolution = v;

    # Read Map
    image = read_pgm("/home/lui/itri_map.pgm", byteorder='<')
    
    # Generate Mag Map
    mag_map_resolution = 0.5
    height = int(image.shape[0]*map_resolution/mag_map_resolution)
    width = int(image.shape[1]*map_resolution/mag_map_resolution)
    mag_map = np.zeros((height, width, 3))
    print("(Generate Magnetic Map... size: %(x)s* %(y)s" % {'x': width, 'y': height})
    #initialize the colormap (jet)
    colormap = mpl.cm.jet
    #add a normalization
    cNorm = mpl.colors.Normalize(vmin=0, vmax=100)
    #init the mapping
    scalarMap = mtpltcm.ScalarMappable(norm=cNorm, cmap=colormap)
    # Draw Map
    im = plt.imshow(mag_map)
    plt.colorbar()

    # Param
    pre_location = [0.0, 0.0];

    # Start ROS Node
    rospy.init_node('mag_recoder')
    listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # print(trans)        
        if(calc_p2p_dist(pre_location[0],pre_location[1],trans[0],trans[1])%mag_map_resolution < mag_map_resolution):
            y = int(trans[0]/mag_map_resolution - map_origin[0]/mag_map_resolution)
            x = int(trans[1]/mag_map_resolution - map_origin[1]/mag_map_resolution)
            
            if x>=height or y>= width or x<0 or y<0:
                continue
            value = int(math.sqrt(x*x + y*y))
            mag_map[x][y] = value

            print("(X: %(x)s, Y: %(y)s) = %(value)s" % {'x': x, 'y': y, 'value': mag_map[x][y]})
        
        show_mag_map = cv2.flip(mag_map, 0)
        gray = cv2.cvtColor(show_mag_map.astype(np.uint8), cv2.COLOR_BGR2GRAY)
        colors = scalarMap.to_rgba(gray)
            
        im.set_data(colors)
        plt.pause(0.1)
