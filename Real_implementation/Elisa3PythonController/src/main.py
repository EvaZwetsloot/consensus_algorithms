from nodes import Nodes
import numpy as np
import rospy
import json
import time
from configuration import *


if __name__ == "__main__":
    with open('mapper.json') as json_file:
        mapper = json.load(json_file)
    active_robots = list(mapper.keys())

    # Init phase
    print("start")
    nodes = Nodes(active_robots)

    # Set Leds
    nodes.set_leds(green=0, blue=0, red=10)


    # Reset the robot odom in the beginning
    for i in range(3):
        print("wait for odom response")
        # nodes.move('still', step_size= 0.0, theta=0.)
        rospy.sleep(1)
    print("start reset")
    # nodes.reset('theor')

    # Move Robots
    last_saved_time = 0
    step_size = 1.0
    theta = 0.0
    t_last = time.time()
    t = time.time()
    t_save = 0
    t_last_save = time.time()
    t_int = 0
    t_time_diff = time.time()
    t_time = 0
    #for t in range(last_saved_time, 100):
    while(t_time < 100):
        t_time = time.time()-t_time_diff

        print("t: ", t)
        nodes.store_data(t_int)

        nodes.loop_fuc(t_int, 'move')
        t = time.time()-t_last
        t_save = time.time() - t_last_save

        if (t > 0.5):
            t_last = time.time()
            nodes.reset('theor')

        if(t_save > 0.05):
            t_last_save = time.time()
            t_int +=1
            nodes.k +=1
            print("SAVE DATA")

    nodes.save_data(0)

    # Stop engine
    nodes.move('still', step_size= 0.0, theta=0.)

    print('done')