### CLIENT CODE

import numpy as np
import visualizer #This line runs the setup of visualizer.py
import time
import threading

# State variables are given custom nicknames to refer to them later, and initial values.
# This allows multiple constructed objects to refer to the same variable in an intuitive way.
visualizer.set_state_and_key(False, {"drone_x":0,"drone_y":0,"drone_z":0})

# Coordinates given to the visualizer are read as if they are in "client coordinate space" 
# or "on the client canvas" - this is because my code will not know what scale you are using, 
# whether you mean x=100 to be halfway across the screen or all the way across, etc.
# I will assume that (0,0) is the bottom left corner; simply tell the visualizer what your max x and max y coordinates are.
visualizer.set_coord_system(100, 100)
# Now, a sprite rendered at x=75 y=75 will show up with its bottom left corner aligned 3/4 of the way over and 3/4 of the way up the canvas.
# Splines with width 5 will take 5/100 of the canvas width. Etc



#Add the drone and its PNG source file.
#Listeners correspondence and object scaling will be improved, but in summary this 
# sets the drone sprite's x property to the state variable drone_x whenever drone_x changes, and likewise for y.
visualizer.add_object("Drone", "drone.png", listeners={"drone_x":["x"], "drone_y":["y"]}, scale_x=0.2, scale_y=0.2)



#Add a function dependent on, for example, where the drone is.
def func_defined(x,y, drone_y):
    return x+y-drone_y
#These arguments indicate that the negative area is filled and the positive area is not.
#Also, level sets of several values will be plotted.
visualizer.add_function("Function-defined sprite 1", func_defined, ["drone_y"], False, True, [-50.0, 10.0, 50.0, 150.0])


def line(s, drone_y):
    return [(s),(drone_y + s%5)]
#This adds a one-segment spline from s=10 to s=60, dependent on drone y, with width 1.
visualizer.add_spline("Zigzag spline", [[line, 10, 60, ["drone_y"]]], 1)

#The main code is moved to a thread, because run_app() blocks while the app is open.
class motion_sim_thread(threading.Thread):
    def run(self):
        for i in range(50):
            time.sleep(0.5)
            visualizer.update_state({"drone_y":i*2}) #Move the drone up 2 every half second
sim = motion_sim_thread()
sim.start()
visualizer.run_app()




