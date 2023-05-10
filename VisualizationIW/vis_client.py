### EXAMPLE CLIENT CODE

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
visualizer.add_object("Drone", "drone.png", listeners={"drone_x":["x"], "drone_y":["y"]}, init_x = 7, scale_x=0.2, scale_y=0.2)

#Add a function dependent on, for example, where the drone is.
#Requires a signature of ["drone_y"]
def func_defined(x,y, drone_y):
    return x+y-drone_y

def cool_area(x, y):
    x -= 50
    y -= 50
    x /= 15
    y /= 25
    #equation inspired by matplotlib coutour demo
    #https://matplotlib.org/stable/gallery/images_contours_and_fields/contour_demo.html
    Z1 = np.exp(-x**2 - y**2)
    Z2 = np.exp(-(x - 1)**2 - (y - 1)**2)
    Z = (Z1 - Z2) * 2
    return Z*100
#These arguments indicate that the negative 
#area is filled and the positive area is not.
#Also, level sets of several values will be plotted.
visualizer.add_function("Function-defined sprite 1", 
cool_area, [], False, True, [-150.0, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0])

def interesting_spline(s):
    #expect range s=0 to s=80 inclusive
    if s<=10:
        return (20+s*2, 10)
    elif s <= 30:
        return (40, 10+(s-10)*2)
    elif s <= 40:
        return(40+(s-30)*2, 50)
    elif s <= 50:
        return (60, 50-(s-40)*2)
    elif s <= 70:
        return (60-(s-50)*2, 30)
    else:
        return (20, 30-(s-70)*2)

#drone-dependent spline - to use this, make sure signature=["drone_y"] for the spline part you pass in
def line(s, drone_y):
    return [(s),(drone_y + s%5)]

visualizer.add_spline("Interesting spline", [[interesting_spline, 0, 81, []]], 1)

#The main code is moved to a thread, because run_app() blocks while the app is open.
class motion_sim_thread(threading.Thread):
    def run(self):
        for i in range(50):
            time.sleep(0.25)
            visualizer.update_state({"drone_y":i*2}) #Move the drone up 2 every 0.25 second
sim = motion_sim_thread()
sim.start()
visualizer.run_app()




