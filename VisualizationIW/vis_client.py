"""
Example Client code and instructions for the project:
Real-Time Visualization for Reachability Analysis in Robotic Motion-Planning Systems
Anna Lucas
"""

import numpy as np
import visualizer  # This line runs the setup of visualizer.py
import time
import sys
import threading
import SettingsGUI


### Initial Setup

### State Vector
# State variables are given custom nicknames to refer to them later, and initial values.
# This allows multiple constructed objects to refer to the same variable in an intuitive way.
visualizer.set_initial_state_and_key(
    mode_auto=True,
    state_dict={"drone_1_x": 0.0, "drone_1_y": 0.0, "drone_2_x": 0.0, "drone_2_y": 0.0},
)

### Coordinates
# Coordinates given to the visualizer are read as if they are in "client coordinate space"
# or "on the client canvas" - this is because my code will not know what scale you are using,
# whether you mean x=100 to be halfway across the screen or all the way across, etc.
# I will assume that (0,0) is the bottom left corner; simply tell the visualizer what your max x and max y coordinates are.
visualizer.set_coord_system(100, 100)
# Now, a sprite rendered at x=75 y=75 will show up 3/4 of the way over and 3/4 of the way up the canvas.
# Splines with width 5 will take 5/100 of the canvas width. Etc

# Example coordinate usage: You are projecting with a square projector on a mat that
# is 12 feet by 12 feet. In addition to this, you expect your functions to return values in units of feet.
# No conversion is needed. Simply indicate visualizer.set_coord_system(12, 12) to set the scale and max
# of both axes.

# Example coordinate usage 2: Your projector space is 15 feet wide by 10 feet tall, but you only need
# to project on a 10' by 10' mat. I would recommend setting coordinates to (15, 10) and angling the
# projector such that the bottom left corner of the projection is aligned with the bottom
# left corner of the mat.

### Color Scheme
# Several optional arguments to this function are provided to change colors of items.
# Currently the background color is not yet configurable. Black (current default) is my strong
# recommendation for negative space in a projection context.

# Example: Change the fill color of positive areas of surface functions to green
visualizer.set_color_scheme(positive_fill_color=[0, 255, 0, 255])


### Render Objects

# Each object to be rendered needs to be added to the visualizer with the below wrapper functions
# The nickname helps to identify the object from the settings menu
# For image objects, the filename should be an image in the same directory
# Non-image shapes are not yet supported
# Indicate the links between this object's state and the state vector items with the state link arguments
# In this case the x_state_link argument indicates that the rendered image's x position is tied to the
# state vector item named "drone_1_x"
# The movement I mention in the nicknames is created by the client-side "Motion Simulation Thread", not the
# visualizer
visualizer.add_image_object(
    nickname="Drone 1 - moving in zigzag",
    filename_or_shape="drone.png",
    x_state_link="drone_1_x",
    y_state_link="drone_1_y",
    scale_x=0.2,
    scale_y=0.2,
)

visualizer.add_image_object(
    nickname="Drone 2 - moves along spline",
    filename_or_shape="drone.png",
    x_state_link="drone_2_x",
    y_state_link="drone_2_y",
    scale_x=0.2,
    scale_y=0.2,
)


### Area Functions
# The area function you provide for the sprite will be sampled across the whole canvas
# in x and y at a certain resolution.
# So, when the visualizer wants to know what to display at x=50 y=75, it will call your provided function
# with the arguments area_function(50, 75, ...)
# If the area function depends on items from the state vector, additional arguments need to be specified
# The link to your area function will be passed to the visualizer and treated as a black box on each resample


# This is an area function with no additional arguments or state dependence.
# Below is an example "Function-defined sprite 1" of defining an area function render object with it.
# Note that signature=[] because no additional arguments past the x and y sampling are needed.
def cool_area(x, y):
    x -= 50
    y -= 50
    x /= 15
    y /= 25
    # equation inspired by matplotlib contour demo
    # https://matplotlib.org/stable/gallery/images_contours_and_fields/contour_demo.html
    Z1 = np.exp(-(x**2) - y**2)
    Z2 = np.exp(-((x - 1) ** 2) - (y - 1) ** 2)
    Z = (Z1 - Z2) * 2
    return Z * 100 + 10


# This creates an area function covering the screen, sampled with the above function.
# The arguments indicate that the negative area is filled with color and the positive area is not.
# Also, level sets of several values will be plotted.
visualizer.add_function(
    nickname="Function-defined sprite 1",
    func=cool_area,
    signature=[],
    fill_pos=False,
    fill_neg=True,
    level_sets=[-150.0, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0],
)


# This is an example of an area function, to be queried across x and y, that takes two additional arguments
# When an area function is defined taking two additional arguments,
# indicate their link with the signature argument when creating the render object
# An example signature argument corresponding to this function:
# Say we want an area function that renders a small circle around Drone 1.
# The center x and center y arguments should be filled with the current location of drone 1
# while the x and y arguments appearing first will be automatically queried in many values by the sampler.
# signature = ["drone_1_x", "drone_1_y"] indicates that the two additional argument slots
# should be filled with the current x and y position of drone 1.
def circle_around_center(x, y, center_x, center_y):
    return 10 - np.linalg.norm([x - center_x, y - center_y])


# The x and y arguments will be incremented at regular intervals to sample from (0, 0) to the client coordinate
# space max, in a grid pattern.

# The below call adds an area function defined by circle_around_center and dependent on the drone 1 location.
# The other arguments indicate that the positive area of the function will be filled,
# the negative area will be transparent,
# and no level sets will be plotted.
visualizer.add_function(
    nickname="Area of Radius 10 client units around Drone 1",
    func=circle_around_center,
    signature=["drone_1_x", "drone_1_y"],
    fill_pos=True,
    fill_neg=False,
    level_sets=[],
    custom_sample_res=50,  # Downscale the sample resolution because this gets updated often
)


# This is a spline function I wrote to illustrate what plotted roads could look like.
# Splines can also depend on state vector items like the others, but I've omitted the example
def interesting_spline(s):
    # expect range s=0 to s=80 inclusive
    if s <= 10:
        return (20 + s * 2, 10)
    elif s <= 30:
        return (40, 10 + (s - 10) * 2)
    elif s <= 40:
        return (40 + (s - 30) * 2, 50)
    elif s <= 50:
        return (60, 50 - (s - 40) * 2)
    elif s <= 70:
        return (60 - (s - 50) * 2, 30)
    else:
        return (20, 30 - (s - 70) * 2)


# The spline parts argument should be a list of dictionaries in a specific format
# Each dictionary should include a "spline function", "min s", "max s", and "signature" entry
# This is provided so that a spline with many different functions defining its parts
#  can be grouped together when toggling visibility, etc
# My choice of example is to define the parts and ranges within the interesting_spline function and
#  only use one "spline part" in the visualizer, but it can be done either way
visualizer.add_spline(
    nickname="Interesting spline - figure 8 style",
    spline_parts=[
        {
            "spline function": interesting_spline,
            "min s": 0,
            "max s": 81,
            "signature": [],
        }
    ],
    width=1,
)


# Example function to see the gradient arrow grid working - note the two value return
def xy_arrowgrid(x, y):
    return [x / 100, y / 100]


# Working similarly to the area function and spline but sampled at spaced out intervals
# and expects a function returning two values [dx, dy] instead of one
visualizer.add_arrowgrid(
    nickname="Simple XY Arrow Grid",
    func=xy_arrowgrid,
    signature=[],
    arrow_scale=3,
    arrow_clarity=500,
)

# Example of changing the min and max on the settings gui slider of some state item
# Default slider goes from 0 to 100
visualizer.add_slider_override(item="drone_1_x", minmax=(-10, 150))


###Threading
# My threading paradigm below allows all of the processes to terminate cleanly
#  when the visualizer window is closed
# Because graphics elements cannot be legally modified outside of their thread, instead of killing
#  the graphics threads (visualizer and settings gui), we allow these threads to check for whether
#  the other threads are alive and if not, terminate their own thread
# There are likely many correct ways to accomplish the same behavior

all_threads_alive = True


def check_all_threads_alive(kill=False):
    # threadsafe
    # called periodically from pyglet and tkinter windows
    global all_threads_alive
    if kill:
        print("cross-thread kill")
        all_threads_alive = False
    return all_threads_alive


visualizer.set_thread_check_function(fn=check_all_threads_alive)


# Since we are later going to call visualizer.run_app from this file, which blocks while the visualizer
#  is open, any further code like the motion simulation should be in a separate thread
# More complications are induced by visualizer being its own thread as all of the
#  imports, definitions, and configuring above have to take place on that thread, but could be doable
class motion_sim_thread(threading.Thread):
    global all_threads_alive

    def run(self):
        i = 0
        while i < 200:
            i += 1
            time.sleep(0.25)
            visualizer.update_state_automatic(
                {"drone_1_x": abs((i % 20) - 10) + 20, "drone_1_y": i / 2}
            )  # Move the drone up 0.25 second and in a zigzag x

            d2x, d2y = interesting_spline(i % 81)
            visualizer.update_state_automatic(
                {"drone_2_x": d2x, "drone_2_y": d2y}
            )  # Move the drone along the spline

            if not all_threads_alive:
                break


class gui_thread(threading.Thread):
    def run(self):
        SettingsGUI.SettingsGUI(visualizer, check_all_threads_alive)


# Start the threads and run the "app" (visualizer pyglet window)
gui = gui_thread()
gui.start()
sim = motion_sim_thread()
sim.start()
visualizer.run_app()
gui.join()
sim.join()
sys.exit(0)
