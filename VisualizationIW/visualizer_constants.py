"""
Configurable constants and settings for the project:
Real-Time Visualization for Reachability Analysis in Robotic Motion-Planning Systems
Anna Lucas
"""

# Settings GUI main use
expand_triangle_unicode = "\u25bc"
collapse_triangle_unicode = "\u25b2"

invisible_icon_unicode = "\u25ef"
visible_icon_unicode = "\u2b24"

gui_refresh_rate_ms = 500

window_minsize_x_px = 300
window_minsize_y_px = 100
default_gui_width = 400


# visualizer.py main use

default_function_resolution = 200  # Takes 200 by 200 samples by default
default_spline_resolution = 300  # Draws onto 300 by 300 image before upscaling
levelset_epsilon = 2  # If function value is within +/- epsilon of the level set indicated, it is filled with level set color

spline_sample_stepsize = 1

default_slider_min = 0
default_slider_max = 100

arrow_thickness = 1
arrow_tip_length = 0.15

background_thread_refresh_rate = 0.2
