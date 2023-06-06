"""
Visualizer of the project:
Real-Time Visualization for Reachability Analysis in Robotic Motion-Planning Systems
Anna Lucas
"""


import pyglet
import numpy as np
import cv2
import threading
import time
from visualizer_constants import *


mode_automatic = True

# Default color palette
# Array format for use in function sampling
color_transparent = np.array([0, 0, 0, 0], dtype=np.uint8)
color_positive = np.array([205, 231, 176, 255], dtype=np.uint8)  # Light green
color_levelset = np.array([152, 71, 62, 255], dtype=np.uint8)  # Brown
color_negative = np.array([236, 11, 67, 255], dtype=np.uint8)  # Bright red
# 4-tuple format for use in opencv primitives
color_spline = (107, 125, 125, 255)  # Gray
color_arrowgrid = (255, 255, 255, 255)  # White


### CANVAS AND COORDINATES

# The following all get updated
client_canvas_width = 150
client_canvas_height = 100
window_width = 1280
window_height = 720


def set_coord_system(w, h):
    """Sets the location of the upper right corner of the client canvas, defining units and scaling.

    Args:
      w: width in client's choice of units of the projection space
      h: height in client's choice of units of the projection space

    """
    global client_canvas_width
    global client_canvas_height
    client_canvas_width = w
    client_canvas_height = h


# Provide a list (of length 4) of RGBA int values for each color you want to update (between [0, 255])
def set_color_scheme(
    positive_fill_color=None,
    levelset_color=None,
    negative_fill_color=None,
    spline_color=None,
    arrowgrid_color=None,
):
    """Updates the render colors of certain objects.
    All arguments should be specified as a list of 4 rgba int values, like [100, 100, 100, 255]
    If an argument is not provided, no change is made to that color.

    Args:
        positive_fill_color
        levelset_color
        negative_fill_color
        spline_color
        arrowgrid_color
    """
    global color_positive, color_levelset, color_negative, color_spline, color_arrowgrid
    if positive_fill_color is not None:
        assert len(positive_fill_color) == 4
        color_positive = np.array(positive_fill_color, dtype=np.uint8)
    if levelset_color is not None:
        assert len(levelset_color) == 4
        color_levelset = np.array(levelset_color, dtype=np.uint8)
    if negative_fill_color is not None:
        assert len(negative_fill_color) == 4
        color_negative = np.array(negative_fill_color, dtype=np.uint8)
    if spline_color is not None:
        assert len(spline_color) == 4
        color_spline = (
            spline_color[0],
            spline_color[1],
            spline_color[2],
            spline_color[3],
        )
    if arrowgrid_color is not None:
        assert len(arrowgrid_color) == 4
        color_arrowgrid = (
            arrowgrid_color[0],
            arrowgrid_color[1],
            arrowgrid_color[2],
            arrowgrid_color[3],
        )


def coords2px(x, y):
    """Converts client coordinates to the canvas coordinate space (pixels)."""
    return int(x / client_canvas_width * window_width), int(
        y / client_canvas_height * window_height
    )


def px2coords(x, y):
    """Converts canvas coordinates (pixels) to client coordinate space."""
    return (
        x / window_width * client_canvas_width,
        y / window_height * client_canvas_height,
    )


### INTERFACING

# Dictionary with keys as names of the state vector items and values as values of the state vector items
state = {}
# Dictionary with keys as names of the state vector items and values True only if the state has not been
# updated since the last render refresh
state_valid = {}
# Queue of visibility toggle actions to perform, where each list item represents an action, and is
# the category of the item to be toggled followed by the nickname then desired toggle state:
# [ ["Image Objects", "Drone 1", True], ...  ]
visibility_toggles = []
# Maintains minimum and maximum slider values for each state vector item, stored as a tuple (min, max)
slider_overrides = {}


def set_initial_state_and_key(mode_auto: bool, state_dict: dict[str, float]):
    """Setup function defining the motion update mode and state vector items (and their initial values)
    Automatic mode indicates some kind of threaded client code is sending motion updates
    Manual (non automatic) mode indicates the sliders on the settings GUI will be sending motion updates
    """
    global mode_automatic
    global state
    mode_automatic = mode_auto
    state = state_dict
    for i in state_dict.keys():
        state_valid[i] = False
        slider_overrides[i] = (default_slider_min, default_slider_max)


def set_to_automatic_mode():
    global mode_automatic
    mode_automatic = True


def set_to_manual_mode():
    global mode_automatic
    mode_automatic = False


def print_state():
    for key in state.keys():
        print(key, ", ", state[key])


def get_state_item_names():
    return state.keys()


def get_state_with_values():
    return state


def get_slider_overrides():
    return slider_overrides


def add_slider_override(item, minmax):
    """expect minmax in the form (0, 100)
    update the slider values of the state vector entry with nickname item"""
    assert item in slider_overrides.keys()
    slider_overrides[item] = minmax


def add_image_object(
    nickname: str,
    filename_or_shape,
    x_state_link=None,
    y_state_link=None,
    theta_deg_state_link=None,
    init_x=0,
    init_y=0,
    init_theta_rad=0,
    scale_x=1,
    scale_y=1,
):
    """Wrapper function for adding a rendered image onto the canvas, that can move, scale, and rotate."""
    theta_deg = init_theta_rad / 3.1415 * 180
    render_images.append(
        RenderImage(
            nickname,
            filename_or_shape,
            x_state_link,
            y_state_link,
            theta_deg_state_link,
            init_x,
            init_y,
            theta_deg,
            scale_x,
            scale_y,
        )
    )


def add_function(
    nickname: str,
    func,
    signature: list[str],
    fill_pos: bool = False,
    fill_neg: bool = False,
    level_sets: list[float] = [0.0],
    custom_sample_res=None,
):
    """Wrapper function for adding an area function / surface function onto the canvas."""
    render_functions.append(
        RenderFunction(
            nickname, func, signature, fill_pos, fill_neg, level_sets, custom_sample_res
        )
    )


def add_spline(nickname: str, spline_parts: list[dict], width: float):
    """Wrapper function for adding a spline onto the canvas.
    group any different functions into the same spline object by passing in a list of segments
    """
    render_splines.append(RenderSpline(nickname, spline_parts, width))


def add_arrowgrid(
    nickname: str,
    func,
    signature: list[str],
    arrow_scale=2,
    sample_spacing=10,
    arrow_clarity=500,
):
    """Wrapper function for adding a rendered grid of arrows, intended to graph the gradient, onto the canvas."""
    render_grids.append(
        RenderArrowgrid(
            nickname=nickname,
            func=func,
            signature=signature,
            arrow_scale=arrow_scale,
            sample_spacing=sample_spacing,
            arrow_clarity=arrow_clarity,
        )
    )


def get_render_objects():
    """Get dictionary of all objects currently rendered."""
    objects = {}
    objects["Area Functions"] = [a_fn.nickname for a_fn in render_functions]
    objects["Splines"] = [spline.nickname for spline in render_splines]
    objects["Image Objects"] = [img.nickname for img in render_images]
    objects["Arrow Grids"] = [grid.nickname for grid in render_grids]
    return objects


### PYGLET OVERHEAD AND EVENTS

# In this paradigm, visualizer checks for the thread kill flag by itself in its own thread, so
# it can terminate cleanly.
check_thread_fn = "Not Set"
kill_background_threads_flag = False


def set_thread_check_function(fn):
    global check_thread_fn
    check_thread_fn = fn


window = pyglet.window.Window(window_width, window_height, resizable=True)
window.maximize()


@window.event
def on_draw():
    # Pyglet internal event loop
    do_updates()
    window.clear()
    batch.draw()


@window.event
def on_resize(width, height):
    global window_width
    global window_height
    window_width = width
    window_height = height
    for fn_sprite in [*render_functions, *render_splines, *render_grids]:
        s = fn_sprite.pyg_sprite
        s.update(
            scale_x=window_width / s.image.width, scale_y=window_height / s.image.height
        )


@window.event
def on_close():
    try:
        global kill_background_threads_flag
        kill_background_threads_flag = True
        check_thread_fn(True)  # kill if thread defined
        for thread in [
            obj.thread for obj in [*render_functions, *render_splines, *render_grids]
        ]:
            thread.join()
    except:
        print("No thread check function defined before closing window!")
    print("Visualizer Window closing")


def update_state_automatic(new_state: dict[str, float]):
    if not mode_automatic:
        return
    # call from outside app thread - don't call any sprites or render changes
    global state
    for updated_item in new_state.keys():
        state[updated_item] = new_state[updated_item]
        state_valid[updated_item] = False
    for item in [*render_functions, *render_splines, *render_grids]:
        item.start_update_thread(override_state=new_state)


def update_state_manual(new_state: dict[str, float]):
    if mode_automatic:
        return
    # call from outside app thread - don't call any sprites or render changes
    global state
    for updated_item in new_state.keys():
        state[updated_item] = new_state[updated_item]
        state_valid[updated_item] = False
    for item in [*render_functions, *render_splines, *render_grids]:
        item.start_update_thread(override_state=new_state)


def toggle_visibility(category, nickname, target_state):
    # Call from outside app thread - don't call any sprites or render changes
    global visibility_toggles
    visibility_toggles.append([category, nickname, target_state])


def do_updates():
    # call from inside app thread - can call sprites and render changes
    state_changes = {}
    for i in state.keys():
        if state_valid[i] is False:
            state_changes[i] = state[i]
            state_valid[i] = True
    for item in render_images:
        item.do_update(state_changes)

    for item in [*render_functions, *render_splines, *render_grids]:
        item.check_new_img_avail()

    while len(visibility_toggles) > 0:
        toggle = visibility_toggles.pop(0)
        selected = None
        # ["Area Functions", "Splines", "Image Objects", "Arrow Grids"]
        if toggle[0] == "Area Functions":
            for a_fn in render_functions:
                if a_fn.nickname == toggle[1]:
                    selected = a_fn
                    break
        if toggle[0] == "Splines":
            for spline in render_splines:
                if spline.nickname == toggle[1]:
                    selected = spline
                    break
        if toggle[0] == "Image Objects":
            for img in render_images:
                if img.nickname == toggle[1]:
                    selected = img
                    break
        if toggle[0] == "Arrow Grids":
            for grid in render_grids:
                if grid.nickname == toggle[1]:
                    selected = grid
                    break
        assert selected is not None
        selected.toggle_visibility(toggle[2])


def run_app():
    pyglet.app.run()


### RENDERING ###

batch = pyglet.graphics.Batch()

# To manually change the order of the render objects, add them to an Ordered Group and specify
# the order with the indices
functions_group = pyglet.graphics.Group(order=0)
splines_group = pyglet.graphics.Group(order=1)
images_group = pyglet.graphics.Group(order=3)
grids_group = pyglet.graphics.Group(order=2)

render_images = []
render_splines = []
render_functions = []
render_grids = []


def flatten_area_function_rgba_3d(arr_3d):
    # Quick workaround for needing to flatten across the other axis than default
    # Probably a better way to do this
    # OpenCV primitives used the default flattening but pyglet for some reason was
    # switching x and y when I set the texture with the default flattening method
    w, h, _ = arr_3d.shape
    flat = np.zeros(w * h * 4, dtype=np.uint8)
    for x in range(w):
        for y in range(h):
            for i in range(4):
                # switch order from default flattening
                flat[y * w * 4 + x * 4 + i] = arr_3d[x][y][i]
    return flat.tobytes()


class RenderImage:
    def __init__(
        self,
        nickname: str,
        filename_or_shape,
        x_state_link,
        y_state_link,
        theta_deg_state_link,
        init_x,
        init_y,
        init_theta_deg,
        init_scale_x,
        init_scale_y,
    ):
        self.nickname = nickname

        self.x_state_link = x_state_link
        self.y_state_link = y_state_link
        self.theta_deg_state_link = theta_deg_state_link

        self.visible = True

        # e.g. "Drone"
        # will display in GUI menu heading
        self.nickname = nickname

        self.listeners = []

        for link in [x_state_link, y_state_link, theta_deg_state_link]:
            if link is not None:
                self.listeners.append(link)

        self.static = False
        if len(self.listeners) == 0:
            self.static = True
        else:
            for listener in self.listeners:
                assert listener in state.keys()

        self.props = {"x": init_x, "y": init_y, "theta_deg": init_theta_deg}

        init_x, init_y = coords2px(init_x, init_y)

        self.init_scale_x = init_scale_x
        self.init_scale_y = init_scale_y

        self.img = pyglet.image.load(filename_or_shape)
        # Center the image anchor
        self.img.anchor_x = int(self.img.width / 2)
        self.img.anchor_y = int(self.img.height / 2)

        self.pyg_sprite = pyglet.sprite.Sprite(
            self.img, batch=batch, group=images_group, x=init_x, y=init_y
        )
        self.pyg_sprite.update(
            scale_x=init_scale_x, scale_y=init_scale_y, rotation=init_theta_deg
        )

    def toggle_visibility(self, target_vis_state):
        if target_vis_state == self.pyg_sprite.visible:
            return
        if target_vis_state == True:
            # make visible
            self.pyg_sprite.visible = True
        else:
            # make invisible
            self.pyg_sprite.visible = False

    def do_update(self, new_state_values: dict[str, float]):
        if self.static:
            return

        # new_state_values example: {"drone_x":32, "drone_y":79}
        updates_to_make = [i for i in new_state_values.keys() if i in self.listeners]

        if len(updates_to_make) == 0:
            return

        # figure out what the sprite state should be changed to
        for changed_statevec_item in updates_to_make:
            if changed_statevec_item == self.x_state_link:
                self.props["x"] = new_state_values[changed_statevec_item]
            if changed_statevec_item == self.y_state_link:
                self.props["y"] = new_state_values[changed_statevec_item]
            if changed_statevec_item == self.theta_deg_state_link:
                self.props["theta_deg"] = new_state_values[changed_statevec_item]

        # set sprite state
        new_x, new_y = coords2px(self.props["x"], self.props["y"])
        self.pyg_sprite.update(x=new_x, y=new_y, rotation=self.props["theta_deg"])


class RenderSpline:
    def __init__(self, nickname: str, spline_parts: list[dict], width):
        # e.g. "Drone"
        # will display in GUI menu heading
        self.nickname = nickname

        self.listeners = []
        self.static = False
        for i in spline_parts:
            self.listeners.extend(i["signature"])

        if len(self.listeners) == 0:
            self.static = True

        self.spline_parts = spline_parts

        self.width = int(
            width / client_canvas_width * default_spline_resolution
        )  # standardize inputs

        func_args = [
            [state[i] for i in spline_part["signature"]]
            for spline_part in self.spline_parts
        ]
        init_img = self.resample_image(func_args)

        img = pyglet.image.ImageData(
            default_spline_resolution,
            default_spline_resolution,
            "RGBA",
            init_img.flatten().tobytes(),
        )
        self.pyg_sprite = pyglet.sprite.Sprite(img, batch=batch, group=splines_group)

        self.thread = threading.Thread(target=self.thread_target)
        self.thread_override_state = {}
        self.thread_resample_flag = False
        self.new_img_avail = False
        self.new_img = None
        self.thread.start()

    def toggle_visibility(self, target_vis_state):
        if target_vis_state == self.pyg_sprite.visible:
            return
        if target_vis_state == True:
            # make visible
            self.pyg_sprite.visible = True
        else:
            # make invisible
            self.pyg_sprite.visible = False

    def start_update_thread(self, override_state: dict[str, float]):
        # run from outside thread
        if self.static:
            return
        if self.new_img_avail:
            return

        # do we need to resample?
        # new_state_values example: {"drone_x":32, "drone_y":79}
        updates_to_make = [i for i in override_state.keys() if i in self.listeners]
        if len(updates_to_make) == 0:
            return

        self.thread_override_state = override_state
        self.thread_resample_flag = True

    def thread_target(self):
        while not kill_background_threads_flag:
            if self.thread_resample_flag:
                self.thread_resample_flag = False
                func_args = []
                for spline_part in self.spline_parts:
                    # compute required arguments for each spline part function
                    sig = spline_part["signature"]
                    func_args_part = []
                    for i in sig:
                        if i in self.thread_override_state.keys():
                            func_args_part.append(self.thread_override_state[i])
                        else:
                            func_args_part.append(state[i])
                    func_args.append(func_args_part)

                # resample
                img = self.resample_image(func_args)
                # update sprite
                self.new_img = pyglet.image.ImageData(
                    default_spline_resolution,
                    default_spline_resolution,
                    "RGBA",
                    img.flatten().tobytes(),
                )
                self.new_img_avail = True
            else:
                time.sleep(background_thread_refresh_rate)

    def check_new_img_avail(self):
        if self.static or not self.new_img_avail:
            return
        self.pyg_sprite.image = self.new_img
        self.new_img = None
        self.new_img_avail = False

    def resample(self, func_args: list[list[float]]):
        points_x = []
        points_y = []
        ds = spline_sample_stepsize
        # self.segment_functions = spline_parts[:,0]
        # self.range_begin = spline_parts[:,1]
        # self.range_end = spline_parts[:,2]
        # self.function_args = spline_parts[:,3,:]
        for i in range(len(self.spline_parts)):
            p = self.spline_parts[i]
            for s in np.arange(p["min s"], p["max s"], ds):
                x, y = p["spline function"](s, *(func_args[i]))
                points_x.append(x)
                points_y.append(y)

        # take samples with signature args - end up with a list of points
        return points_x, points_y

    def resample_image(self, func_args):
        points_x, points_y = self.resample(func_args)
        # points are now in coords-space
        # want to map them evenly onto spline resolution-size space (pixels)
        for i in range(len(points_x)):
            points_x[i] = int(
                points_x[i] / client_canvas_width * default_spline_resolution
            )
            points_y[i] = int(
                points_y[i] / client_canvas_height * default_spline_resolution
            )
        img = np.zeros(
            (default_spline_resolution, default_spline_resolution, 4), dtype=np.uint8
        )

        for i in range(0, len(points_x) - 1):
            cv2.line(
                img,
                (points_x[i], points_y[i]),
                (points_x[i + 1], points_y[i + 1]),
                color=color_spline,
                thickness=self.width,
            )

        return img


class RenderFunction:
    def __init__(
        self,
        nickname: str,
        func,
        signature: list[str],
        fill_pos: bool,
        fill_neg: bool,
        level_sets: list[float],
        custom_sample_resolution,
    ):
        # e.g. "Drone"
        # will display in GUI menu heading
        self.nickname = nickname

        self.static = False
        if len(signature) == 0:
            self.static = True

        self.func = func
        self.signature = signature
        self.fill_pos = fill_pos
        self.fill_neg = fill_neg
        self.level_sets = level_sets
        self.fn_res = default_function_resolution
        if custom_sample_resolution is not None:
            self.fn_res = custom_sample_resolution

        init_samples = self.resample([state[i] for i in self.signature])
        init_img = pyglet.image.ImageData(
            self.fn_res,
            self.fn_res,
            "RGBA",
            flatten_area_function_rgba_3d(init_samples),
        )

        self.pyg_sprite = pyglet.sprite.Sprite(
            init_img, batch=batch, group=functions_group
        )
        # Scaling fits to window on resize

        self.thread = threading.Thread(target=self.thread_target)
        self.thread_state_override = {}
        self.thread_resample_flag = False
        self.new_img_avail = False
        self.new_img = None
        self.thread.start()

    def toggle_visibility(self, target_vis_state):
        if target_vis_state == self.pyg_sprite.visible:
            # print("Warning: Visibility toggle mismatch between visualizer and GUI on ",self.nickname,". Thread error? Resyncing.")
            pass
        if target_vis_state == True:
            # make visible
            self.pyg_sprite.visible = True
        else:
            # make invisible
            self.pyg_sprite.visible = False

    def start_update_thread(self, override_state: dict[str, float]):
        # run from outside thread
        if self.static:
            return
        if self.new_img_avail:
            return

        # do we need to resample?
        # new_state_values example: {"drone_x":32, "drone_y":79}
        updates_to_make = [i for i in override_state.keys() if i in self.signature]
        if len(updates_to_make) == 0:
            return

        self.thread_state_override = override_state
        self.thread_resample_flag = True

    def thread_target(self):
        while not kill_background_threads_flag:
            if self.thread_resample_flag:
                self.thread_resample_flag = False
                func_args = []
                for i in range(len(self.signature)):
                    s = self.signature[i]
                    if s in self.thread_state_override.keys():
                        func_args.append(self.thread_state_override[s])
                    else:
                        func_args.append(state[s])
                # resample
                img = self.resample(func_args)
                # update sprite
                self.new_img = pyglet.image.ImageData(
                    self.fn_res, self.fn_res, "RGBA", flatten_area_function_rgba_3d(img)
                )
                self.new_img_avail = True
            else:
                time.sleep(background_thread_refresh_rate)

    def check_new_img_avail(self):
        if self.static or not self.new_img_avail:
            return
        self.pyg_sprite.image = self.new_img
        self.new_img = None
        self.new_img_avail = False

    def resample(self, func_args):
        samples = np.zeros((self.fn_res, self.fn_res, 4), dtype=np.uint8)

        # take samples with signature args - end up with an array of values
        for i in range(self.fn_res):
            for j in range(self.fn_res):
                # call f(x, y, other arguments...) and save into samples array
                sample = self.func(
                    *px2coords(
                        i * window_width / self.fn_res, j * window_height / self.fn_res
                    ),
                    *func_args
                )

                levelset_pixel = False
                # can make this smoother later with level set utility
                for ls in self.level_sets:
                    if (
                        levelset_epsilon + sample > ls
                        and -1 * levelset_epsilon + sample < ls
                    ):
                        levelset_pixel = True
                        continue

                if levelset_pixel:
                    samples[i, j] = color_levelset
                else:
                    if sample > 0 and self.fill_pos:
                        samples[i, j] = color_positive
                    elif sample < 0 and self.fill_neg:
                        samples[i, j] = color_negative
                    else:
                        samples[i, j] = color_transparent
        return samples


class RenderArrowgrid:
    def __init__(
        self,
        nickname: str,
        func,
        signature: list[str],
        arrow_scale,
        sample_spacing,
        arrow_clarity,
    ):
        # e.g. "Drone"
        # will display in GUI menu heading
        self.nickname = nickname

        self.static = False
        if len(signature) == 0:
            self.static = True

        self.func = func
        self.signature = signature

        self.arrow_clarity = arrow_clarity
        self.sample_spacing = sample_spacing
        # self.width = int(width/client_canvas_width*arrow_clarity) # Decided against customizable width
        # as these arrows don't look that great anyway

        self.arrow_scale = arrow_scale / client_canvas_width * arrow_clarity

        init_img = self.resample_image(state[i] for i in signature)

        img = pyglet.image.ImageData(
            self.arrow_clarity, self.arrow_clarity, "RGBA", init_img.flatten().tobytes()
        )
        self.pyg_sprite = pyglet.sprite.Sprite(img, batch=batch, group=grids_group)

        self.thread = threading.Thread(target=self.thread_target)
        self.thread_override_state = {}
        self.thread_resample_flag = False
        self.new_img_avail = False
        self.new_img = None
        self.thread.start()

    def toggle_visibility(self, target_vis_state):
        if target_vis_state == self.pyg_sprite.visible:
            return
        if target_vis_state == True:
            # make visible
            self.pyg_sprite.visible = True
        else:
            # make invisible
            self.pyg_sprite.visible = False

    def start_update_thread(self, override_state: dict[str, float]):
        # run from outside thread
        if self.static:
            return
        if self.new_img_avail:
            return

        # do we need to resample?
        # new_state_values example: {"drone_x":32, "drone_y":79}
        updates_to_make = [i for i in override_state.keys() if i in self.signature]
        if len(updates_to_make) == 0:
            return

        self.thread_override_state = override_state
        self.thread_resample_flag = True

    def thread_target(self):
        while not kill_background_threads_flag:
            if self.thread_resample_flag:
                self.thread_resample_flag = False
                func_args = []
                for i in range(len(self.signature)):
                    s = self.signature[i]
                    if s in self.thread_state_override.keys():
                        func_args.append(self.thread_state_override[s])
                    else:
                        func_args.append(state[s])
                # resample
                img = self.resample_image(func_args)
                # update sprite
                self.new_img = pyglet.image.ImageData(
                    self.arrow_clarity,
                    self.arrow_clarity,
                    "RGBA",
                    img.flatten().tobytes(),
                )
                self.new_img_avail = True
            else:
                time.sleep(background_thread_refresh_rate)

    def check_new_img_avail(self):
        if self.static or not self.new_img_avail:
            return
        self.pyg_sprite.image = self.new_img
        self.new_img = None
        self.new_img_avail = False

    def resample_image(self, func_args):
        num_arrows_x = int(client_canvas_width / self.sample_spacing)
        num_arrows_y = int(client_canvas_height / self.sample_spacing)
        samples = np.zeros((num_arrows_x, num_arrows_y, 2))

        # take samples with signature args - end up with an array of values
        for i in range(num_arrows_x):
            for j in range(num_arrows_y):
                # call f(x, y, other arguments...) and save into samples array
                sample = self.func(
                    i * self.sample_spacing, j * self.sample_spacing, *func_args
                )
                samples[i][j][0] = sample[0]
                samples[i][j][1] = sample[1]

        samples *= self.arrow_scale  # now in pixel space, hopefully

        img = np.zeros((self.arrow_clarity, self.arrow_clarity, 4), dtype=np.uint8)

        for i in range(num_arrows_x):
            for j in range(num_arrows_y):
                px_arrow_x = int(
                    i * self.sample_spacing / client_canvas_width * self.arrow_clarity
                )
                px_arrow_y = int(
                    j * self.sample_spacing / client_canvas_height * self.arrow_clarity
                )

                # sample[0] is dx, sample[1] is dy
                # draw the arrow
                dx = int(samples[i][j][0])
                dy = int(samples[i][j][1])

                cv2.arrowedLine(
                    img,
                    (px_arrow_x, px_arrow_y),
                    (px_arrow_x + dx, px_arrow_y + dy),
                    color=color_arrowgrid,
                    thickness=arrow_thickness,
                    tipLength=arrow_tip_length,
                )
        return img
