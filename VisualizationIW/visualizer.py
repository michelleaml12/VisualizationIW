from os import stat
import pyglet
import numpy as np
import cv2


### CONSTANTS

fn_res = 100 # Takes 100 by 100 samples
spline_res = 300 # Draws onto 300 by 300 image before upscaling
mode_automatic = False # Currently a constant - not sure if this functionality will go within-class

#Random color palette I generated - there will be more options
colors_rgba = np.array([
    [0,0,0,0],
    [138,163,153,255],
    [125,132,178,255],
    [143,166,203,255],
    [219,244,167,255],
    [213,249,222,255]    
    ],dtype=np.uint8)
colors_rgb = np.array([
    [0,0,0],
    [138,163,153],
    [125,132,178],
    [143,166,203],
    [219,244,167],
    [213,249,222]    
    ],dtype=np.uint8)

### CANVAS AND COORDINATES

# The following all get updated
client_canvas_width=150
client_canvas_height=100
window_width = 1280
window_height = 720

def set_coord_system(w,h):
    global client_canvas_width
    global client_canvas_height
    client_canvas_width = w
    client_canvas_height = h

def coords2px(x, y):
    return x/client_canvas_width*window_width, y/client_canvas_height*window_height

def px2coords(x, y):
    return x/window_width*client_canvas_width, y/window_height*client_canvas_height


### INTERFACING

state = {}
state_valid = {}

def set_state_and_key(mode_auto:bool, state_dict:dict[str,float]):
    global mode_automatic
    global state
    mode_automatic = mode_auto
    state=state_dict
    for i in state_dict:
        state_valid[i] = False

def print_state():
    for key in state.keys():
        print(key, ", ", state[key])

def add_object(nickname:str, filename_or_shape, listeners=None, init_x=0, init_y=0, init_theta_rad=0, scale_x=1, scale_y=1):
    x_px, y_px = coords2px(init_x, init_y)
    theta_deg = init_theta_rad/3.1415*180
    render_objects.append(RenderObject(nickname, filename_or_shape, listeners, x_px, y_px, theta_deg, scale_x, scale_y))

def add_function(nickname:str, func, signature:list[str], fill_pos:bool=False, fill_neg:bool=False, level_sets:list[float]=[0.0]):
    render_functions.append(RenderFunction(nickname, func, signature, fill_pos, fill_neg, level_sets))

def add_spline(nickname:str, spline_parts:list[list], width:float):
    #group any different functions into the same spline object by passing in a list of segments
    #segment defined as [generic function to query, beginning s, ending s, function signature]
    render_splines.append(RenderSpline(nickname, spline_parts, width))



### PYGLET OVERHEAD AND EVENTS

window = pyglet.window.Window(window_width, window_height, resizable=True)
window.maximize()

@window.event
def on_draw():
    do_updates()
    window.clear()
    batch.draw()

@window.event
def on_resize(width, height):
    global window_width
    global window_height
    window_width = width
    window_height = height
    print(f'The window was resized to {width},{height}')
    for fn_sprite in [* render_functions, *render_splines]:
        s = fn_sprite.pyg_sprite
        s.update(scale_x = window_width / s.image.width, scale_y = window_height / s.image.height)

def update_state(new_state:dict[str, float]):
    #call from outside app thread - don't call any sprites or render changes
    global state
    for updated_item in new_state.keys():
        state[updated_item] = new_state[updated_item]
        state_valid[updated_item] = False

def do_updates():
    #call from inside app thread - can call sprites and render changes
    state_changes = {}
    for i in state.keys():
        if state_valid[i] == False:
            state_changes[i]=state[i]
            state_valid[i] = True
    for item in [*render_objects, *render_functions, *render_splines]:
        item.do_update(state_changes)

def run_app():
    pyglet.app.run()


### RENDERING ###

batch = pyglet.graphics.Batch()
functions_group = pyglet.graphics.Group(order=0)
splines_group = pyglet.graphics.Group(order=1)
objects_group = pyglet.graphics.Group(order=2)

render_objects = []
render_splines = []
render_functions = []

class RenderObject():
    def __init__(self, nickname:str, filename_or_shape, listeners, init_x, init_y, init_theta_deg, init_scale_x, init_scale_y):
        self.nickname = nickname
               
        #Take listeners as a dictionary where key is the name of the
        #   state vector item, and value is a list of the properties that listen 
        #probably will just be one item per list in practice
        #example listeners: {"drone_x":["x"]} updates the x state of this sprite 
        #   with any change to the drone_x state item
        self.listeners = listeners
        
        #e.g. "Drone"
        #will display in GUI menu heading
        self.nickname= nickname

        self.static = False
        assert(listeners==None or type(listeners)==type({"str":["str"]}))
        if listeners==None or len(listeners.keys())==0:
            self.static = True

        img = pyglet.image.load(filename_or_shape)
        self.pyg_sprite = pyglet.sprite.Sprite(img, batch=batch, group=objects_group, x=init_x, y=init_y)
        self.pyg_sprite.update(scale_x=init_scale_x, scale_y=init_scale_y, rotation=init_theta_deg)

        #Take initial props as the properties of pyg_sprite
        self.props = {"x":self.pyg_sprite.x, 
                             "y":self.pyg_sprite.y, 
                             "theta_deg":self.pyg_sprite.rotation}
        if not self.static:
            #check linkage
            for state_item in listeners.keys():
                assert(state_item in state.keys())
                for link in listeners[state_item]:
                    assert(link in self.props.keys())

    def do_update(self, new_state_values:dict[str, float]):
        #print("Render Object Update Loop:",self.nickname)
        if self.static: return

        #new_state_values example: {"drone_x":32, "drone_y":79}
        updates_to_make = [i for i in new_state_values.keys() if i in self.listeners.keys()]

        if len(updates_to_make)==0: return

        #figure out what the sprite state should be changed to
        for changed_statevec_item in updates_to_make:
            for prop_to_change in self.listeners[changed_statevec_item]:
                #e.g. self.state["x"] = new_state_values["drone_x"]
                self.props[prop_to_change]=new_state_values[changed_statevec_item]

        #set sprite state
        self.pyg_sprite.update(x=self.props["x"], y=self.props["y"], rotation=self.props["theta_deg"])


class RenderSpline():
    def __init__(self, nickname:str, spline_parts:list[list], width):   
       
        #e.g. "Drone"
        #will display in GUI menu heading
        self.nickname= nickname

        self.listeners = []
        self.static = False
        for i in spline_parts:
            self.listeners.extend(i[3])

        if len(self.listeners)==0:
            self.static = True

        self.spline_parts = spline_parts

        self.width = int(width / client_canvas_width * spline_res) # standardize inputs

        init_img = self.resample_image()
        img = pyglet.image.ImageData(spline_res, spline_res, "RGBA", init_img.flatten().tobytes())
        self.pyg_sprite = pyglet.sprite.Sprite(img, batch=batch, group=splines_group)   

    def do_update(self, new_state_values:dict[str, float]):
        #print("Render Spline Update Loop:",self.nickname)
        if self.static: return

        #do we need to resample?
        #new_state_values example: {"drone_x":32, "drone_y":79}
        updates_to_make = [i for i in new_state_values.keys() if i in self.listeners]
        if len(updates_to_make)==0: return

        #resample
        img = self.resample_image()
        #update sprite
        self.pyg_sprite.image = pyglet.image.ImageData(spline_res, spline_res, "RGBA", img.flatten().tobytes() )

    def resample(self):
        print("Render Spline Resample Loop:",self.nickname)
        points_x=[]
        points_y=[]
        ds=2
        #self.segment_functions = spline_parts[:,0]
        #self.range_begin = spline_parts[:,1]
        #self.range_end = spline_parts[:,2]
        #self.function_args = spline_parts[:,3,:]
        for p in self.spline_parts:
            for s in np.arange(p[1], p[2], ds):
                x,y = p[0](s, *[state[i] for i in p[3]])
                points_x.append(x)
                points_y.append(y)

        #take samples with signature args - end up with a list of points
        return points_x, points_y

    def resample_image(self):
        points_x, points_y = self.resample()
        #points are now in coords-space
        #want to map them evenly onto spline resolution-size space (pixels)
        for i in range(len(points_x)):
            points_x[i] = int(points_x[i]/client_canvas_width*spline_res)
            points_y[i] = int(points_y[i]/client_canvas_height*spline_res)
        img = np.zeros((spline_res, spline_res, 4), dtype=np.uint8)

        for i in range(0, len(points_x)-1):
            cv2.line(img, (points_x[i], points_y[i]), (points_x[i+1],points_y[i+1]), color=(255,255,255,255), thickness=self.width)

        return img


class RenderFunction():
    def __init__(self, nickname:str, func, signature:list[str], fill_pos:bool=False, fill_neg:bool=False, level_sets:list[float]=[0.0]):

        #e.g. "Drone"
        #will display in GUI menu heading
        self.nickname= nickname

        self.static = False
        if len(signature)==0:
            self.static = True

        self.func=func 
        self.signature = signature
        self.fill_pos = fill_pos
        self.fill_neg = fill_neg
        self.level_sets = level_sets

        init_samples = self.resample()
        flat = np.ndarray.flatten(init_samples)
        init_img = pyglet.image.ImageData(fn_res, fn_res, "RGBA", flat.tobytes())
        
        self.pyg_sprite = pyglet.sprite.Sprite(init_img, batch=batch, group=functions_group)
        #Scaling fits to window on resize

    def do_update(self, new_state_values:dict[str, float]):
        #print("Render Function Update Loop:",self.nickname)
        if self.static: return

        #do we need to resample?
        #new_state_values example: {"drone_x":32, "drone_y":79}
        updates_to_make = [i for i in new_state_values.keys() if i in self.signature]
        if len(updates_to_make)==0: return

        #resample
        samples = self.resample()

        #update sprite
        self.pyg_sprite.image = pyglet.image.ImageData(fn_res, fn_res, "RGBA", samples.flatten().tobytes() )


    def resample(self):
        print("Render Function Resample Loop:",self.nickname)
        #say sample is image size?
        samples = np.zeros((fn_res,fn_res,4),dtype=np.uint8)

        #take samples with signature args - end up with an array of values
        for i in range(fn_res):
            for j in range(fn_res):
                #call f(x, y, other arguments...) and save into samples array
                extra_args = [state[i] for i in self.signature]
                sample=self.func(*px2coords(i*window_width/fn_res, j*window_height/fn_res) , * extra_args)

                #Decide tag - 0=transparent 1=fill_pos 2=level set 3=fill_neg
                tag = 0

                #fill level sets with tag 2
                #can make this smoother later with level set utility
                for ls in self.level_sets:
                    if 2+sample>ls and -2+sample<ls:
                        tag=2
                        continue

                if tag==0: #if not in a level set pixel, see if it needs pos or neg fill, or neither
                    if sample>0 and self.fill_pos:
                        tag=1
                    elif sample<0 and self.fill_neg:
                        tag=3
                
                #convert mask array to image texture formatting and datatype
                samples[i,j] = colors_rgba[tag]
        return samples














