"""
Settings GUI for the visualizer of the project:
Real-Time Visualization for Reachability Analysis in Robotic Motion-Planning Systems
Anna Lucas
"""

import tkinter as tk
from tkinter import ttk
from visualizer_constants import *

tk.Tcl().eval("set tcl_platform(threaded)")


default_rows = {
    "Area Functions": ["Not Set", "Not Set 2"],
    "Image Objects": ["Not Set"],
}


class SettingsGUI:
    def __init__(
        self,
        visualizer,
        check_thread_fn,
    ):
        self.check_thread_fn = check_thread_fn
        self.window = tk.Tk()
        self.window.minsize(window_minsize_x_px, window_minsize_y_px)
        self.frame = tk.Frame(self.window)

        # need to ensure consistent formatting to be able to display the right kinds of settings
        # e.g. image objects will have an xy state and rotation while the others will not
        self.categories = ["Area Functions", "Splines", "Image Objects", "Arrow Grids"]
        # might want a category later for state vector items

        self.visualizer = visualizer

        self.state_vector_cache = visualizer.get_state_with_values()
        self.slider_minmax = visualizer.get_slider_overrides()

        self.rows = visualizer.get_render_objects()
        self.populate_rows()

        self.window.after(gui_refresh_rate_ms, self.task)
        self.window.mainloop()

    def task(self):
        if not self.check_thread_fn():
            self.window.destroy()
        self.state_vector_cache = self.visualizer.get_state_with_values()
        # set these into the labels?
        subframe = None
        try:
            subframe = self.panel.children["!frame2"]
        except:
            print("Close GUI detected")
            return
        mode_manual = self.panel.manual_mode.get()
        for child in subframe.children.keys():
            state_slider = subframe.children[child]
            for nickname in self.state_vector_cache.keys():
                if nickname == state_slider.nickname:
                    state_slider.set_my_label(self.state_vector_cache[nickname])
                    if not mode_manual:
                        state_slider.set_my_slider(self.state_vector_cache[nickname])

        self.window.after(gui_refresh_rate_ms, self.task)  # repeat this soon

    def populate_rows(self):
        self.frame.destroy()

        # expect rows as dictionary with keys in self.categories
        for category in self.rows.keys():
            assert category in self.categories
            category_list = []
            for row in self.rows[category]:
                category_list.append(row)  # do other stuff here later
            frame = self.CollapsibleFrame(
                self.window, category, category_list, self.visualizer
            )
            frame.pack(side="top", fill="x")

        self.panel = self.StateVectorPanel(
            self.window,
            self.state_vector_cache,
            self.slider_minmax,
            False,
            self.visualizer,
        )
        self.panel.pack(side="top", fill="x")

    # Collapsible frames adapted from CustomFrame class in https://stackoverflow.com/a/48430121
    class CollapsibleFrame(tk.Frame):
        def __init__(self, parent, category, item_nicknames, visualizer):
            tk.Frame.__init__(self, parent)
            header = tk.Frame(self)
            self.visualizer = visualizer
            self.sub_frame = tk.Frame(
                self, relief="sunken", width=default_gui_width, height=22, borderwidth=1
            )
            header.pack(side="top", fill="x")
            self.sub_frame.pack(side="top", fill="both", expand=True)

            # handles toggle state
            self.expanded = tk.IntVar(value=0)

            self.label = tk.Label(
                header, text=category + " (" + str(len(item_nicknames)) + ")"
            )
            self.expand_toggle_btn = ttk.Checkbutton(
                header,
                width=2,
                text=expand_triangle_unicode,
                variable=self.expanded,
                style="Toolbutton",
                command=self.toggle_expansion,
            )
            self.expand_toggle_btn.pack(side="left")

            self.label.pack(side="left")

            self.sub_frame.pack(side="top", fill="both", expand=True)

            for item_nickname in item_nicknames:
                self.VisibilityToggleEntry(
                    self.sub_frame, category, item_nickname, visualizer
                ).pack(side="top", fill="x")

            # this sets the initial state
            self.toggle_expansion(False)

        def toggle_expansion(self, show=None):
            show = self.expanded.get() if show is None else show
            if show:
                self.sub_frame.pack(side="top", fill="x", expand=True)
                self.expand_toggle_btn.configure(text=collapse_triangle_unicode)
            else:
                self.sub_frame.forget()
                self.expand_toggle_btn.configure(text=expand_triangle_unicode)

        # To put inside collapsible frames we want each entry to have a name label and a toggle checkbox
        # entry should probably be its own frame
        # when creating, return a pointer to the visibility checkbox to be saved in an array?
        # also save a visibility state
        # send the new visibility state to the visualizer

        class VisibilityToggleEntry(tk.Frame):
            def __init__(self, parent, category, nickname, visualizer):
                tk.Frame.__init__(self, parent)
                header = tk.Frame(self)
                header.pack(side="top", fill="x")
                # handles toggle state
                self.sprite_visible = tk.IntVar(value=1)

                self.category = category
                self.nickname = nickname
                self.vis_toggle_fn = visualizer.toggle_visibility

                self.label = tk.Label(header, text=nickname)
                self.vis_toggle_btn = ttk.Checkbutton(
                    header,
                    width=2,
                    text=expand_triangle_unicode,
                    variable=self.sprite_visible,
                    style="Toolbutton",
                    command=self.toggle_vis,
                )
                # self.entry = tk.Entry(header, width=11)

                self.vis_toggle_btn.pack(side="left", padx=10)

                self.label.pack(side="left")

                # this sets the initial state
                self.toggle_vis(True)

            def toggle_vis(self, show=None):
                show = self.sprite_visible.get() if show is None else show
                if show:
                    self.vis_toggle_btn.configure(text=visible_icon_unicode)
                    self.vis_toggle_fn(self.category, self.nickname, True)
                else:
                    self.vis_toggle_btn.configure(text=invisible_icon_unicode)
                    self.vis_toggle_fn(self.category, self.nickname, False)

    # Collapsible frames adapted from CustomFrame class in https://stackoverflow.com/a/48430121
    class StateVectorPanel(tk.Frame):
        def __init__(
            self, parent, state_vec_cache, slider_minmax, starting_mode, visualizer
        ):
            tk.Frame.__init__(self, parent)
            header = tk.Frame(self)
            self.visualizer = visualizer
            self.sub_frame = tk.Frame(
                self, relief="sunken", width=default_gui_width, height=22, borderwidth=1
            )
            header.pack(side="top", fill="x")
            self.sub_frame.pack(side="top", fill="both", expand=True)

            # handles toggle state
            self.expanded = tk.IntVar(value=0)

            self.manual_mode = tk.IntVar(value=0)

            self.label = tk.Label(header, text="State Vector")
            self.expand_toggle_btn = ttk.Checkbutton(
                header,
                width=2,
                text=expand_triangle_unicode,
                variable=self.expanded,
                style="Toolbutton",
                command=self.toggle_expansion,
            )
            self.mode_toggle_btn = ttk.Checkbutton(
                header,
                width=40,
                text="Automatic Mode (Click to change)",
                variable=self.manual_mode,
                style="Toolbutton",
                command=self.toggle_state_mode,
            )
            self.expand_toggle_btn.pack(side="left")
            self.label.pack(side="left")
            self.mode_toggle_btn.pack(side="right", padx=20)

            self.sub_frame.pack(side="top", fill="both", expand=True)

            for item_nickname in state_vec_cache.keys():
                self.StateVectorSlider(
                    self.sub_frame,
                    item_nickname,
                    visualizer,
                    slider_minmax[item_nickname],
                    init_val=state_vec_cache[item_nickname],
                ).pack(side="top", fill="x")

            # this sets the initial state
            self.toggle_expansion(False)

            self.toggle_state_mode(starting_mode)

        def toggle_expansion(self, show=None):
            show = self.expanded.get() if show is None else show
            if show:
                self.sub_frame.pack(side="top", fill="x", expand=True)
                self.expand_toggle_btn.configure(text=collapse_triangle_unicode)
            else:
                self.sub_frame.forget()
                self.expand_toggle_btn.configure(text=expand_triangle_unicode)

        def toggle_state_mode(self, manual=None):
            manual = self.manual_mode.get() if manual is None else manual
            if manual:
                self.mode_toggle_btn.configure(text="Manual Mode (Click to change)")
                self.visualizer.set_to_manual_mode()
            else:
                self.mode_toggle_btn.configure(text="Automatic Mode (Click to change)")
                self.visualizer.set_to_automatic_mode()

        class StateVectorSlider(tk.Frame):
            def __init__(
                self, parent, nickname, visualizer, minmax=(0, 100), init_val=10
            ):
                tk.Frame.__init__(self, parent)
                header = tk.Frame(self)
                header.pack(side="top", fill="x")
                self.visualizer = visualizer

                # handles toggle state
                self.nickname = nickname

                self.name_label = tk.Label(header, text=nickname)
                self.current_state_label = tk.Label(header, text=str(init_val))

                self.name_label.pack(side="left", padx=10)
                self.current_state_label.pack(side="left", padx=10)

                self.slider_min = minmax[0]
                self.slider_max = minmax[1]

                self.slider = tk.Scale(
                    header,
                    from_=self.slider_min,
                    to=self.slider_max,
                    orient="horizontal",
                    command=self.val_command,
                )
                self.slider.set(init_val)
                self.slider.pack(side="right", padx=10)

            def val_command(self, val):
                print("val cmd", val, self.nickname)
                self.visualizer.update_state_manual({self.nickname: float(int(val))})

                # this sets the initial state
                # self.toggle_vis(True)

            def set_my_slider(self, val):
                if val > self.slider_max:
                    val = self.slider_max
                if val < self.slider_min:
                    val = self.slider_min
                self.slider.set(val)

            def set_my_label(self, val):
                self.current_state_label.configure(text=str(val))
