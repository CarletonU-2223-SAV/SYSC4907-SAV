#!/usr/bin/env python3
import tkinter as tk
import networkx as nx
from enum import Enum
import MapSerializer as MS
from PIL import ImageTk, Image
from Models import RoadSegment, RoadSegmentType, MapModel, Lane, Connection, Path, Point
import os
from typing import Union, Tuple, List


class Mode(Enum):
    POINT = 0
    PATH = 1


op_mode: Mode = Mode.POINT


# Set the operation mode
def set_mode(mode: Mode):
    global op_mode
    op_mode = mode


# id of selected element (segment/lane/path)
selected_seg_id: int = -1
selected_lane_id: int = -1
selected_path_id: int = -1
map_model: MapModel = MapModel()


# Handle load from file
def handle_load():
    global map_model
    map_model = MS.load_from_file()
    # Update render
    rerender_points()
    rerender_road_segments_panel()
    rerender_paths_panel()


CITY = 1
NH = 2


# Updates the canvas image
def update_canvas(img_choice: int):
    if img_choice == CITY:
        canvas.itemconfig(canvas_container, image=img[0])
    elif img_choice == NH:
        canvas.itemconfig(canvas_container, image=img[1])


# Create a new graph
G = nx.Graph()

# Adding nodes (Point) and edges to the graph, representing the roads on the map
# Currently only NH graph added
startpoint = Point(645, 573, RoadSegmentType.STRAIGHT)
point1 = Point(114, 573, RoadSegmentType.INTERSECTION)
point2 = Point(641, 252, RoadSegmentType.INTERSECTION)
point3 = Point(1171, 573, RoadSegmentType.INTERSECTION)
point4 = Point(91, 600, RoadSegmentType.STRAIGHT)
point5 = Point(91, 573, RoadSegmentType.INTERSECTION)
point6 = Point(91, 549, RoadSegmentType.STRAIGHT)
point7 = Point(91, 987, RoadSegmentType.STRAIGHT)
point8 = Point(91, 252, RoadSegmentType.STRAIGHT)
point9 = Point(91, 226, RoadSegmentType.INTERSECTION)
point10 = Point(91, 202, RoadSegmentType.STRAIGHT)
point11 = Point(91, 47, RoadSegmentType.INTERSECTION)
point12 = Point(91, 27, RoadSegmentType.INTERSECTION)
point13 = Point(111, 25, RoadSegmentType.INTERSECTION)
point14 = Point(616, 19, RoadSegmentType.STRAIGHT)
point15 = Point(642, 19, RoadSegmentType.INTERSECTION)
point16 = Point(671, 19, RoadSegmentType.STRAIGHT)
point17 = Point(1167,25, RoadSegmentType.INTERSECTION)
point18 = Point(1192, 27, RoadSegmentType.INTERSECTION)
point19 = Point(1194, 51, RoadSegmentType.INTERSECTION)
point20 = Point(1196, 548, RoadSegmentType.STRAIGHT)
point21 = Point(1196, 573, RoadSegmentType.INTERSECTION)
point22 = Point(1196, 598, RoadSegmentType.STRAIGHT)
point23 = Point(1196, 989, RoadSegmentType.STRAIGHT)
point24 = Point(645, 989, RoadSegmentType.STRAIGHT)
point25 = Point(118, 228, RoadSegmentType.INTERSECTION)
point26 = Point(614, 228, RoadSegmentType.STRAIGHT)
point27 = Point(641, 228, RoadSegmentType.INTERSECTION)
point28 = Point(641, 200, RoadSegmentType.STRAIGHT)
point29 = Point(641, 43, RoadSegmentType.INTERSECTION)

edges = [
    (startpoint, point1, {"weight": 6}),
    (startpoint, point2, {"weight": 4}),
    (startpoint, point3, {"weight": 6}),
    (startpoint, point24, {"weight": 5}),
    (point1, point5, {"weight": 0.5}),
    (point5, point4, {"weight": 0.5}),
    (point5, point6, {"weight": 0.5}),
    (point4, point7, {"weight": 5}),
    (point6, point8, {"weight": 5}),
    (point8, point9, {"weight": 0.5}),
    (point9, point10, {"weight": 0.5}),
    (point10, point11, {"weight": 2}),
    (point11, point12, {"weight": 0.5}),
    (point12, point13, {"weight": 0.5}),
    (point13, point14, {"weight": 6}),
    (point14, point15, {"weight": 0.5}),
    (point15, point16, {"weight": 0.5}),
    (point16, point17, {"weight": 6}),
    (point17, point18, {"weight": 0.5}),
    (point18, point19, {"weight": 0.5}),
    (point19, point20, {"weight": 6}),
    (point20, point21, {"weight": 0.5}),
    (point21, point22, {"weight": 0.5}),
    (point22, point23, {"weight": 5}),
    (point3, point21, {"weight": 0.5}),
    (point9, point25, {"weight": 0.5}),
    (point25, point26, {"weight": 6}),
    (point26, point27, {"weight": 0.5}),
    (point27, point28, {"weight": 0.5}),
    (point27, point2, {"weight": 0.5}),
    (point28, point29, {"weight": 2}),
    (point29, point15, {"weight": 0.5}),
]
G.add_edges_from(edges)

#used for iteration
pointList = [startpoint,
             point1,
             point2,
             point3,
             point4,
             point5,
             point6,
             point7,
             point8,
             point9,
             point10,
             point11,
             point12,
             point13,
             point14,
             point15,
             point16,
             point17,
             point18,
             point19,
             point20,
             point21,
             point22,
             point23,
             point24,
             point25,
             point26,
             point27,
             point28,
             point29]

# Find the shortest path using dijkstra
def find_shortest_path(start_point_id, end_point_id):
    return nx.dijkstra_path(G, start_point_id, end_point_id)

# create path from clicking on canvas
def handle_canvas_m1(event):
    x, y = event.x, event.y

    #delete line and path if already created
    canvas.delete("line")
    if not(map_model.empty()):
        map_model.delete_path(0)

    #choosing node to travel too
    point_found = False
    for point in pointList:
        if abs(x - point.x) < 10 and abs(y - point.y) < 10:
            endpoint = point
            point_found = True
            break

    if point_found:
        path = find_shortest_path(startpoint, endpoint)
    else:
        #if no node is clicked
        return

    #create line on canvas showing generated path
    for i in range(len(path)):
        if i < len(path)-1:
            canvas.create_line(path[i].x, path[i].y, path[i+1].x, path[i+1].y, fill="red", width= 3, tags="line")

    #add path to map_model for navigation
    lane: Lane = Lane()
    lane.set_lane(path)
    map_model.add_path(lane)

# Delete point
def handle_canvas_m3(event):
    overlap_size: int = 3
    x, y = event.x, event.y
    # list of object ids in box
    points: tuple = canvas.find_overlapping(x - overlap_size, y - overlap_size, x + overlap_size, y + overlap_size)
    # Clicked a point
    if len(points) > 0 and points[-1] != 1:
        # id of point to remove
        point_id = points[-1]
        # get road_segment id
        seg_id, lane_id = get_point_data(point_id)
        # remove from map model
        map_model.road_segments[seg_id].remove_point(lane_id, point_id)

        # remove from canvas
        canvas.delete(point_id)
        rerender_road_segments_panel()


# Get point info from the tag data
def get_point_data(point_id) -> tuple:
    tags: List[str] = canvas.gettags(point_id)
    seg_id_str = tags[0]
    lane_id_str = tags[1]
    seg_id = int(seg_id_str[1:])
    lane_id = int(lane_id_str[1:])
    return seg_id, lane_id


# Size is radius, returns id of point
def draw_point(x, y, tags: Union[str, Tuple], size: int = 2) -> int:
    # include tag of segment_id
    return canvas.create_oval(x - size, y - size, x + size, y + size, fill='red', outline='yellow', tags=tags)


# clear existing highlights
def clear_point_highlights():
    for pid in canvas.find_all():
        # Ignore the image on the canvas
        if pid != 1:
            canvas.itemconfigure(pid, outline='')


# Highlight the points that belong to the given segment id
def highlight_seg_points(segment_id: int):
    clear_point_highlights()

    points = canvas.find_withtag(f's{segment_id}')
    # highlight points with matching tag
    for index, pid in enumerate(points):
        if index == 0:
            canvas.itemconfigure(pid, outline='blue')
        else:
            canvas.itemconfigure(pid, outline='yellow')


# Highlight points that belong to the specified path
def highlight_path(path_id=selected_path_id):
    clear_point_highlights()
    sel_path: Path = map_model.paths[path_id]
    # print(f'{sel_path}')
    # Empty path
    if sel_path.empty():
        pass

    for con, p in sel_path.connections:
        canvas.itemconfigure(p.pid, outline='yellow')


# Some wrapper classes for basic GUI elements
class RoadSegmentLabel(tk.Label):
    def __init__(self, segment_id: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.segment_id = segment_id


class LaneLabel(tk.Label):
    def __init__(self, lane_id: int, segment_id: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.lane_id = lane_id
        self.segment_id = segment_id


class PathLabel(tk.Label):
    def __init__(self, path_id: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.path_id = path_id


class RoadSegmentFrame(tk.Frame):
    def __init__(self, segment_id: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.segment_id = segment_id


def create_road(seg_type: RoadSegmentType):
    map_model.add_road_segment(seg_type)
    rerender_road_segments_panel()


# Create a new lane in the selected RoadSegment
def create_lane():
    if selected_seg_id == -1:
        pass
    map_model.road_segments[selected_seg_id].add_lane()
    rerender_road_segments_panel()


def create_path():
    map_model.add_path()
    rerender_paths_panel()


def rerender_road_segments_panel():
    clear_frame(seg_frame)
    # Render each RoadSegment in the map
    for key, value in map_model.road_segments.items():
        # different style for selected segment
        relief = 'sunken' if selected_seg_id == key else 'flat'
        bg = '#D3D3D3' if selected_seg_id == key else '#fff'
        # create a label for each road segment
        fr = RoadSegmentFrame(segment_id=value.segment_id, master=seg_frame)
        # Title
        label = RoadSegmentLabel(segment_id=value.segment_id, master=fr, bg=bg,
                                 text=value.seg_type, anchor='w', justify='left', relief=relief)
        label.bind('<Button-1>', handle_select_segment)
        label.pack(side='top', fill='x')
        # Render each Lane in the RoadSegment
        for index, lane in enumerate(value.lanes):
            lane_label = LaneLabel(lane_id=index, segment_id=value.segment_id, master=fr, text=lane, wraplength=300)
            lane_label.bind('<Button-1>', handle_select_lane)
            lane_label.pack(side='top', fill='x')

        fr.pack(side='top', fill='x')


def rerender_paths_panel():
    clear_frame(path_frame)

    if not map_model.paths:
        label = tk.Label(text='No paths', master=path_frame, wraplength=300)
        label.pack(side='top', fill='x')
        return

    for index, path in enumerate(map_model.paths):
        label = PathLabel(path_id=index, text=path, master=path_frame, wraplength=300)
        label.bind('<Button-1>', handle_select_path)
        label.pack(side='top', fill='x')


# Rerender the points in all road_segments. Called on map load from file.
def rerender_points():
    # Delete all points
    for pid in canvas.find_all():
        if pid != 1:
            canvas.delete(pid)

    # Redraw them on the canvas
    for sid, road_segment in map_model.road_segments.items():
        # Update the point ids in the lanes
        for index, lane in enumerate(road_segment.lanes):
            for p in lane.points:
                # Get and assign new ids
                new_pid = draw_point(p.x, p.y, tags=(f's{sid}', f'l{index}'))
                p.update_pid(new_pid)


# Update the path pids when the map pids change. Call after rerender_points()
def update_path_pids():
    # TODO: implement
    pass


def clear_seg_panel_highlights():
    for fr in seg_frame.winfo_children():
        for label in fr.winfo_children():
            label['relief'] = 'flat'
            label['bg'] = '#fff'


def clear_path_panel_highlights():
    for label in path_frame.winfo_children():
        label['relief'] = 'flat'
        label['bg'] = '#fff'


def handle_select_segment(event):
    global selected_seg_id
    global selected_lane_id
    global selected_path_id

    clicked = event.widget
    clear_seg_panel_highlights()
    clear_path_panel_highlights()

    clicked['relief'] = 'sunken'
    clicked['bg'] = '#D3D3D3'

    selected_seg_id = clicked.segment_id
    selected_lane_id = -1
    selected_path_id = -1
    # Update point highlights
    highlight_seg_points(selected_seg_id)
    set_mode(Mode.POINT)


def handle_select_lane(event):
    global selected_seg_id
    global selected_lane_id
    global selected_path_id
    # Handle the label highlighting
    clicked: LaneLabel = event.widget
    parent: RoadSegmentFrame = clicked.master

    clear_seg_panel_highlights()
    clear_path_panel_highlights()

    clicked['relief'] = 'sunken'
    clicked['bg'] = '#D3D3D3'
    # update selected lane id
    selected_seg_id = parent.segment_id
    selected_lane_id = clicked.lane_id
    selected_path_id = -1

    # Handle the point highlighting
    highlight_lane_points()
    set_mode(Mode.POINT)


def handle_select_path(event):
    global selected_seg_id
    global selected_lane_id
    global selected_path_id

    clicked = event.widget
    clear_seg_panel_highlights()
    clear_path_panel_highlights()

    clicked['relief'] = 'sunken'
    clicked['bg'] = '#D3D3D3'

    selected_seg_id = -1
    selected_lane_id = -1
    selected_path_id = clicked.path_id
    set_mode(Mode.PATH)

    clear_point_highlights()
    highlight_path()


def highlight_lane_points():
    clear_point_highlights()
    points = canvas.find_withtag(f's{selected_seg_id}')
    # Filter for points in the lane
    lane_points = [e for e in points if canvas.gettags(e)[1] == f'l{selected_lane_id}']
    # highlight points with matching tag
    for index, pid in enumerate(lane_points):
        # Highlight
        if index == 0:
            canvas.itemconfigure(pid, outline='blue')
        else:
            canvas.itemconfigure(pid, outline='yellow')


# TODO: delete lane?


# Removes all contents from the frame
def clear_frame(fr):
    for widget in fr.winfo_children():
        widget.destroy()


# Not really used for much yet
def handle_undo(event):
    if selected_path_id != -1:
        map_model.paths[selected_path_id].remove_last_point()
        rerender_paths_panel()
        highlight_path()


def init_menu_bar():
    # Menu bar
    menubar = tk.Menu(window)
    window.config(menu=menubar)

    # File menu
    file_menu = tk.Menu(menubar, tearoff=False)
    file_menu.add_command(
        label='save coords',
        command=lambda: MS.save_to_file(map_model)
    )
    file_menu.add_command(
        label='load coords',
        command=handle_load
    )
    menubar.add_cascade(
        label='File',
        menu=file_menu
    )

    # Maps Menu
    maps_menu = tk.Menu(menubar, tearoff=False)
    maps_menu.add_command(
        label='City',
        command=lambda: update_canvas(CITY)
    )
    maps_menu.add_command(
        label='NH',
        command=lambda: update_canvas(NH)
    )
    menubar.add_cascade(
        label='Maps',
        menu=maps_menu
    )

    # Create menu
    create_menu = tk.Menu(menubar, tearoff=False)
    create_menu.add_command(
        label='straight road',
        command=lambda: create_road(RoadSegmentType.STRAIGHT)
    )
    create_menu.add_command(
        label='curved road',
        command=lambda: create_road(RoadSegmentType.TURN)
    )
    create_menu.add_command(
        label='intersection',
        command=lambda: create_road(RoadSegmentType.INTERSECTION)
    )
    create_menu.add_command(
        label='lane',
        command=create_lane
    )
    create_menu.add_command(
        label='path',
        command=create_path
    )
    menubar.add_cascade(
        label='Create',
        menu=create_menu
    )

    # Mode menu
    mode_menu = tk.Menu(menubar, tearoff=False)
    mode_menu.add_command(
        label='point',
        command=lambda: set_mode(Mode.POINT)
    )
    mode_menu.add_command(
        label='link',
        command=lambda: set_mode(Mode.LINK)
    )
    mode_menu.add_command(
        label='path',
        command=lambda: set_mode(Mode.PATH)
    )
    menubar.add_cascade(
        label='Mode',
        menu=mode_menu
    )


# For debug
def print_converted_path(event):
    if selected_path_id == -1:
        pass
    path_points = map_model.convert_path(selected_path_id)

    # Format that print-out like the coords.txt file that "simple_path" can interpret
    for x, y, _ in path_points:
        print(f'{x}:{y}')


# Root window
window = tk.Tk()
window.title("Map creator")
init_menu_bar()

# Canvas
img = [ImageTk.PhotoImage(Image.open('AirSim_graphs/City_Graph.png')),
       ImageTk.PhotoImage(Image.open('AirSim_graphs/NH_Graph.png'))]
h = img[0].height()
w = img[0].width()

# Road segments panel
seg_frame = tk.Frame(master=window)
label_a = tk.Label(master=seg_frame, text='no segments yet')
label_a.pack()

# Paths panel
path_frame = tk.Frame(master=window)
label_b = tk.Label(master=path_frame, text='no paths yet')
label_b.pack()

canvas = tk.Canvas(window, width=w, height=h)
canvas_container = canvas.create_image(0, 0, image=img[1], anchor='nw')
canvas.pack(side="left", fill="both", expand="yes")
# Canvas event handlers
canvas.bind("<Button-1>", handle_canvas_m1)
canvas.bind("<Button-3>", handle_canvas_m3)
window.bind_all('<Control-z>', handle_undo)
window.bind_all('<KeyPress-p>', print_converted_path)

# No idea why this is the order and not the reverse...
path_frame.pack(side="right", fill="both", expand="yes")
seg_frame.pack(side="right", fill="both", expand="yes")

window.mainloop()
