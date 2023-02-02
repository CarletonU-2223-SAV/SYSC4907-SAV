#!/usr/bin/env python3
import tkinter as tk
import networkx as nx
import MapSerializer as MS
from PIL import ImageTk, Image
from Models import RoadSegmentType, MapModel, Lane, Point


CITY = 1
NH = 2
map_model: MapModel = MapModel()

# Create a new graph
G = nx.Graph()

# Adding nodes (Point) and edges to the graph, representing the roads on the map
# Currently only NH graph added
startpoint = Point(645, 573, RoadSegmentType.TURN)
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
point30 = Point(575, 573, RoadSegmentType.STRAIGHT)
point31 = Point(644, 510, RoadSegmentType.STRAIGHT)
point32 = Point(705, 572, RoadSegmentType.STRAIGHT)
point33 = Point(645, 640, RoadSegmentType.STRAIGHT)

edges = [
    (startpoint, point30, {"weight": 0.5}),
    (startpoint, point31, {"weight": 0.5}),
    (startpoint, point32, {"weight": 0.5}),
    (startpoint, point33, {"weight": 0.5}),
    (point30, point1, {"weight": 5.5}),
    (point31, point2, {"weight": 3.5}),
    (point32, point3, {"weight": 5.5}),
    (point33, point24, {"weight": 4.5}),
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
             point29,
             point30,
             point31,
             point32,
             point33]

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
    draw_line(path)

    #add path to map_model for navigation
    lane: Lane = Lane()
    lane.set_lane(path)
    map_model.add_path(lane)

# draws path onto canvas
def draw_line(path):
    canvas.delete("line")
    for i in range(len(path)):
        if i < len(path) - 1:
            canvas.create_line(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y, fill="red", width=3, tags="line")

# Handle load from file
def handle_load():
    global map_model
    map_model = MS.load_from_file()
    draw_line(map_model.get_path())

# Updates the canvas image
def update_canvas(img_choice: int):
    if img_choice == CITY:
        canvas.itemconfig(canvas_container, image=img[0])
    elif img_choice == NH:
        canvas.itemconfig(canvas_container, image=img[1])

# For debug
def print_converted_path():
    path_points = map_model.convert_path(0)

    # Format that print-out like the coords.txt file that "simple_path" can interpret
    for x, y, _ in path_points:
        print(f'{x}:{y}')

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

# Root window
window = tk.Tk()
window.title("Map creator")
init_menu_bar()

# Canvas
img = [ImageTk.PhotoImage(Image.open('AirSim_graphs/City_Graph.png')),
       ImageTk.PhotoImage(Image.open('AirSim_graphs/NH_Graph.png'))]
h = img[0].height()
w = img[0].width()

canvas = tk.Canvas(window, width=w, height=h)
canvas_container = canvas.create_image(0, 0, image=img[1], anchor='nw')
canvas.pack(side="left", fill="both", expand="yes")
# Canvas event handlers
canvas.bind("<Button-1>", handle_canvas_m1)
window.bind_all('<KeyPress-p>', print_converted_path)

window.mainloop()
