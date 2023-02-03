#!/usr/bin/env python3
import tkinter as tk
import networkx as nx
import MapSerializer as MS
from PIL import ImageTk, Image
from Models import RoadSegmentType, MapModel, Lane, Point


CITY = 1
NH = 2
current_image = NH
map_model: MapModel = MapModel()

# Create a graph
NH_G = nx.Graph()
C_G = nx.Graph()

# Adding nodes (Point) and edges to the graph, representing the roads on the map
NH_startpoint = Point(645, 573, RoadSegmentType.TURN)
NH_point1 = Point(114, 573, RoadSegmentType.INTERSECTION)
NH_point2 = Point(641, 252, RoadSegmentType.INTERSECTION)
NH_point3 = Point(1171, 573, RoadSegmentType.INTERSECTION)
NH_point4 = Point(91, 600, RoadSegmentType.STRAIGHT)
NH_point5 = Point(91, 573, RoadSegmentType.INTERSECTION)
NH_point6 = Point(91, 549, RoadSegmentType.STRAIGHT)
NH_point7 = Point(91, 987, RoadSegmentType.STRAIGHT)
NH_point8 = Point(91, 252, RoadSegmentType.STRAIGHT)
NH_point9 = Point(91, 226, RoadSegmentType.INTERSECTION)
NH_point10 = Point(91, 202, RoadSegmentType.STRAIGHT)
NH_point11 = Point(91, 47, RoadSegmentType.INTERSECTION)
NH_point12 = Point(91, 27, RoadSegmentType.INTERSECTION)
NH_point13 = Point(111, 25, RoadSegmentType.INTERSECTION)
NH_point14 = Point(616, 19, RoadSegmentType.STRAIGHT)
NH_point15 = Point(642, 19, RoadSegmentType.INTERSECTION)
NH_point16 = Point(671, 19, RoadSegmentType.STRAIGHT)
NH_point17 = Point(1167,25, RoadSegmentType.INTERSECTION)
NH_point18 = Point(1192, 27, RoadSegmentType.INTERSECTION)
NH_point19 = Point(1194, 51, RoadSegmentType.INTERSECTION)
NH_point20 = Point(1196, 548, RoadSegmentType.STRAIGHT)
NH_point21 = Point(1196, 573, RoadSegmentType.INTERSECTION)
NH_point22 = Point(1196, 598, RoadSegmentType.STRAIGHT)
NH_point23 = Point(1196, 989, RoadSegmentType.STRAIGHT)
NH_point24 = Point(645, 989, RoadSegmentType.STRAIGHT)
NH_point25 = Point(118, 228, RoadSegmentType.INTERSECTION)
NH_point26 = Point(614, 228, RoadSegmentType.STRAIGHT)
NH_point27 = Point(641, 228, RoadSegmentType.INTERSECTION)
NH_point28 = Point(641, 200, RoadSegmentType.STRAIGHT)
NH_point29 = Point(641, 43, RoadSegmentType.INTERSECTION)
NH_point30 = Point(575, 573, RoadSegmentType.STRAIGHT)
NH_point31 = Point(644, 510, RoadSegmentType.STRAIGHT)
NH_point32 = Point(705, 572, RoadSegmentType.STRAIGHT)
NH_point33 = Point(645, 640, RoadSegmentType.STRAIGHT)

C_startpoint = Point(614, 589, RoadSegmentType.TURN)
C_point1 = Point(632, 564, RoadSegmentType.STRAIGHT)
C_point2 = Point(754, 564, RoadSegmentType.INTERSECTION)
C_point3 = Point(769, 564, RoadSegmentType.INTERSECTION)
C_point4 = Point(770, 579, RoadSegmentType.STRAIGHT)
C_point5 = Point(773, 699, RoadSegmentType.STRAIGHT)
C_point6 = Point(763, 748, RoadSegmentType.STRAIGHT)
C_point7 = Point(747, 759, RoadSegmentType.STRAIGHT)
C_point8 = Point(687, 769, RoadSegmentType.STRAIGHT)
C_point9 = Point(591, 768, RoadSegmentType.INTERSECTION)
C_point10 = Point(650, 531, RoadSegmentType.STRAIGHT) ##
C_point11 = Point(651, 502, RoadSegmentType.STRAIGHT)
C_point12 = Point(632, 473, RoadSegmentType.STRAIGHT)
C_point13 = Point(607, 453, RoadSegmentType.STRAIGHT)
C_point14 = Point(590, 444, RoadSegmentType.INTERSECTION)
C_point15 = Point(564, 443, RoadSegmentType.STRAIGHT)
C_point16 = Point(532, 448, RoadSegmentType.STRAIGHT)
C_point17 = Point(502, 465, RoadSegmentType.STRAIGHT)
C_point18 = Point(478, 489, RoadSegmentType.STRAIGHT)
C_point19 = Point(464, 516, RoadSegmentType.INTERSECTION)
C_point20 = Point(460, 530, RoadSegmentType.INTERSECTION)
C_point21 = Point(463, 561, RoadSegmentType.STRAIGHT)
C_point22 = Point(473, 587, RoadSegmentType.STRAIGHT)
C_point23 = Point(487, 609, RoadSegmentType.STRAIGHT)
C_point24 = Point(507, 626, RoadSegmentType.STRAIGHT)
C_point25 = Point(535, 640, RoadSegmentType.INTERSECTION)
C_point26 = Point(547, 646, RoadSegmentType.INTERSECTION)
C_point27 = Point(549, 783, RoadSegmentType.STRAIGHT)
C_point28 = Point(550, 984, RoadSegmentType.STRAIGHT)
C_point29 = Point(1019, 564, RoadSegmentType.STRAIGHT) ##
C_point30 = Point(1045, 555, RoadSegmentType.INTERSECTION)
C_point31 = Point(1046, 495, RoadSegmentType.STRAIGHT)
C_point32 = Point(1042, 433, RoadSegmentType.STRAIGHT)
C_point33 = Point(1032, 370, RoadSegmentType.STRAIGHT)
C_point34 = Point(1014, 303, RoadSegmentType.STRAIGHT)
C_point35 = Point(993, 246, RoadSegmentType.INTERSECTION)
C_point36 = Point(985, 230, RoadSegmentType.INTERSECTION)
C_point37 = Point(1000,224 , RoadSegmentType.STRAIGHT)
C_point38 = Point(1041, 201, RoadSegmentType.STRAIGHT)
C_point39 = Point(1082, 187, RoadSegmentType.STRAIGHT)
C_point40 = Point(940, 182, RoadSegmentType.STRAIGHT)
C_point41 = Point(898, 145, RoadSegmentType.STRAIGHT)
C_point42 = Point(843, 114, RoadSegmentType.STRAIGHT)
C_point43 = Point(787, 94, RoadSegmentType.STRAIGHT)
C_point44 = Point(727, 80, RoadSegmentType.STRAIGHT)
C_point45 = Point(662, 72, RoadSegmentType.STRAIGHT)
C_point46 = Point(596, 70, RoadSegmentType.INTERSECTION)
C_point47 = Point(583, 70, RoadSegmentType.INTERSECTION)
C_point48 = Point(583, 56, RoadSegmentType.STRAIGHT)
C_point49 = Point(583, 4, RoadSegmentType.STRAIGHT)
C_point50 = Point(584, 427, RoadSegmentType.STRAIGHT) ##
C_point51 = Point(583, 364, RoadSegmentType.INTERSECTION)
C_point52 = Point(564, 344, RoadSegmentType.STRAIGHT)
C_point53 = Point(499, 345, RoadSegmentType.STRAIGHT)
C_point54 = Point(437, 342, RoadSegmentType.STRAIGHT)
C_point55 = Point(393, 334, RoadSegmentType.STRAIGHT)
C_point56 = Point(347, 318, RoadSegmentType.STRAIGHT)
C_point57 = Point(295, 290, RoadSegmentType.STRAIGHT)
C_point58 = Point(259, 263, RoadSegmentType.STRAIGHT)
C_point59 = Point(249, 243, RoadSegmentType.INTERSECTION)
C_point60 = Point(242, 221, RoadSegmentType.INTERSECTION)
C_point61 = Point(106, 220, RoadSegmentType.STRAIGHT)
C_point62 = Point(241, 10, RoadSegmentType.STRAIGHT)
C_point63 = Point(583, 322, RoadSegmentType.STRAIGHT)
C_point64 = Point(445, 530, RoadSegmentType.STRAIGHT) ##
C_point65 = Point(201, 530, RoadSegmentType.STRAIGHT)
C_point66 = Point(57, 530, RoadSegmentType.STRAIGHT)
C_point67 = Point(471, 602, RoadSegmentType.INTERSECTION) ##
C_point68 = Point(456, 616, RoadSegmentType.STRAIGHT)
C_point69 = Point(302, 738, RoadSegmentType.STRAIGHT)
C_point70 = Point(283, 758, RoadSegmentType.STRAIGHT)
C_point71 = Point(269, 787, RoadSegmentType.STRAIGHT)
C_point72 = Point(261, 821, RoadSegmentType.STRAIGHT)
C_point73 = Point(255, 984, RoadSegmentType.STRAIGHT)

NH_edges = [
    (NH_startpoint, NH_point30, {"weight": 0.5}),
    (NH_startpoint, NH_point31, {"weight": 0.5}),
    (NH_startpoint, NH_point32, {"weight": 0.5}),
    (NH_startpoint, NH_point33, {"weight": 0.5}),
    (NH_point30, NH_point1, {"weight": 5.5}),
    (NH_point31, NH_point2, {"weight": 3.5}),
    (NH_point32, NH_point3, {"weight": 5.5}),
    (NH_point33, NH_point24, {"weight": 4.5}),
    (NH_point1, NH_point5, {"weight": 0.5}),
    (NH_point5, NH_point4, {"weight": 0.5}),
    (NH_point5, NH_point6, {"weight": 0.5}),
    (NH_point4, NH_point7, {"weight": 5}),
    (NH_point6, NH_point8, {"weight": 5}),
    (NH_point8, NH_point9, {"weight": 0.5}),
    (NH_point9, NH_point10, {"weight": 0.5}),
    (NH_point10, NH_point11, {"weight": 2}),
    (NH_point11, NH_point12, {"weight": 0.5}),
    (NH_point12, NH_point13, {"weight": 0.5}),
    (NH_point13, NH_point14, {"weight": 6}),
    (NH_point14, NH_point15, {"weight": 0.5}),
    (NH_point15, NH_point16, {"weight": 0.5}),
    (NH_point16, NH_point17, {"weight": 6}),
    (NH_point17, NH_point18, {"weight": 0.5}),
    (NH_point18, NH_point19, {"weight": 0.5}),
    (NH_point19, NH_point20, {"weight": 6}),
    (NH_point20, NH_point21, {"weight": 0.5}),
    (NH_point21, NH_point22, {"weight": 0.5}),
    (NH_point22, NH_point23, {"weight": 5}),
    (NH_point3, NH_point21, {"weight": 0.5}),
    (NH_point9, NH_point25, {"weight": 0.5}),
    (NH_point25, NH_point26, {"weight": 6}),
    (NH_point26, NH_point27, {"weight": 0.5}),
    (NH_point27, NH_point28, {"weight": 0.5}),
    (NH_point27, NH_point2, {"weight": 0.5}),
    (NH_point28, NH_point29, {"weight": 2}),
    (NH_point29, NH_point15, {"weight": 0.5}),
]

C_edges = [
(C_startpoint, C_point1, {"weight": 1}),
    (C_point1, C_point2, {"weight": 4}),
    (C_point2, C_point3, {"weight": 0.5}),
    (C_point3, C_point4, {"weight": 0.5}),
    (C_point4, C_point5, {"weight": 4}),
    (C_point5, C_point6, {"weight": 2}),
    (C_point6, C_point7, {"weight": 0.5}),
    (C_point7, C_point8, {"weight": 2}),
    (C_point8, C_point9, {"weight": 3}),
    (C_point1, C_point10, {"weight": 3}),
    (C_point10, C_point11, {"weight": 1}),
    (C_point11,	C_point12, {"weight": 1}),
    (C_point12,	C_point13, {"weight": 1}),
    (C_point13,	C_point14, {"weight": 0.5}),
    (C_point14,	C_point15, {"weight": 1}),
    (C_point15,	C_point16, {"weight": 1}),
    (C_point16,	C_point17, {"weight": 1}),
    (C_point17,	C_point18, {"weight": 1}),
    (C_point18,	C_point19, {"weight": 1}),
    (C_point19,	C_point20, {"weight": 0.5}),
    (C_point20,	C_point21, {"weight": 1}),
    (C_point21,	C_point22, {"weight": 1}),
    (C_point22,	C_point23, {"weight": 1}),
    (C_point23,	C_point24, {"weight": 1}),
    (C_point24,	C_point25, {"weight": 1}),
    (C_point25,	C_point26, {"weight": 0.5}),
    (C_point26, C_point27, {"weight": 4}),
    (C_point9, C_point27, {"weight": 2}),
    (C_point27, C_point28, {"weight": 6}),
    (C_point3, C_point29, {"weight": 8}),
    (C_point29, C_point30, {"weight": 1}),
    (C_point30, C_point31, {"weight": 2}),
    (C_point31, C_point32, {"weight": 2}),
    (C_point32, C_point33, {"weight": 2}),
    (C_point33, C_point34, {"weight": 2}),
    (C_point34,	C_point35, {"weight": 2}),
    (C_point35,	C_point36, {"weight": 0.5}),
    (C_point36,	C_point37, {"weight": 0.5}),
    (C_point37,	C_point38, {"weight": 1}),
    (C_point38,	C_point39, {"weight": 1}),
    (C_point36, C_point40, {"weight": 2}),
    (C_point40,	C_point41, {"weight": 2}),
    (C_point41,	C_point42, {"weight": 2}),
    (C_point42,	C_point43, {"weight": 2}),
    (C_point43,	C_point44, {"weight": 2}),
    (C_point44,	C_point45, {"weight": 2}),
    (C_point45,	C_point46, {"weight": 2}),
    (C_point46,	C_point47, {"weight": 0.5}),
    (C_point47,	C_point48, {"weight": 0.5}),
    (C_point48,	C_point49, {"weight": 2}),
    (C_point14, C_point50, {"weight": 0.5}),
    (C_point50,	C_point51, {"weight": 2}),
    (C_point51,	C_point52, {"weight": 1}),
    (C_point52,	C_point53, {"weight": 2}),
    (C_point53,	C_point54, {"weight": 2}),
    (C_point54,	C_point55, {"weight": 1}),
    (C_point55,	C_point56, {"weight": 2}),
    (C_point56,	C_point57, {"weight": 2}),
    (C_point57,	C_point58, {"weight": 2}),
    (C_point58,	C_point59, {"weight": 1}),
    (C_point59,	C_point60, {"weight": 1}),
    (C_point60, C_point61, {"weight": 4}),
    (C_point60, C_point62, {"weight": 6.5}),
    (C_point51, C_point63, {"weight": 1}),
    (C_point47, C_point63, {"weight": 8}),
    (C_point20, C_point64, {"weight": 0.5}),
    (C_point64, C_point65, {"weight": 7.5}),
    (C_point65, C_point66, {"weight": 4.5}),
    (C_point22, C_point67, {"weight": 0.5}),
    (C_point67,	C_point68, {"weight": 0.5}),
    (C_point68,	C_point69, {"weight": 6}),
    (C_point69,	C_point70, {"weight": 1}),
    (C_point70,	C_point71, {"weight": 1}),
    (C_point71,	C_point72, {"weight": 1}),
    (C_point72,	C_point73, {"weight": 5})
]

NH_G.add_edges_from(NH_edges)
C_G.add_edges_from(C_edges)

#used for iteration
NH_pointList = [NH_startpoint,
             NH_point1,
             NH_point2,
             NH_point3,
             NH_point4,
             NH_point5,
             NH_point6,
             NH_point7,
             NH_point8,
             NH_point9,
             NH_point10,
             NH_point11,
             NH_point12,
             NH_point13,
             NH_point14,
             NH_point15,
             NH_point16,
             NH_point17,
             NH_point18,
             NH_point19,
             NH_point20,
             NH_point21,
             NH_point22,
             NH_point23,
             NH_point24,
             NH_point25,
             NH_point26,
             NH_point27,
             NH_point28,
             NH_point29,
             NH_point30,
             NH_point31,
             NH_point32,
             NH_point33
]

C_pointList = [C_startpoint,
            C_point1,
            C_point2,
            C_point3,
            C_point4,
            C_point5,
            C_point6,
            C_point7,
            C_point8,
            C_point9,
            C_point10,
            C_point11,
            C_point12,
            C_point13,
            C_point14,
            C_point15,
            C_point16,
            C_point17,
            C_point18,
            C_point19,
            C_point20,
            C_point21,
            C_point22,
            C_point23,
            C_point24,
            C_point25,
            C_point26,
            C_point27,
            C_point28,
            C_point29,
            C_point30,
            C_point31,
            C_point32,
            C_point33,
            C_point34,
            C_point35,
            C_point36,
            C_point37,
            C_point38,
            C_point39,
            C_point40,
            C_point41,
            C_point42,
            C_point43,
            C_point44,
            C_point45,
            C_point46,
            C_point47,
            C_point48,
            C_point49,
            C_point50,
            C_point51,
            C_point52,
            C_point53,
            C_point54,
            C_point55,
            C_point56,
            C_point57,
            C_point58,
            C_point59,
            C_point60,
            C_point61,
            C_point62,
            C_point63,
            C_point64,
            C_point65,
            C_point66,
            C_point67,
            C_point68,
            C_point69,
            C_point70,
            C_point71,
            C_point72,
            C_point73
]

# Find the shortest path using dijkstra
def find_shortest_path(graph, start_point_id, end_point_id):
    return nx.dijkstra_path(graph, start_point_id, end_point_id)

# create path from clicking on canvas
def handle_canvas_m1(event):
    global map_model, current_image
    x, y = event.x, event.y

    #delete line and path if already created
    canvas.delete("line")
    if not(map_model.empty()):
        map_model.delete_path(0)

    #choosing node to travel too
    point_found = False
    endpoint = -1
    if current_image == CITY:
        for point in C_pointList:
            if abs(x - point.x) < 10 and abs(y - point.y) < 10:
                endpoint = point
                point_found = True
                break
    else:
        for point in NH_pointList:
            if abs(x - point.x) < 10 and abs(y - point.y) < 10:
                endpoint = point
                point_found = True
                break

    if point_found:
        if current_image == CITY:
            path = find_shortest_path(C_G, C_startpoint, endpoint)
        else:
            path = find_shortest_path(NH_G, NH_startpoint, endpoint)
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
    global current_image
    current_image = img_choice
    canvas.delete("line")
    if img_choice == CITY:
        canvas.itemconfig(canvas_container, image=img[0])
    elif img_choice == NH:
        canvas.itemconfig(canvas_container, image=img[1])

# For debug
def print_converted_path():
    global map_model
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
