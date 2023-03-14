#!/usr/bin/env python3
import math
import os
import tkinter as tk
from typing import List, Tuple

import networkx as nx
import MapSerializer as MS
from PIL import ImageTk, Image
from Models import RoadSegmentType, MapModel, Lane, Point
from sympy import Point as Point2, Segment

# can optimize these metrics to inc/dec points placed among path
STRAIGHT_INTERVAL = 30
CURVE_POINTS = 8

CITY = 1
NH = 2
current_image = NH
dest_node: Point = Point(None, None, RoadSegmentType.STRAIGHT)
map_model: MapModel = MapModel()

# Create a graph
NH_G = nx.Graph()
C_G = nx.Graph()

# Adding nodes (Point) and edges to the graph, representing the roads on the map
NH_startpoint = Point(645, 573, RoadSegmentType.TURN)
NH_intersection_1 = Point(89, 573, RoadSegmentType.INTERSECTION)
NH_intersection_2 = Point(89, 229, RoadSegmentType.INTERSECTION)
NH_intersection_3 = Point(89, 22, RoadSegmentType.INTERSECTION)
NH_intersection_4 = Point(645, 229, RoadSegmentType.INTERSECTION)
NH_intersection_5 = Point(645, 22, RoadSegmentType.INTERSECTION)
NH_intersection_6 = Point(1195, 573, RoadSegmentType.INTERSECTION)
NH_intersection_7 = Point(1195, 22, RoadSegmentType.INTERSECTION)
NH_intersection_8 = Point(1195, 993, RoadSegmentType.INTERSECTION)
NH_intersection_9 = Point(89, 993, RoadSegmentType.INTERSECTION)
NH_intersection_10 = Point(645, 993, RoadSegmentType.INTERSECTION)

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

def find_edge_weight(point1: Point, point2: Point) -> float:
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

NH_edges = [
    (NH_startpoint, NH_intersection_1, {"weight": find_edge_weight(NH_startpoint, NH_intersection_1)}),
    (NH_startpoint, NH_intersection_4, {"weight": find_edge_weight(NH_startpoint, NH_intersection_4)}),
    (NH_startpoint, NH_intersection_6, {"weight": find_edge_weight(NH_startpoint, NH_intersection_6)}),
    (NH_startpoint, NH_intersection_10, {"weight": find_edge_weight(NH_startpoint, NH_intersection_10)}),
    (NH_intersection_1, NH_intersection_2, {"weight": find_edge_weight(NH_intersection_1, NH_intersection_2)}),
    (NH_intersection_2, NH_intersection_3, {"weight": find_edge_weight(NH_intersection_2, NH_intersection_3)}),
    (NH_intersection_2, NH_intersection_4, {"weight": find_edge_weight(NH_intersection_2, NH_intersection_4)}),
    (NH_intersection_3, NH_intersection_5, {"weight": find_edge_weight(NH_intersection_3, NH_intersection_5)}),
    (NH_intersection_4, NH_intersection_2, {"weight": find_edge_weight(NH_intersection_4, NH_intersection_2)}),
    (NH_intersection_4, NH_intersection_5, {"weight": find_edge_weight(NH_intersection_4, NH_intersection_5)}),
    (NH_intersection_5, NH_intersection_7, {"weight": find_edge_weight(NH_intersection_5, NH_intersection_7)}),
    (NH_intersection_6, NH_intersection_7, {"weight": find_edge_weight(NH_intersection_6, NH_intersection_7)}),
    (NH_intersection_6, NH_intersection_8, {"weight": find_edge_weight(NH_intersection_6, NH_intersection_8)}),
    (NH_intersection_1, NH_intersection_9, {"weight": find_edge_weight(NH_intersection_1, NH_intersection_9)})
]

C_edges = [
    (C_startpoint, C_point1, {"weight": find_edge_weight(C_startpoint, C_point1)}),
    (C_point1, C_point2, {"weight": find_edge_weight(C_point1, C_point2)}),
    (C_point2, C_point3, {"weight": find_edge_weight(C_point2, C_point3)}),
    (C_point3, C_point4, {"weight": find_edge_weight(C_point3, C_point4)}),
    (C_point4, C_point5, {"weight": find_edge_weight(C_point4, C_point5)}),
    (C_point5, C_point6, {"weight": find_edge_weight(C_point5, C_point6)}),
    (C_point6, C_point7, {"weight": find_edge_weight(C_point6, C_point7)}),
    (C_point7, C_point8, {"weight": find_edge_weight(C_point7, C_point8)}),
    (C_point8, C_point9, {"weight": find_edge_weight(C_point8, C_point9)}),
    (C_point1, C_point10, {"weight": find_edge_weight(C_point1, C_point10)}),
    (C_point10, C_point11, {"weight": find_edge_weight(C_point10, C_point11)}),
    (C_point11,	C_point12, {"weight": find_edge_weight(C_point11, C_point12)}),
    (C_point12,	C_point13, {"weight": find_edge_weight(C_point12, C_point13)}),
    (C_point13,	C_point14, {"weight": find_edge_weight(C_point13, C_point14)}),
    (C_point14,	C_point15, {"weight": find_edge_weight(C_point14, C_point15)}),
    (C_point15,	C_point16, {"weight": find_edge_weight(C_point15, C_point16)}),
    (C_point16,	C_point17, {"weight": find_edge_weight(C_point16, C_point17)}),
    (C_point17,	C_point18, {"weight": find_edge_weight(C_point17, C_point18)}),
    (C_point18,	C_point19, {"weight": find_edge_weight(C_point18, C_point19)}),
    (C_point19,	C_point20, {"weight": find_edge_weight(C_point19, C_point20)}),
    (C_point20,	C_point21, {"weight": find_edge_weight(C_point20, C_point21)}),
    (C_point21,	C_point22, {"weight": find_edge_weight(C_point21, C_point22)}),
    (C_point22,	C_point23, {"weight": find_edge_weight(C_point22, C_point23)}),
    (C_point23,	C_point24, {"weight": find_edge_weight(C_point23, C_point24)}),
    (C_point24,	C_point25, {"weight": find_edge_weight(C_point24, C_point25)}),
    (C_point25,	C_point26, {"weight": find_edge_weight(C_point25, C_point26)}),
    (C_point26, C_point27, {"weight": find_edge_weight(C_point26, C_point27)}),
    (C_point9, C_point27, {"weight": find_edge_weight(C_point9, C_point27)}),
    (C_point27, C_point28, {"weight": find_edge_weight(C_point27, C_point28)}),
    (C_point3, C_point29, {"weight": find_edge_weight(C_point3, C_point29)}),
    (C_point29, C_point30, {"weight": find_edge_weight(C_point29, C_point30)}),
    (C_point30, C_point31, {"weight": find_edge_weight(C_point30, C_point31)}),
    (C_point31, C_point32, {"weight": find_edge_weight(C_point31, C_point32)}),
    (C_point32, C_point33, {"weight": find_edge_weight(C_point32, C_point33)}),
    (C_point33, C_point34, {"weight": find_edge_weight(C_point33, C_point34)}),
    (C_point34,	C_point35, {"weight": find_edge_weight(C_point34, C_point35)}),
    (C_point35,	C_point36, {"weight": find_edge_weight(C_point35, C_point36)}),
    (C_point36,	C_point37, {"weight": find_edge_weight(C_point36, C_point37)}),
    (C_point37,	C_point38, {"weight": find_edge_weight(C_point37, C_point38)}),
    (C_point38,	C_point39, {"weight": find_edge_weight(C_point38, C_point39)}),
    (C_point36, C_point40, {"weight": find_edge_weight(C_point36, C_point40)}),
    (C_point40,	C_point41, {"weight": find_edge_weight(C_point40, C_point41)}),
    (C_point41,	C_point42, {"weight": find_edge_weight(C_point41, C_point42)}),
    (C_point42,	C_point43, {"weight": find_edge_weight(C_point42, C_point43)}),
    (C_point43,	C_point44, {"weight": find_edge_weight(C_point43, C_point44)}),
    (C_point44,	C_point45, {"weight": find_edge_weight(C_point44, C_point45)}),
    (C_point45,	C_point46, {"weight": find_edge_weight(C_point45, C_point46)}),
    (C_point46,	C_point47, {"weight": find_edge_weight(C_point46, C_point47)}),
    (C_point47,	C_point48, {"weight": find_edge_weight(C_point47, C_point48)}),
    (C_point48,	C_point49, {"weight": find_edge_weight(C_point48, C_point49)}),
    (C_point14, C_point50, {"weight": find_edge_weight(C_point14, C_point50)}),
    (C_point50,	C_point51, {"weight": find_edge_weight(C_point50, C_point51)}),
    (C_point51,	C_point52, {"weight": find_edge_weight(C_point51, C_point52)}),
    (C_point52,	C_point53, {"weight": find_edge_weight(C_point52, C_point53)}),
    (C_point53,	C_point54, {"weight": find_edge_weight(C_point53, C_point54)}),
    (C_point54,	C_point55, {"weight": find_edge_weight(C_point54, C_point55)}),
    (C_point55,	C_point56, {"weight": find_edge_weight(C_point55, C_point56)}),
    (C_point56,	C_point57, {"weight": find_edge_weight(C_point56, C_point57)}),
    (C_point57,	C_point58, {"weight": find_edge_weight(C_point57, C_point58)}),
    (C_point58,	C_point59, {"weight": find_edge_weight(C_point58, C_point59)}),
    (C_point59,	C_point60, {"weight": find_edge_weight(C_point59, C_point60)}),
    (C_point60, C_point61, {"weight": find_edge_weight(C_point60, C_point61)}),
    (C_point60, C_point62, {"weight": find_edge_weight(C_point60, C_point61)}),
    (C_point51, C_point63, {"weight": find_edge_weight(C_point51, C_point63)}),
    (C_point47, C_point63, {"weight": find_edge_weight(C_point47, C_point63)}),
    (C_point20, C_point64, {"weight": find_edge_weight(C_point20, C_point64)}),
    (C_point64, C_point65, {"weight": find_edge_weight(C_point64, C_point65)}),
    (C_point65, C_point66, {"weight": find_edge_weight(C_point65, C_point66)}),
    (C_point22, C_point67, {"weight": find_edge_weight(C_point22, C_point67)}),
    (C_point67,	C_point68, {"weight": find_edge_weight(C_point67, C_point68)}),
    (C_point68,	C_point69, {"weight": find_edge_weight(C_point68, C_point69)}),
    (C_point69,	C_point70, {"weight": find_edge_weight(C_point69, C_point70)}),
    (C_point70,	C_point71, {"weight": find_edge_weight(C_point70, C_point71)}),
    (C_point71,	C_point72, {"weight": find_edge_weight(C_point71, C_point72)}),
    (C_point72,	C_point73, {"weight": find_edge_weight(C_point72, C_point73)})
]

NH_G.add_edges_from(NH_edges)
C_G.add_edges_from(C_edges)

#used for iteration
NH_pointList = [NH_startpoint,
              NH_intersection_1,
              NH_intersection_2,
              NH_intersection_3,
              NH_intersection_4,
              NH_intersection_5,
              NH_intersection_6,
              NH_intersection_7
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
def find_shortest_path(graph: nx.Graph(), start_node: Point, end_node: Point) -> List:
    return nx.dijkstra_path(graph, start_node, end_node)

# Add destination node to graph
def add_dest_node(graph: nx.Graph(), end_node: Point) -> bool:
    closest_edge = None
    closest_distance = 100000
    end_point = Point2(end_node.x, end_node.y)

    for edge in graph.edges():
        pos1 = Point2(edge[0].x, edge[0].y)
        pos2 = Point2(edge[1].x, edge[1].y)
        edge_line = Segment(pos1, pos2)
        distance = edge_line.distance(end_point)
        if distance < closest_distance:
            closest_edge = edge
            closest_distance = distance

    #checks if chosen end-point is on the road
    if closest_distance < 20:
        canvas.delete("popup")
        graph.add_node(end_node)
        node1, node2 = closest_edge
        graph.remove_edge(node1, node2)
        graph.add_edge(node1, end_node, weight=find_edge_weight(node1, end_node))
        graph.add_edge(node2, end_node, weight=find_edge_weight(node2, end_node))
        return True
    else:
        canvas.delete("popup")
        canvas.create_text(1550, 100, text="Error: Click on the roads to select a location.", font=("Arial", 20), fill="red", tags="popup")
        return False


def straight_point_formula(start_point: Point, end_point: Point, i: int) -> Tuple[float, float]:
    distance = find_edge_weight(start_point, end_point)
    x = start_point.x + i * STRAIGHT_INTERVAL * (end_point.x - start_point.x) / distance
    y = start_point.y + i * STRAIGHT_INTERVAL * (end_point.y - start_point.y) / distance
    return x, y

# creates points along a straight path
def generate_straight_points(start_point: Point, end_point: Point) -> List:
    distance = find_edge_weight(start_point, end_point)
    num_points_straight = int(distance / STRAIGHT_INTERVAL)

    # create points along straight road
    points_straight = []
    for j in range(2, num_points_straight):
        x, y = straight_point_formula(start_point, end_point, j)
        points_straight.append(Point(x, y, RoadSegmentType.STRAIGHT))
    return points_straight

# creates points along an intersection
def generate_curve(start_point: Point, mid_point: Point, end_point: Point) -> List[Point]:
    points = []
    for i in range(CURVE_POINTS):
        t = i / (CURVE_POINTS - 1)
        p = quadratic_bezier(t, start_point, mid_point, end_point)
        points.append(p)
    return points

def quadratic_bezier(t: float, p0: Point, p1: Point, p2: Point) -> Point:
    x = (1 - t) ** 2 * p0.x + 2 * (1 - t) * t * p1.x + t ** 2 * p2.x
    y = (1 - t) ** 2 * p0.y + 2 * (1 - t) * t * p1.y + t ** 2 * p2.y
    return Point(x, y, RoadSegmentType.INTERSECTION)

# optimize path by adding more points on straight roads and a curved path at intersections
def optimize_path(path: List[Point]) -> List[Point]:
    start_point = path[0]
    end_point = path[-1]
    second_last_end_point = path[-2]
    new_path = [start_point]

    for i, _ in enumerate(path[:-2]):
        p0 = path[i]
        p1 = path[i + 1]
        p2 = path[i + 2]
        straight_points = generate_straight_points(p0, p1)
        new_path.extend(straight_points)

        # create points along intersection
        if len(path) > 2:
            i0 = straight_points[-1]
            i1 = p1
            i2_x, i2_y = straight_point_formula(p1, p2, 1)
            i2 = Point(i2_x, i2_y, RoadSegmentType.STRAIGHT)
            curve_points = generate_curve(i0, i1, i2)
            new_path.extend(curve_points)

    straight_points = generate_straight_points(second_last_end_point, end_point)
    new_path.extend(straight_points)
    new_path.append(end_point)

    return new_path


# create path from clicking on canvas
def handle_canvas_m1(event):
    global map_model, current_image, dest_node
    x, y = event.x, event.y

    #delete line and path if already created
    canvas.delete("line", "dot")
    if not map_model.empty():
        map_model.delete_path(0)

    #delete destination node from graph when creating new path
    if not dest_node.x is None:
        if current_image == NH and NH_G.has_node(dest_node):
            neighbor_nodes = list(NH_G.neighbors(dest_node))
            NH_G.remove_node(dest_node)
            node1, node2 = neighbor_nodes
            NH_G.add_edge(node1, node2, weight=find_edge_weight(node1, node2))

        elif current_image == CITY and C_G.has_node(dest_node):
            neighbor_nodes = list(C_G.neighbors(dest_node))
            C_G.remove_node(dest_node)
            node1, node2 = neighbor_nodes
            C_G.add_edge(node1, node2, weight=find_edge_weight(node1, node2))

    #adding chosen node to graph
    dest_node = Point(x, y, RoadSegmentType.STRAIGHT)
    path: List[Point] = []
    if current_image == NH:
        if not add_dest_node(NH_G, dest_node):
            return
        path = find_shortest_path(NH_G, NH_startpoint, dest_node)
        path = optimize_path(path)
    elif current_image == CITY:
        if not add_dest_node(C_G, dest_node):
            return
        path = find_shortest_path(C_G, C_startpoint, dest_node)

    #create line on canvas showing generated path
    draw_line(path)

    #add path to map_model for navigation
    lane: Lane = Lane()
    lane.set_lane(path)
    map_model.add_path(lane)

# draws path onto canvas
def draw_line(path: List[Point]):
    canvas.delete("line", "dot")
    size = 3
    for i in range(len(path)):
        if i < len(path) - 1:
            canvas.create_line(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y, fill="red", width=3, tags="line")
        canvas.create_oval(path[i].x - size, path[i].y - size, path[i].x + size, path[i].y + size, fill='red',
                           outline='yellow', tags="dot")

# Handle load from file
def handle_load():
    global map_model
    map_model = MS.load_from_file()
    draw_line(map_model.get_path())

# Updates the canvas image
def update_canvas(img_choice: int):
    global current_image
    current_image = img_choice
    canvas.delete("line", "dot")
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
window.state("zoomed")
init_menu_bar()

# Canvas
img = [ImageTk.PhotoImage(Image.open(os.path.normpath(os.getcwd() + os.sep + os.pardir)+"/AirSimMaps/maps/City_Top.png")),
       ImageTk.PhotoImage(Image.open(os.path.normpath(os.getcwd() + os.sep + os.pardir)+"/AirSimMaps/maps/NH_Top.png"))]
h = img[0].height()
w = img[0].width()

canvas = tk.Canvas(window, width=w, height=h)
canvas_container = canvas.create_image(0, 0, image=img[1], anchor='nw')
canvas.pack(side="left", fill="both", expand=True)
# Canvas event handlers
canvas.bind("<Button-1>", handle_canvas_m1)
window.bind_all('<KeyPress-p>', print_converted_path)

window.mainloop()
