#!/usr/bin/env python3

from enum import Enum
from typing import List, Tuple

X_COORD = 0
Y_COORD = 1
NH = 0
CITY = 1

class RoadSegmentType(Enum):
    STRAIGHT = 0
    TURN = 1
    INTERSECTION = 2

# Model a point on the canvas
class Point:
    def __init__(self, x, y, seg_type: RoadSegmentType):
        self.x = x
        self.y = y
        self.seg_type = seg_type

    def point_to_gui_coords(self, map_choice: int) -> Tuple[float, float]:
        # weird inversion correction due to the AirSim map being "upside-down"
        x, y = self.y, -self.x

        # scale correction
        c1 = (x * MapModel.AirSim_scale_factor[map_choice],
              y * MapModel.AirSim_scale_factor[map_choice])
        # re-centering correction factor
        c2 = (c1[X_COORD] - MapModel.AirSim_correction_factor[map_choice][X_COORD],
              c1[Y_COORD] - MapModel.AirSim_correction_factor[map_choice][Y_COORD])

        return c2

    def __str__(self):
        return f'{(self.x, self.y, self.seg_type)}'

    def __repr__(self):
        return self.__str__()

class Lane:
    def __init__(self, points: List[Point] = None):
        self.points: List[Point] = points

    def __str__(self):
        return f'{self.points}'

    def __repr__(self):
        return f'{self.points}'

    def add_point(self, p: Point):
        self.points.append(p)

    def remove_point(self, point_id: int):
        # I hate list comprehensions
        self.points = [e for e in self.points if e.pid != point_id]

    def set_lane(self, path: List[Point]):
        self.points = path

    def empty(self):
        return not self.points

    def get_gui_coords(self) -> List[Tuple[float, float]]:
        out = [(point.x, point.y) for point in self.points]
        return out

class MapModel:
    # Semantic versioning: Major.Minor.Patch
    version: str = '0.1.0'
    AirSim_scale_factor: Tuple[float] = [4.33564, 2.046295]  # Calculated using collected points on the map
    AirSim_correction_factor: List[List[int]] = [[-645, -573], [-614, -589]]  # Start points of vehicles in environments

    def __init__(self):
        self.paths: List[Lane] = []
        self.instance_version: str = '0.1.0'

    # Add new path to model
    def add_path(self, new_path: Lane):
        self.paths.append(new_path)

    def get_path(self) -> List[Point]:
        return self.paths[0].points

    def delete_path(self, index):
        del self.paths[index]

    def empty(self):
        return not self.paths

    # Use NH/City map correction factors.
    def convert_point(self, point: Point, map_choice: int) -> Tuple[float, float]:
        # re-centering correction factor
        c1 = (point.x + MapModel.AirSim_correction_factor[map_choice][X_COORD],
              point.y + MapModel.AirSim_correction_factor[map_choice][Y_COORD])
        # scale correction
        c2 = (c1[X_COORD] / MapModel.AirSim_scale_factor[map_choice],
              c1[Y_COORD] / MapModel.AirSim_scale_factor[map_choice])
        # weird inversion correction due to the AirSim map being "upside-down"
        return -c2[Y_COORD], c2[X_COORD]

    # Convert path to AirSim coords
    # TODO: include the segment type information somewhere...
    def convert_path(self, index) -> List[Tuple[float, float, RoadSegmentType]]:
        airsim_path: List[Tuple[float, float, RoadSegmentType]] = []
        sel_path = self.paths[index]
        # Empty path
        if sel_path.empty():
            pass

        # choose between AirSim environments based off starting point of given path
        # and start point of vehicle in environment
        x_points_path = [point.x for point in sel_path.points]
        x_startpoint_path = x_points_path[0]
        x_startpoint_nh = -1 * MapModel.AirSim_correction_factor[NH][X_COORD]
        x_startpoint_city = -1 * MapModel.AirSim_correction_factor[CITY][X_COORD]

        if x_startpoint_path == x_startpoint_nh:
            map_choice = NH
        elif x_startpoint_path == x_startpoint_city:
            map_choice = CITY
        else:
            map_choice = NH

        # Iterate through points in the path
        for  p in sel_path.points:
            converted_point = self.convert_point(p, map_choice)

            segment_type = p.seg_type
            airsim_path.append((converted_point[X_COORD], converted_point[Y_COORD], segment_type))

        return airsim_path
