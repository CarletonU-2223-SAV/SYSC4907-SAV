#!/usr/bin/env python3

"""
Code adapted from

Path tracking simulation with pure pursuit steering and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import numpy as np
import math
from map_information import *
from Models import X_COORD, Y_COORD


class CarState:
    """
    Contains the car information needed in pure pursuit
    The cartesian coordinates for location
    The yaw of the car
    The cars wheel base
    """

    def __init__(self, wheel_base: float, point: Point = Point((0, 0)), yaw: float = 0.0, velocity: float = 0.0):
        self.wheel_base = wheel_base
        self.point = point
        self.yaw = yaw
        self.velocity = velocity
        self.rear_x = self.point.coordinate[X_COORD] - ((wheel_base / 2) * math.cos(self.yaw))
        self.rear_y = self.point.coordinate[Y_COORD] - ((wheel_base / 2) * math.sin(self.yaw))

    def update_pos(self, point: Point, quaternion: Tuple[float, float, float, float]):
        """
        Updates the cars position
        Cartesian coordinates for the cars position
        Quaternion to find the yaw of the car
        """
        self.point = point
        self.yaw = euler_from_quaternion(quaternion)
        self.rear_x = self.point.coordinate[X_COORD] - ((self.wheel_base / 2) * math.cos(self.yaw))
        self.rear_y = self.point.coordinate[Y_COORD] - ((self.wheel_base / 2) * math.sin(self.yaw))

    def update_speed(self, velocity: float):
        """
        Updates the cars speed
        """
        self.velocity = velocity

    def calc_distance(self, point_x: float, point_y: float) -> float:
        """
        Find the distance between the cars current location and another point
        Find the euclidean distance
        """
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class PurePursuit:
    """
    look ahead distance is the distance pure pursuit scans to find goals points
    look forward gain is used to increase the look ahead distance based on the cars current speed
    path is the path the algorithm must track
    """

    def __init__(self, look_ahead_distance: float, look_forward_gain: float, path: List[Point]):
        self.look_forward_gain = look_forward_gain
        self.look_ahead_distance = look_ahead_distance
        self.path = path
        self.old_nearest_point_index = None
        self.old_steer_target = 0

    def pure_pursuit_steer_control(self, car_state: CarState) -> Tuple[float, int]:
        """
        Returns the steering angle in radians and the index to the goal points its turning to
        Takes in the current state of the car
        """
        index, look_forward = self.search_target_index(car_state)

        # Checks to see if the previous found goal points is actually ahead of what it sees now
        if self.old_steer_target >= index:
            index = self.old_steer_target

        # Steering angle of the front wheel
        alpha = math.atan2(self.path[index].coordinate[Y_COORD] - car_state.rear_y,
                           self.path[index].coordinate[X_COORD] - car_state.rear_x) - car_state.yaw
        # Getting the ackerman steering angle, the 1 is to ensure the angle is the correct sign
        delta = math.atan2(2.0 * car_state.wheel_base * math.sin(alpha) / look_forward, 1)
        self.old_steer_target = index

        return delta, index

    def search_target_index(self, car_state: CarState) -> Tuple[int, float]:
        """
        Finds the next goal point given the cars state.
        Adjusts the looks forward distance based on the cars speed.
        """
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            # Mapping though all points in the path, and finding the difference in height and width to that point.
            dx = [car_state.rear_x - point.coordinate[X_COORD] for point in self.path]
            dy = [car_state.rear_y - point.coordinate[Y_COORD] for point in self.path]
            # Getting the minimum hypotenuse, aka the nearest point
            # For this first time set it to the closest point.
            distances = np.hypot(dx, dy)
            index = np.argmin(distances)
            self.old_nearest_point_index = index
        else:
            # Get distance between the nearest point we found last time and the cars current position.
            index = self.old_nearest_point_index
            distance_this_index = car_state.calc_distance(self.path[index].coordinate[X_COORD], self.path[index].coordinate[Y_COORD])
            while True:
                # Get the distance from the cars current position to the next point after the nearest point
                distance_next_index = car_state.calc_distance(self.path[index + 1].coordinate[X_COORD],
                                                              self.path[index + 1].coordinate[Y_COORD])
                # If the closest point is still the nearest point we found earlier exit the loop
                if distance_this_index < distance_next_index:
                    break
                # If the next next point is actually closer go to that instead and repeat the calculation
                index = index + 1 if (index + 1) < len(self.path) else index
                distance_this_index = distance_next_index
            self.old_nearest_point_index = index

        # Finds where we should actually be looking taking into around the vehicles current speed
        # look ahead distance=look ahead gain*vehicle forward velocity
        adjusted_look_forward = self.look_forward_gain * car_state.velocity + self.look_ahead_distance

        # Check and see if the speed adjusted look forward distance needs a further point
        while adjusted_look_forward > car_state.calc_distance(self.path[index].coordinate[X_COORD],
                                                              self.path[index].coordinate[Y_COORD]):
            if (index + 1) >= len(self.path):
                break
            index += 1

        return index, adjusted_look_forward


def euler_from_quaternion(quaternion: Tuple[float, float, float, float]) -> float:
    """
    Returns yaw from a quaternion
    Takes in a quaternion because Airsim provides orientation in that format.
    """
    x, y, z, w = quaternion

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw_z = math.atan2(siny_cosp, cosy_cosp)

    return yaw_z
