import subprocess
from geometry_msgs.msg import Point32
from math import cos, pi
from typing import List, Tuple
import os
import rosservice
import rospy
from mrs_msgs.srv import PathSrv, PathSrvRequest

METERS_IN_DEGREE = 111319.5


class PathProperties:
    def __init__(self, energy, time):
        self.energy = energy
        self.time = time


def gps_coordinates_to_meters(x, y, origin_x, origin_y):
    meters_in_long_degree = cos((origin_x / 180) * pi) * METERS_IN_DEGREE
    return y * meters_in_long_degree - origin_y * meters_in_long_degree, \
        x * METERS_IN_DEGREE - origin_x * METERS_IN_DEGREE


def meters_to_gps_coordinates(x, y, origin_x, origin_y):
    meters_in_long_degree = cos((origin_x / 180) * pi) * METERS_IN_DEGREE
    return (y + origin_x * METERS_IN_DEGREE) / METERS_IN_DEGREE, \
           (x + origin_y * meters_in_long_degree) / meters_in_long_degree


def meter_point_from_gps(x, y, origin_x, origin_y) -> Point32:
    new_x, new_y = gps_coordinates_to_meters(x, y, origin_x, origin_y)
    return Point32(new_x, new_y, 20)


def gps_point_from_meters(x, y, origin_x, origin_y) -> (float, float):
    new_x, new_y = meters_to_gps_coordinates(x, y, origin_x, origin_y)
    return new_y, new_x


def get_path_properties(path: List[Tuple[float, float, float]]):
    TEMP_CSV_FILE = ".__temp.csv"
    if os.path.exists(TEMP_CSV_FILE):
        os.remove(TEMP_CSV_FILE)
    with open(TEMP_CSV_FILE, 'w') as f:
        for p in path:
            print(f'{p[0]},{p[1]},', file=f)

    output = subprocess.check_output(['./energy_calculation', TEMP_CSV_FILE])
    return tuple(map(float, output.decode('utf-8').split('\n')[-2].split(',')))


def send_path_to_service(path, service):
    service_list = rosservice.get_service_list()
    if service not in service_list:
        return False, "Service not in service list"
    try:
        follow_path = rospy.ServiceProxy(service, PathSrv)
        path_srv = PathSrvRequest(path)
        res = follow_path(path_srv)
        if not res.success:
            return False, res.message
        else:
            return True, res.message
    except Exception as e:
        return False, "Error while calling service: " + str(e)
