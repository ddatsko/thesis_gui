from typing import List, Tuple
import rospy
from thesis_path_generator.srv import CalculateEnergy, CalculateEnergyRequest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from .utils import gps_coordinates_to_meters
from .toppra_trajectory_generation.src.trajectory_generation_manager import TrajectoryGenerationManager2
from .toppra_trajectory_generation.src.path_manipulations import add_waypoints_with_distance
import numpy as np
import math

"""
NOTE: two methods of energy calculation are possible in this file.
Ab important thing to consider is the fact that they accept different parameters for constraints.
Own methods is based on maximum allowed path deviation metric while TOPPRA aims to visit all the equadistantly located poitns
without caring about maximum path deviation.
This means that there is no exact correspondence between two methods and parameters for them

Also, own method calculates the optimal speed and hover power consumption inside based on UAV parameters,
while these parameters need to be provided for toppra. To match and compare two methods, firstly 
calculate the path parameters by own method, then use calculated values for 
HOVER_POWER_CONSUMPTION, MAX_SPEED_POWER_CONSUMPTION and MAX_SPEED parameters
"""

ENERGY_CALCULATION_METHOD = 'own'
# Common parameters for own and TOPPRA
MAX_ACC = 2

# Parameters for own algorithm.
UAV_AREA = 0.07
UAV_MASS = 3.2
NUMBER_OF_PROPELLERS = 4
PROPELLER_RADIUS = 0.19
ALLOWED_PATH_DEVIATION = 2

# Parameters for TOPPRA energy estimation
SAMPLING_DT = 0.1
MAX_ACC_EPS = 0.2
MAX_SPEED = 8.1
MAX_SPEED_EPS = 0.2
HOVER_POWER_CONSUMPTION = 426.03
MAX_SPEED_POWER_CONSUMPTION = 465.23
DISTANCE_BETWEEN_WAYPOINTS = 20
# Max length of a path chunk. Used to split path into smaller chunks as toppra does not work well when therea re too many points
MAX_PATH_CHUNK_LENGTH = 10



def _calculate_paths_energies_ros(calculate_req):
    rospy.wait_for_service("/calculate_energy")
    proxy = rospy.ServiceProxy("/calculate_energy", CalculateEnergy)
    res = proxy(calculate_req)
    if not res.success:
        print(f"Unsuccessful service call: {res.message}")
        return []
    else:
        return res.energies


def _path_from_gps_coordinates_to_meters(path: List[Tuple[float, float]]) -> np.array:
    """
    Convert path from gps coordinates to meters.
    NOTE: each path point must be in (longitude, latitude) format
    :param path: Path as list of points
    :return: Path of the same length as list of points in metric coordinates
    """
    gps_origin_lat = path[0][1]
    gps_origin_lon = path[0][0]
    res = []
    for p in path:
        res.append(gps_coordinates_to_meters(p[1], p[0], gps_origin_lat, gps_origin_lon))
    return np.array(res)


def get_path_properties(path: List[Tuple[float, float, float]]):
    original_path_length = len(path)
    if ENERGY_CALCULATION_METHOD == 'toppra':
        if len(path) > MAX_PATH_CHUNK_LENGTH:
            chunks_num = math.ceil(len(path) / MAX_PATH_CHUNK_LENGTH)
            chunk_size = math.ceil(len(path) / chunks_num)

            cur_idx = 0
            waypoints_chunks = []
            while cur_idx < len(path) - 1:
                last_chunk_idx = min(cur_idx + chunk_size, len(path) - 1)
                waypoints_chunks.append(path[cur_idx:last_chunk_idx + 1])
                cur_idx = last_chunk_idx
        else:
            waypoints_chunks = [path]

        total_energy = 0
        for path in waypoints_chunks:

            # Create trajectory generation manager and fill it with parameters
            manager = TrajectoryGenerationManager2(4)
            manager.max_acc = MAX_ACC - MAX_ACC_EPS
            manager.max_acc_eps = MAX_ACC_EPS
            manager.max_speed = MAX_SPEED
            manager.max_speed_eps = MAX_SPEED_EPS
            manager.max_vert_acc = 1
            manager.max_vert_speed = 2
            manager.max_heading_acc = 1
            manager.max_heading_speed = 2

            path = _path_from_gps_coordinates_to_meters(path)
            path = np.append(path, np.zeros((path.shape[0], 2)), axis=1)

            path_equidist = add_waypoints_with_distance(path, DISTANCE_BETWEEN_WAYPOINTS)
            # Generate and sample the trajectory
            trajectory = manager.plan_trajectory(path_equidist)
            ts_sample = np.arange(0, trajectory.duration, SAMPLING_DT)

            qds_sample = trajectory(ts_sample, 1)

            res_energy = 0.0
            prev_x_energy = 0.0
            prev_y_energy = 0.0

            for (v_x, v_y, _, _) in qds_sample:
                x_energy = UAV_MASS * v_x * v_x / 2
                y_energy = UAV_MASS * v_y * v_y / 2

                res_energy += abs(prev_x_energy - x_energy) + abs(prev_y_energy - y_energy)
                prev_x_energy = x_energy
                prev_y_energy = y_energy

                total_speed = (v_x * v_x + v_y + v_y) ** 0.5

                # If the speed is close enough to max speed, assume power consumption on optimal speed
                if total_speed > MAX_SPEED - 1:
                    power = MAX_SPEED_POWER_CONSUMPTION
                else:
                    power = HOVER_POWER_CONSUMPTION
                res_energy += power + SAMPLING_DT
            total_energy += res_energy
        return total_energy, 0, 0, original_path_length

    elif ENERGY_CALCULATION_METHOD == 'own':
        calculation_req = CalculateEnergyRequest()
        calculation_req.override_drone_parameters = True
        calculation_req.drone_area = UAV_AREA
        calculation_req.drone_mass = UAV_MASS
        calculation_req.number_of_propellers = NUMBER_OF_PROPELLERS
        calculation_req.propeller_radius = PROPELLER_RADIUS
        calculation_req.average_acceleration = MAX_ACC
        calculation_req.allowed_path_deviation = ALLOWED_PATH_DEVIATION
        calculation_req.paths = [Path()]
        for p in path:
            calculation_req.paths[0].poses.append(PoseStamped())
            calculation_req.paths[0].poses[-1].pose.position.x = p[1]
            calculation_req.paths[0].poses[-1].pose.position.y = p[0]
        calculation_req.paths[0].header.frame_id = "latlon_origin"
        ros_calculation_res = _calculate_paths_energies_ros(calculation_req)
        return ros_calculation_res[0], 0, 0, original_path_length
